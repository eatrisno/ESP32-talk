// ESP32-S3 mesh walkie-talkie: USB-C headset, ESP-NOW, every node in range joins one
// conference automatically. See README.md.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

#include "audio.hpp"
#include "button.hpp"
#include "config.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hid_buttons.hpp"
#include "led.hpp"
#include "nvs.h"
#include "driver/temperature_sensor.h"
#include "nvs_flash.h"
#include "radio.hpp"
#include "range_test.hpp"
#include "usb_headset.hpp"
#include "voice_gate.hpp"

namespace {

const char* TAG = "app";

char s_name[12] = "";
nvs_handle_t s_nvs;
// Guards s_name, s_ping_* and the talk toggle: app task, console task and radio task use them.
SemaphoreHandle_t s_app_mtx;
volatile uint8_t s_hid_down = 0;
uint32_t s_ping_token = 0, s_ping_ms = 0;

static uint32_t s_range_token = 0;
static wt::RangeStats s_range_stats;
static volatile bool s_range_active = false;
static volatile bool s_walk_mode = false;
static volatile bool s_walk_pending = false;
static uint32_t s_walk_probe_sent_ms = 0;
static uint16_t s_walk_seq = 0;

static volatile bool s_distress_active = false;
static volatile uint32_t s_distress_peer = 0;
static char s_distress_peer_name[12] = "";
static uint32_t s_last_distress_tx_ms = 0;
static uint32_t s_last_distress_beep_ms = 0;
static temperature_sensor_handle_t s_temp_sensor = nullptr;

uint32_t now_ms() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

// --- persistent settings ---------------------------------------------------------------

uint8_t nvs_u8(const char* k, uint8_t def) {
    uint8_t v = def;
    nvs_get_u8(s_nvs, k, &v);
    return v;
}
void nvs_put_u8(const char* k, uint8_t v) {
    nvs_set_u8(s_nvs, k, v);
    nvs_commit(s_nvs);
}

// The session is half of the CCM nonce (sender|session|seq), so it must never repeat under
// the same key. A boot counter alone repeats after a flash erase or NVS reset, so the first
// value after such a reset is random in [2^30, 2^31): another erase picks a new random start,
// and a collision with any earlier session is about 1 in 10^9 per erase. Returns 0 if the new
// value could not be stored; the radio must then stay off, or the next boot would reuse it.
uint32_t next_session() {
    uint32_t s = 0;
    if (nvs_get_u32(s_nvs, "session", &s) != ESP_OK || s == 0)
        s = (1u << 30) + (esp_random() & ((1u << 30) - 1));
    s++;
    if (nvs_set_u32(s_nvs, "session", s) != ESP_OK || nvs_commit(s_nvs) != ESP_OK) return 0;
    return s;
}

// --- feedback --------------------------------------------------------------------------

// Soft, gentle "pi.." feedback
void chime_join() { audio::tone(550, 45, 450); }
void chime_leave() { audio::tone(420, 45, 400); }
void chime_peer_joined() { audio::tone(550, 30, 400); }
void chime_peer_left() { audio::tone(400, 30, 350); }

void sound_distress_alert() {
    // Gentle, non-deafening distress tone: warm two-tone 620Hz (130ms), gap 30ms, 480Hz (170ms) at amplitude 380
    audio::tone(620, 130, 380);
    audio::tone(0, 30);
    audio::tone(480, 170, 380);
}

void sound_all_clear() {
    // Gentle soothing all-clear chime: rising 450Hz (70ms) -> 600Hz (100ms)
    audio::tone(450, 70, 400);
    audio::tone(0, 20);
    audio::tone(600, 100, 400);
}

void update_led() {
    using led::Color;
    if (s_distress_active || s_distress_peer != 0) {
        static uint32_t s_last_distress_flash = 0;
        static bool s_red_on = false;
        uint32_t now = now_ms();
        if (now - s_last_distress_flash >= 250) {
            s_last_distress_flash = now;
            s_red_on = !s_red_on;
        }
        led::set(s_red_on ? Color::Red : Color::Off);
        return;
    }

    static bool s_was_sending = false;
    static bool s_was_receiving = false;
    bool sending = audio::sending();
    bool receiving = audio::receiving();

    // Subtle edge confirmation flashes: 1 short flash at start, then OFF
    if (sending && !s_was_sending) {
        led::flash(Color::Red, 70);    // 1 quick red blip when you start transmitting
    }
    if (receiving && !s_was_receiving) {
        led::flash(Color::Green, 70);  // 1 quick green blip when incoming voice begins
    }
    s_was_sending = sending;
    s_was_receiving = receiving;

    if (audio::loopback()) led::set(Color::White);
    else if (!usb_headset::spk_ready()) led::set(Color::Blue);
    else led::set(Color::Off);  // silent & dark while speaking and listening
}

void send_distress_packet(bool active) {
    wt::DistressPayload dp;
    dp.sender = radio::self_id();
    dp.active = active ? 1 : 0;
    xSemaphoreTake(s_app_mtx, portMAX_DELAY);
    strncpy(dp.name, s_name, sizeof(dp.name) - 1);
    dp.name[sizeof(dp.name) - 1] = 0;
    xSemaphoreGive(s_app_mtx);
    radio::send(wt::MSG_DISTRESS, reinterpret_cast<const uint8_t*>(&dp), sizeof(dp));
}

void toggle_distress() {
    s_distress_active = !s_distress_active;
    send_distress_packet(s_distress_active);
    if (s_distress_active) {
        printf("\n*** [DISTRESS ACTIVATED] Emergency SOS broadcasting! Click button to cancel. ***\n\n");
        sound_distress_alert();
    } else {
        printf("\n*** [DISTRESS CANCELLED] All-clear broadcasted. ***\n\n");
        sound_all_clear();
    }
}

// --- actions ---------------------------------------------------------------------------

// Name + talking flag, as sent in BEACON / JOIN / LEAVE. Caller holds s_app_mtx.
void state_payload(uint8_t* p) {
    p[0] = audio::talking();
    memcpy(p + 1, s_name, sizeof s_name);
}

void broadcast_state() {
    uint8_t p[1 + sizeof s_name];
    xSemaphoreTake(s_app_mtx, portMAX_DELAY);
    state_payload(p);
    xSemaphoreGive(s_app_mtx);
    radio::send(p[0] ? wt::MSG_JOIN : wt::MSG_LEAVE, p, sizeof p);
}

void toggle_talk() {
    xSemaphoreTake(s_app_mtx, portMAX_DELAY);  // button and console can press at once
    bool on = !audio::talking();
    audio::set_talking(on);
    xSemaphoreGive(s_app_mtx);
    on ? chime_join() : chime_leave();
    broadcast_state();
    ESP_LOGI(TAG, "mic %s", on ? "OPEN" : "muted");
}

void send_ping() {
    uint32_t tok = static_cast<uint32_t>(esp_timer_get_time());
    xSemaphoreTake(s_app_mtx, portMAX_DELAY);
    s_ping_token = tok;
    s_ping_ms = now_ms();
    xSemaphoreGive(s_app_mtx);
    radio::send(wt::MSG_PING, reinterpret_cast<uint8_t*>(&tok), 4);
    audio::tone(650, 30, 400);
    printf("ping sent\n");
}

void volume_step(int d) {
    int v = audio::volume() + d;
    bool limit = v < 0 || v > 100;
    v = v < 0 ? 0 : (v > 100 ? 100 : v);
    audio::set_volume(static_cast<uint8_t>(v));
    nvs_put_u8("vol", static_cast<uint8_t>(v));
    printf("Volume: %d%%\n", v);
    // Soft, pleasant volume feedback: gentle pip (350 amp), warm frequencies (550Hz up, 420Hz down)
    uint16_t freq = limit ? (d > 0 ? 650 : 350) : (d > 0 ? 550 : 420);
    audio::tone(freq, 25, 350);
}

void radar_check() {
    uint32_t now = now_ms();
    struct PeerInfo {
        char name[12];
        int8_t rssi;
        uint8_t hops;
        uint32_t age_s;
    };
    PeerInfo active[wt::kMaxPeers];
    int count = 0;
    int best_rssi = -128;
    bool has_direct = false;

    radio::peers_lock();
    for (const wt::Peer* p = radio::peers().begin(); p != radio::peers().end(); ++p) {
        if (!p->used) continue;
        uint32_t age = (now - p->last_ms) / 1000;
        if (age > 15) continue;  // active within 15 seconds
        PeerInfo& pi = active[count++];
        memcpy(pi.name, p->name, sizeof pi.name);
        pi.rssi = p->rssi;
        pi.hops = p->hops;
        pi.age_s = age;
        if (p->hops == 0) {
            has_direct = true;
            if (p->rssi > best_rssi) best_rssi = p->rssi;
        }
    }
    radio::peers_unlock();

    printf("\n[Radar] %d peer(s) connected:\n", count);
    for (int i = 0; i < count; i++) {
        const char* dist = "far";
        if (active[i].hops == 0) {
            if (active[i].rssi >= -70) dist = "close / strong (beep panjang)";
            else if (active[i].rssi >= -88) dist = "medium / mid-range (beep sedang)";
            else dist = "far / weak (beep pendek)";
        } else {
            dist = "relayed via mesh (beep pendek)";
        }
        printf("  - %-11s %s  RSSI %4d dBm  (%s)\n",
               active[i].name[0] ? active[i].name : "?",
               active[i].hops ? "mesh relay" : "direct    ",
               active[i].rssi, dist);
    }
    if (count == 0) printf("  No peers in range.\n\n");
    else printf("\n");

    // Audio acoustic telemetry in headset:
    // 1. Tell peer count with N distinct pips
    // 2. Pause
    // 3. Tell distance with intuitive beep duration
    if (count == 0) {
        // 0 peers: soft low descending tone (350Hz 80ms, gap 20ms, 250Hz 100ms)
        audio::tone(350, 80, 380);
        audio::tone(0, 20);
        audio::tone(250, 100, 380);
    } else {
        // Step 1: N clear distinct pips for peer count (550Hz, 45ms each, gap 85ms)
        int pips = count > 5 ? 5 : count;
        for (int i = 0; i < pips; i++) {
            audio::tone(550, 45, 380);
            audio::tone(0, 85);
        }
        // Step 2: Pause 220ms so count is clearly separated from distance
        audio::tone(0, 220);
        // Step 3: Distance / signal strength beep length
        if (has_direct && best_rssi >= -70) {
            // Signal Kuat: BEEP PANJANG (620Hz, 350ms)
            audio::tone(620, 350, 400);
        } else if (has_direct && best_rssi >= -88) {
            // Signal Sedang: BEEP SEDANG (540Hz, 160ms)
            audio::tone(540, 160, 380);
        } else {
            // Signal Lemah / Multi-hop: BEEP PENDEK (440Hz, 50ms)
            audio::tone(440, 50, 360);
        }
    }
}

// --- voice mail ------------------------------------------------------------------------

struct VmFrame {
    int16_t adpcm_pred;
    int8_t  adpcm_index;
    uint8_t data_len;
    uint8_t data[wt::kAdpcmBytes]; // 160 bytes
};

struct VmStorage {
    uint32_t vm_id = 0;
    uint16_t total_frames = 0;
    char name[12] = {};
    VmFrame frames[250]; // up to 10 seconds (250 * 40ms = 10s)
};

static VmStorage* s_vm_storage = nullptr;
static bool s_vm_recording = false;
static bool s_vm_outbox_pending = false;
static uint32_t s_last_played_vm_id = 0;

void chime_voicemail_prompt() {
    // Rising 3-tone record start prompt: 440Hz -> 660Hz -> 880Hz
    audio::tone(440, 40, 380);
    audio::tone(0, 20);
    audio::tone(660, 40, 380);
    audio::tone(0, 20);
    audio::tone(880, 60, 400);
}

void chime_voicemail_send() {
    // Rising confirmation swoosh: 600Hz -> 900Hz
    audio::tone(600, 45, 380);
    audio::tone(0, 20);
    audio::tone(900, 75, 420);
}

void chime_voicemail_received() {
    // Distinct answering-machine / dispatcher chime: C5 (523Hz) -> E5 (659Hz) -> G5 (784Hz)
    audio::tone(523, 70, 400);
    audio::tone(0, 30);
    audio::tone(659, 70, 400);
    audio::tone(0, 30);
    audio::tone(784, 100, 420);
}

void on_voicemail_captured_frame(const wt::AudioPayload& a) {
    if (!s_vm_recording || !s_vm_storage) return;
    if (s_vm_storage->total_frames >= 250) {
        // 10 seconds maximum reached: stop recording automatically
        s_vm_recording = false;
        audio::set_recording_vm(false);
        audio::tone(700, 50, 400);
        printf("[Voice Mail] Max 10s recording reached. Press 1x to send.\n");
        return;
    }
    VmFrame& f = s_vm_storage->frames[s_vm_storage->total_frames++];
    f.adpcm_pred = a.adpcm_pred;
    f.adpcm_index = static_cast<int8_t>(a.adpcm_index);
    f.data_len = a.data_len > wt::kAdpcmBytes ? wt::kAdpcmBytes : a.data_len;
    memcpy(f.data, a.data, f.data_len);
}

void start_voicemail_recording() {
    if (s_vm_recording) return;
    if (!s_vm_storage) {
        s_vm_storage = static_cast<VmStorage*>(malloc(sizeof(VmStorage)));
        if (!s_vm_storage) {
            ESP_LOGE(TAG, "Not enough RAM for voice mail recording");
            audio::tone(250, 80, 400);
            return;
        }
    }
    s_vm_storage->vm_id = esp_random();
    s_vm_storage->total_frames = 0;
    xSemaphoreTake(s_app_mtx, portMAX_DELAY);
    memcpy(s_vm_storage->name, s_name, sizeof(s_vm_storage->name));
    xSemaphoreGive(s_app_mtx);

    s_vm_recording = true;
    audio::set_voicemail_cb(on_voicemail_captured_frame);
    audio::set_recording_vm(true);

    chime_voicemail_prompt();
    led::flash(led::Color::Amber, 200);
    printf("\n*** [VOICE MAIL RECORDING] Speak your message now (max 10s). Click button 1x when done to SEND. ***\n\n");
}

void flush_voicemail_to_mesh() {
    if (!s_vm_storage || s_vm_storage->total_frames == 0) return;
    uint16_t total = s_vm_storage->total_frames;
    printf("[Voice Mail] Broadcasting %u frames (%.1fs) to mesh...\n", total, total * 0.04f);
    for (uint16_t i = 0; i < total; i++) {
        wt::VoicemailPayload vp;
        vp.vm_id = s_vm_storage->vm_id;
        vp.frame_index = i;
        vp.total_frames = total;
        memcpy(vp.name, s_vm_storage->name, sizeof(vp.name));
        vp.adpcm_pred = s_vm_storage->frames[i].adpcm_pred;
        vp.adpcm_index = s_vm_storage->frames[i].adpcm_index;
        vp.data_len = s_vm_storage->frames[i].data_len;
        memcpy(vp.data, s_vm_storage->frames[i].data, vp.data_len);

        radio::send(wt::MSG_VOICEMAIL, reinterpret_cast<const uint8_t*>(&vp), sizeof(vp));
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    printf("[Voice Mail] Transmission complete.\n");
    free(s_vm_storage);
    s_vm_storage = nullptr;
    s_vm_outbox_pending = false;
}

void send_voicemail() {
    if (s_vm_recording) {
        s_vm_recording = false;
        audio::set_recording_vm(false);
    }
    if (!s_vm_storage || s_vm_storage->total_frames == 0) {
        printf("[Voice Mail] No audio recorded.\n");
        if (s_vm_storage) { free(s_vm_storage); s_vm_storage = nullptr; }
        return;
    }
    chime_voicemail_send();

    uint32_t now = now_ms();
    int active_peers = 0;
    radio::peers_lock();
    for (const wt::Peer* p = radio::peers().begin(); p != radio::peers().end(); ++p) {
        if (p->used && (now - p->last_ms <= 15000)) active_peers++;
    }
    radio::peers_unlock();

    if (active_peers > 0) {
        flush_voicemail_to_mesh();
        led::flash(led::Color::Cyan, 120);
        audio::tone(800, 30, 350);
    } else {
        s_vm_outbox_pending = true;
        audio::tone(500, 60, 380);
        audio::tone(0, 30);
        audio::tone(400, 80, 380);
        printf("[Voice Mail] Out of range (0 peers). Saved to outbox (%u frames, %.1fs). Will auto-send when a node is detected!\n",
               s_vm_storage->total_frames, s_vm_storage->total_frames * 0.04f);
    }
}

static uint32_t s_last_talk_click_ms = 0;
static uint8_t  s_talk_click_count = 0;

void handle_press(wt::Press p, int which) {
    // which: 0 = talk button (BOOT, external, headset), 1 = vol up, 2 = vol down
    if (p == wt::Press::None) return;
    ESP_LOGI(TAG, "Button %d event: %d", which, static_cast<int>(p));
    if (which == 0) {
        if (s_distress_active) {
            // Any click or press while distress is active immediately cancels it!
            s_talk_click_count = 0;
            toggle_distress();
            return;
        }
        if (p == wt::Press::Click) {
            uint32_t now = now_ms();
            if (s_vm_recording) {
                // In voice mail recording mode: click means STOP & SEND!
                s_talk_click_count = 0;
                send_voicemail();
                return;
            }
            if (s_talk_click_count > 0 && (now - s_last_talk_click_ms <= 450)) {
                s_talk_click_count++;
                s_last_talk_click_ms = now;
                if (s_talk_click_count >= 3) {
                    // Triple click: start Voice Mail record!
                    s_talk_click_count = 0;
                    ESP_LOGI(TAG, "Triple click: start Voice Mail recording");
                    start_voicemail_recording();
                }
            } else {
                s_talk_click_count = 1;
                s_last_talk_click_ms = now;
            }
        } else if (p == wt::Press::Long) {
            // Pre-warning soft ticks at 1.2s indicating button is being held for distress
            audio::tone(650, 20, 350);
            audio::tone(0, 40);
            audio::tone(750, 25, 350);
        } else if (p == wt::Press::Hold) {
            // 2.5s hold: trigger distress signal!
            s_talk_click_count = 0;
            toggle_distress();
        }
    }
    if (which == 1 && p == wt::Press::Click) volume_step(+10);
    if (which == 2 && p == wt::Press::Click) volume_step(-10);
}

// --- radio rx --------------------------------------------------------------------------

void on_rx(const radio::Rx& rx) {
    const uint32_t id = rx.hdr.sender;
    switch (rx.hdr.type) {
        case wt::MSG_AUDIO: {
            wt::AudioPayload a;
            if (wt::read_audio(rx.plain, rx.len, a)) audio::on_audio(id, a);
            break;
        }
        case wt::MSG_BEACON:
        case wt::MSG_JOIN:
        case wt::MSG_LEAVE: {
            if (rx.len < 1 + sizeof s_name) break;
            char name[sizeof s_name];
            memcpy(name, rx.plain + 1, sizeof name);
            name[sizeof name - 1] = 0;
            radio::peers_lock();
            wt::Peer* p = radio::peers().find(id);
            bool was = p && p->talking, now = rx.plain[0] != 0;
            radio::peers().set_name(id, name);
            radio::peers().set_talking(id, now);
            radio::peers_unlock();
            if (now != was && rx.hdr.type != wt::MSG_BEACON) {
                now ? chime_peer_joined() : chime_peer_left();
                ESP_LOGI(TAG, "%s %s", name, now ? "joined" : "left");
            }
            break;
        }
        case wt::MSG_PING:
            if (rx.len >= 4) radio::send(wt::MSG_PONG, rx.plain, 4);
            led::flash(led::Color::Cyan, 150);
            break;
        case wt::MSG_PONG: {
            uint32_t tok;
            if (rx.len < 4) break;
            memcpy(&tok, rx.plain, 4);
            xSemaphoreTake(s_app_mtx, portMAX_DELAY);
            bool mine = tok == s_ping_token;
            uint32_t sent_ms = s_ping_ms;
            xSemaphoreGive(s_app_mtx);
            if (!mine) break;
            char name[sizeof s_name] = "?";
            radio::peers_lock();
            if (const wt::Peer* p = radio::peers().find(id))
                if (p->name[0]) memcpy(name, p->name, sizeof name);
            radio::peers_unlock();
            // ponytail: printf from the radio task blocks it ~5 ms per line at 115200 baud;
            // fine for a ping reply. Move to a log queue if pings get frequent.
            printf("pong %-11s %3lu ms  %s  rssi %d\n", name, (unsigned long)(now_ms() - sent_ms),
                   rx.hops ? "via relay" : "direct", rx.rssi);
            break;
        }
        case wt::MSG_RANGE_REQ: {
            if (rx.len >= sizeof(wt::RangeReq)) {
                wt::RangeReq req;
                memcpy(&req, rx.plain, sizeof(req));
                wt::RangeResp resp;
                resp.token = req.token;
                resp.seq = req.seq;
                resp.req_rssi = rx.rssi;
                radio::send(wt::MSG_RANGE_RESP, reinterpret_cast<const uint8_t*>(&resp), sizeof(resp));
                led::flash(led::Color::Cyan, 40);
            }
            break;
        }
        case wt::MSG_RANGE_RESP: {
            if (rx.len >= sizeof(wt::RangeResp)) {
                wt::RangeResp resp;
                memcpy(&resp, rx.plain, sizeof(resp));
                if (resp.token == s_range_token) {
                    s_walk_pending = false;
                    s_range_stats.add_sample(resp.req_rssi, rx.rssi);
                    // Acoustic pitch proportional to link strength
                    if (rx.rssi >= -75) audio::tone(880, 40, 450);      // Strong
                    else if (rx.rssi >= -90) audio::tone(580, 40, 400); // Medium
                    else audio::tone(380, 50, 400);                     // Edge of range
                    printf("  probe #%-2u: OK | Downlink: %4d dBm | Uplink: %4d dBm\n",
                           resp.seq, rx.rssi, resp.req_rssi);
                }
            }
            break;
        }
        case wt::MSG_DISTRESS: {
            if (rx.len >= sizeof(wt::DistressPayload)) {
                wt::DistressPayload dp;
                memcpy(&dp, rx.plain, sizeof(dp));
                dp.name[sizeof(dp.name) - 1] = 0;
                if (dp.active) {
                    s_distress_peer = dp.sender;
                    strncpy(s_distress_peer_name, dp.name, sizeof(s_distress_peer_name) - 1);
                    s_distress_peer_name[sizeof(s_distress_peer_name) - 1] = 0;
                    printf("\n*** [DISTRESS ALERT] Emergency SOS from %s (%08lx)! RSSI: %d dBm ***\n\n",
                           dp.name, (unsigned long)dp.sender, rx.rssi);
                    sound_distress_alert();
                } else {
                    if (s_distress_peer == dp.sender) s_distress_peer = 0;
                    printf("\n[DISTRESS CLEARED] Emergency resolved by %s (%08lx).\n\n",
                           dp.name, (unsigned long)dp.sender);
                    sound_all_clear();
                }
            }
            break;
        }
        case wt::MSG_VOICEMAIL: {
            if (rx.len >= sizeof(wt::VoicemailPayload)) {
                wt::VoicemailPayload vp;
                memcpy(&vp, rx.plain, sizeof(vp));
                vp.name[sizeof(vp.name) - 1] = 0;
                if (vp.vm_id != s_last_played_vm_id) {
                    s_last_played_vm_id = vp.vm_id;
                    chime_voicemail_received();
                    led::flash(led::Color::Cyan, 150);
                    printf("\n*** [VOICE MAIL RECEIVED] from %-11s (%u frames, %.1fs) ***\n\n",
                           vp.name[0] ? vp.name : "?", vp.total_frames, vp.total_frames * 0.04f);
                }
                wt::AudioPayload a;
                a.voice_id = static_cast<uint8_t>(vp.vm_id & 0xFF);
                a.frame_no = vp.frame_index;
                a.adpcm_pred = vp.adpcm_pred;
                a.adpcm_index = static_cast<uint8_t>(vp.adpcm_index);
                a.data_len = vp.data_len > wt::kAdpcmBytes ? wt::kAdpcmBytes : vp.data_len;
                a.data = vp.data;
                audio::on_audio(id, a);
            }
            break;
        }
    }
}

void run_range_test(int count) {
    if (s_range_active) {
        printf("Range test already active.\n");
        return;
    }
    s_range_active = true;
    s_range_token = esp_random();
    s_range_stats.reset(count);

    printf("\n===========================================================\n");
    printf("  STARTING RANGE TEST (%d probes @ LR 250k Ch %u)\n", count, radio::channel());
    printf("===========================================================\n");

    for (int i = 0; i < count; i++) {
        wt::RangeReq req;
        req.token = s_range_token;
        req.seq = i + 1;
        req.total = count;
        uint16_t rx_before = s_range_stats.received;
        radio::send(wt::MSG_RANGE_REQ, reinterpret_cast<const uint8_t*>(&req), sizeof(req));
        vTaskDelay(pdMS_TO_TICKS(80));
        if (s_range_stats.received == rx_before) {
            audio::tone(250, 30, 350);
            printf("  probe #%-2u: [DROPPED / NO REPLY]\n", i + 1);
        }
    }

    printf("================== RANGE TEST SUMMARY ===================\n");
    printf("  Probes Sent:     %u\n", s_range_stats.sent);
    printf("  Probes Received: %u\n", s_range_stats.received);
    printf("  Packet Loss:     %.1f%%\n", s_range_stats.loss_percent());
    if (s_range_stats.received > 0) {
        printf("  Downlink RSSI:   Avg %.1f dBm (Min %d, Max %d)\n",
               s_range_stats.avg_downlink_rssi(), s_range_stats.min_rssi, s_range_stats.max_rssi);
        printf("  Uplink RSSI:     Avg %.1f dBm\n", s_range_stats.avg_uplink_rssi());
        printf("  Link Margin:     +%.1f dB above -108 dBm sensitivity floor\n",
               s_range_stats.margin_db(-108.0f));
        const char* quality = "EXCELLENT";
        if (s_range_stats.loss_percent() > 20.0f || s_range_stats.avg_downlink_rssi() < -95.0f) quality = "MARGINAL (Near limit)";
        else if (s_range_stats.loss_percent() > 5.0f || s_range_stats.avg_downlink_rssi() < -85.0f) quality = "GOOD";
        printf("  Link Quality:    %s\n", quality);
    } else {
        printf("  Result:          NO LINK (Nodes out of range or not on same channel)\n");
    }
    printf("=========================================================\n\n");
    s_range_active = false;
}

void toggle_walk_mode() {
    s_walk_mode = !s_walk_mode;
    if (s_walk_mode) {
        s_range_token = esp_random();
        s_walk_seq = 0;
        printf("\nWALK RANGE MODE: ON (emitting 1 probe/sec with acoustic pitch)\n");
        printf("Put on headset and walk away. Tone pitch indicates signal strength!\n\n");
        audio::tone(880, 60, 450);
    } else {
        printf("\nWALK RANGE MODE: OFF\n\n");
        audio::tone(400, 60, 400);
    }
}

// --- console ---------------------------------------------------------------------------

void print_status() {
    radio::Stats r = radio::stats();
    audio::Stats a = audio::stats();
    char name[sizeof s_name];
    xSemaphoreTake(s_app_mtx, portMAX_DELAY);
    memcpy(name, s_name, sizeof name);
    xSemaphoreGive(s_app_mtx);
    printf("\n%s  node %08lx  ch %u  vol %u%%  mic %s%s\n", name, (unsigned long)radio::self_id(),
           radio::channel(), audio::volume(), audio::talking() ? "OPEN" : "muted",
           audio::loopback() ? "  [loopback]" : "");
    usb_headset::print_info();
    printf("  peers:\n");
    struct Row { uint32_t id, age; char name[12]; int8_t rssi; bool relay, talking; };
    Row rows[wt::kMaxPeers];
    int nrows = 0;
    radio::peers_lock();  // copy only: printing here would stall the radio task
    uint32_t now = now_ms();
    for (const wt::Peer* p = radio::peers().begin(); p != radio::peers().end(); ++p) {
        if (!p->used) continue;
        Row& r = rows[nrows++];
        r.id = p->id;
        r.age = (now - p->last_ms) / 1000;
        memcpy(r.name, p->name, sizeof r.name);
        r.rssi = p->rssi;
        r.relay = p->hops != 0;
        r.talking = p->talking;
    }
    radio::peers_unlock();
    for (int i = 0; i < nrows; i++)
        printf("    %-11s %08lx  %s  rssi %4d  seen %lus ago%s\n", rows[i].name[0] ? rows[i].name : "?",
               (unsigned long)rows[i].id, rows[i].relay ? "relay " : "direct", rows[i].rssi,
               (unsigned long)rows[i].age, rows[i].talking ? "  talking" : "");
    int8_t pwr = 0;
    esp_wifi_get_max_tx_power(&pwr);
    printf("  radio: mode %s  tx-power %d (%.2f dBm)\n",
           cfg::kLongRange ? "ESP-NOW Long Range (LR 250k, -108 dBm)" : "802.11g 6M (-90 dBm)",
           pwr, pwr * 0.25f);
    printf("  radio: tx %lu rx %lu relayed %lu dup %lu rx-drop %lu auth-fail %lu tx-fail %lu\n",
           (unsigned long)r.tx, (unsigned long)r.rx, (unsigned long)r.relayed, (unsigned long)r.dup,
           (unsigned long)r.rx_drop, (unsigned long)r.auth_fail, (unsigned long)r.tx_fail);
    float temp_c = 0.0f;
    if (s_temp_sensor) temperature_sensor_get_celsius(s_temp_sensor, &temp_c);
    uint32_t free_ram = esp_get_free_heap_size() / 1024;
    uint32_t min_ram = esp_get_minimum_free_heap_size() / 1024;
    printf("  system: Temp %.1f C | RAM Free %lu KB (min %lu KB)%s%s\n",
           temp_c, (unsigned long)free_ram, (unsigned long)min_ram,
           s_distress_active ? "  [DISTRESS ACTIVE]" : "",
           s_distress_peer != 0 ? "  [REMOTE SOS HEARD]" : "");
    printf("  audio: sent %lu gated %lu played %lu lost %lu late %lu underrun %lu agc %.1fx\n\n",
           (unsigned long)a.frames_sent, (unsigned long)a.frames_gated, (unsigned long)a.frames_played,
           (unsigned long)a.lost, (unsigned long)a.late, (unsigned long)a.spk_underrun, a.agc_gain);
}

void selftest() {
    bool ok = true;
    auto check = [&](bool c, const char* what) {
        printf("  %s  %s\n", c ? "PASS" : "FAIL", what);
        ok &= c;
    };
    printf("selftest\n");
    wt::StreamFormat m = usb_headset::mic_format(), s = usb_headset::spk_format();
    check(usb_headset::mic_ready(), "headset microphone streaming");
    check(usb_headset::spk_ready(), "headset speaker streaming");
    uint32_t b0 = audio::stats().mic_bytes;
    vTaskDelay(pdMS_TO_TICKS(500));
    uint32_t got = audio::stats().mic_bytes - b0;
    uint32_t expect = m.valid() ? m.rate * m.frame_bytes() / 2 : 1;
    check(m.valid() && got > expect * 8 / 10 && got < expect * 12 / 10,
          "mic delivers samples at its nominal rate (+/-20%)");
    vTaskDelay(pdMS_TO_TICKS(1100));  // one full peak window
    int16_t pk = audio::stats().mic_peak;
    char pl[80];
    snprintf(pl, sizeof pl, "mic is live, not digital silence (peak %d of 32767)", pk);
    check(pk > 8, pl);
    if (m.valid() && s.valid()) {
        float snr = wt::pipeline_selftest(m.rate, s.rate);
        char line[80];
        snprintf(line, sizeof line, "1 kHz tone through the voice path at %lu->%lu Hz: %.1f dB SNR",
                 (unsigned long)m.rate, (unsigned long)s.rate, snr);
        check(snr > 18.f, line);  // software path only: codec, jitter buffer, resamplers
    }
    check(radio::stats().tx_fail == 0 || radio::stats().tx > radio::stats().tx_fail * 20,
          "radio send failures under 5%");
    printf("%s. Then: 'loop' and speak, you should hear yourself about 0.2 s later.\n",
           ok ? "ALL PASS" : "FAILED");
}

void help() {
    printf("\n  t          talk on/off (same as BOOT or headset button click)\n"
           "  r / c      radar: check peer count and distance (same as double click)\n"
           "  vm         record voice mail / send (same as triple click, then click to send)\n"
           "  send       send recorded voice mail (or flush outbox)\n"
           "  sos        distress SOS on/off (same as holding button 2.5s)\n"
           "  range [N]  run range test with N probes (default 20, measures loss %% & RSSI)\n"
           "  walk       toggle walk range mode (1 probe/sec with acoustic tone)\n"
           "  + / -      volume\n"
           "  p          ping all nodes (same as long press)\n"
           "  s          status, peers, counters\n"
           "  ch N       radio channel 1..13 (all nodes must match)\n"
           "  name X     set this node's name\n"
           "  loop       loopback test: hear your own voice through the codec\n"
           "  selftest   check headset streams and the audio path\n"
           "  reboot\n\n");
}

void console_task(void*) {
    char line[48];
    size_t n = 0;
    for (;;) {
        int c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (c == '\r') continue;  // stdin maps CR to LF already; kept for other consoles
        if (c != '\n') {
            if (n < sizeof line - 1) line[n++] = static_cast<char>(c);
            continue;
        }
        line[n] = 0;
        n = 0;
        if (!strcmp(line, "t")) toggle_talk();
        else if (!strcmp(line, "r") || !strcmp(line, "c")) radar_check();
        else if (!strcmp(line, "vm") || !strcmp(line, "record")) {
            if (s_vm_recording) send_voicemail();
            else start_voicemail_recording();
        }
        else if (!strcmp(line, "send")) send_voicemail();
        else if (!strcmp(line, "sos") || !strcmp(line, "distress")) toggle_distress();
        else if (!strncmp(line, "range", 5) || !strcmp(line, "test")) {
            int count = 20;
            if (strlen(line) > 6) count = atoi(line + 6);
            if (count <= 0 || count > 200) count = 20;
            run_range_test(count);
        }
        else if (!strcmp(line, "walk")) toggle_walk_mode();
        else if (!strcmp(line, "+")) volume_step(+10);
        else if (!strcmp(line, "-")) volume_step(-10);
        else if (!strcmp(line, "p")) send_ping();
        else if (!strcmp(line, "s")) print_status();
        else if (!strcmp(line, "loop")) {
            audio::set_loopback(!audio::loopback());
            printf("loopback %s\n", audio::loopback() ? "ON (speak now)" : "off");
        } else if (!strcmp(line, "selftest")) selftest();
        else if (!strcmp(line, "reboot")) esp_restart();
        else if (!strncmp(line, "ch ", 3)) {
            int ch = atoi(line + 3);
            if (radio::set_channel(static_cast<uint8_t>(ch))) {
                nvs_put_u8("ch", static_cast<uint8_t>(ch));
                printf("channel %d\n", ch);
            } else printf("channel must be 1..13\n");
        } else if (!strncmp(line, "name ", 5) && line[5]) {
            xSemaphoreTake(s_app_mtx, portMAX_DELAY);
            strncpy(s_name, line + 5, sizeof s_name - 1);
            s_name[sizeof s_name - 1] = 0;
            nvs_set_str(s_nvs, "name", s_name);
            nvs_commit(s_nvs);
            xSemaphoreGive(s_app_mtx);
            broadcast_state();
            printf("name %.11s\n", line + 5);
        } else if (line[0]) help();
    }
}

// --- buttons ---------------------------------------------------------------------------

void on_hid_buttons(uint8_t down) {
    if (down != s_hid_down) ESP_LOGI(TAG, "HID raw: 0x%02x", down);
    s_hid_down = down;
}

void on_headset_state(bool connected) {
    if (connected) {
        chime_join();
        ESP_LOGI(TAG, "headset ready");
    }
}

void init_gpio(gpio_num_t pin) {
    gpio_config_t c = {};
    c.pin_bit_mask = 1ULL << pin;
    c.mode = GPIO_MODE_INPUT;
    c.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&c);
}

bool low(gpio_num_t pin) { return gpio_get_level(pin) == 0; }

}  // namespace

extern "C" void app_main() {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_open("wt", NVS_READWRITE, &s_nvs));
    s_app_mtx = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    size_t len = sizeof s_name;
    if (nvs_get_str(s_nvs, "name", s_name, &len) != ESP_OK)
        snprintf(s_name, sizeof s_name, "node-%04x", static_cast<unsigned>(esp_random() & 0xFFFF));
    uint8_t ch = nvs_u8("ch", cfg::kDefaultChannel);
    audio::set_volume(nvs_u8("vol", cfg::kDefaultVolume));

    led::start();
    led::set(led::Color::Blue);
    for (gpio_num_t p : {cfg::kPinBoot, cfg::kPinTalk, cfg::kPinVolUp, cfg::kPinVolDown}) init_gpio(p);

    uint32_t session = next_session();
    if (!session) {
        led::set(led::Color::Red);
        ESP_LOGE(TAG, "cannot store the session counter in NVS, radio stays off (erase flash and reflash)");
        vTaskDelay(portMAX_DELAY);
    }
    if (!radio::start(ch, session, on_rx)) {
        led::set(led::Color::Red);
        ESP_LOGE(TAG, "radio failed, halting");
        vTaskDelay(portMAX_DELAY);
    }
    audio::start();
    temperature_sensor_config_t temp_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&temp_cfg, &s_temp_sensor) == ESP_OK) {
        temperature_sensor_enable(s_temp_sensor);
    }
    usb_headset::start(on_hid_buttons, on_headset_state);
    xTaskCreatePinnedToCore(console_task, "console", 8192, nullptr, 3, nullptr, 0);
    printf("\n%s ready on channel %u. Type 'h' for commands.\n", s_name, ch);

    wt::ButtonDetector talk, up, down;
    uint8_t last_toggle = 0;
    uint8_t last_hid = 0;
    uint32_t hid_talk_start = 0;
    bool hid_talk_long_sent = false;
    bool hid_talk_hold_sent = false;
    uint32_t last_beacon = 0, talk_since = 0;
    for (;;) {
        uint32_t now = now_ms();
        uint8_t hid = s_hid_down;

        // Latching hook/mute control: every change is one click
        uint8_t tog = hid & wt::HID_BTN_TALK_TOGGLE;
        if (tog != last_toggle) {
            last_toggle = tog;
            handle_press(wt::Press::Click, 0);
        }

        // USB HID buttons: packets are digital and bounce-free; act directly on edge
        if ((hid & wt::HID_BTN_VOL_UP) && !(last_hid & wt::HID_BTN_VOL_UP)) {
            handle_press(wt::Press::Click, 1);
        }
        if ((hid & wt::HID_BTN_VOL_DOWN) && !(last_hid & wt::HID_BTN_VOL_DOWN)) {
            handle_press(wt::Press::Click, 2);
        }

        // Talk / Media button on headset (Play/Pause, Voice Command, Mute):
        // Click on release (< 1.2s), Long press when held (>= 1.2s), Hold when held (>= 2.5s)
        if ((hid & wt::HID_BTN_TALK) && !(last_hid & wt::HID_BTN_TALK)) {
            hid_talk_start = now;
            hid_talk_long_sent = false;
            hid_talk_hold_sent = false;
        }
        if ((hid & wt::HID_BTN_TALK) && !hid_talk_long_sent && (now - hid_talk_start >= wt::ButtonDetector::kLongMs)) {
            hid_talk_long_sent = true;
            handle_press(wt::Press::Long, 0);
        }
        if ((hid & wt::HID_BTN_TALK) && !hid_talk_hold_sent && (now - hid_talk_start >= wt::ButtonDetector::kHoldMs)) {
            hid_talk_hold_sent = true;
            handle_press(wt::Press::Hold, 0);
        }
        if (!(hid & wt::HID_BTN_TALK) && (last_hid & wt::HID_BTN_TALK)) {
            if (!hid_talk_long_sent && !hid_talk_hold_sent) {
                handle_press(wt::Press::Click, 0);
            }
        }
        last_hid = hid;

        // Physical GPIO buttons (BOOT, external pins) use debouncer
        handle_press(talk.update(low(cfg::kPinBoot) || low(cfg::kPinTalk), now), 0);
        handle_press(up.update(low(cfg::kPinVolUp), now), 1);
        handle_press(down.update(low(cfg::kPinVolDown), now), 2);

        // Handle pending click evaluation (450 ms after last click)
        if (s_talk_click_count > 0 && (now - s_last_talk_click_ms > 450)) {
            uint8_t count = s_talk_click_count;
            s_talk_click_count = 0;
            if (count == 1) {
                toggle_talk();
            } else if (count == 2) {
                ESP_LOGI(TAG, "Double click: radar check");
                radar_check();
            }
        }

        // Voice mail outbox: automatically deliver stored voice mail when a peer reconnects
        if (s_vm_outbox_pending && s_vm_storage && s_vm_storage->total_frames > 0) {
            uint32_t now_check = now;
            bool has_peer = false;
            radio::peers_lock();
            for (const wt::Peer* p = radio::peers().begin(); p != radio::peers().end(); ++p) {
                if (p->used && (now_check - p->last_ms <= 15000)) { has_peer = true; break; }
            }
            radio::peers_unlock();
            if (has_peer) {
                printf("\n[Voice Mail] Peer detected! Automatically sending outbox voice mail...\n");
                flush_voicemail_to_mesh();
                audio::tone(750, 40, 380);
                audio::tone(0, 20);
                audio::tone(850, 50, 380);
            }
        }

        // Auto-mute a mic that has been left open too long (disabled when kMaxTalkMs == 0).
        if (cfg::kMaxTalkMs > 0 && audio::talking()) {
            if (!talk_since) talk_since = now;
            if (now - talk_since > cfg::kMaxTalkMs) {
                ESP_LOGW(TAG, "mic open for %lus, muting", (unsigned long)(cfg::kMaxTalkMs / 1000));
                toggle_talk();
            }
        } else {
            talk_since = 0;
        }

        if (now - last_beacon >= cfg::kBeaconMs) {
            last_beacon = now;
            uint8_t p[1 + sizeof s_name];
            xSemaphoreTake(s_app_mtx, portMAX_DELAY);
            state_payload(p);
            xSemaphoreGive(s_app_mtx);
            radio::send(wt::MSG_BEACON, p, sizeof p);
            radio::peers_lock();
            radio::peers().prune(now);
            radio::peers_unlock();
        }

        static uint32_t last_walk_probe = 0;
        if (s_walk_mode && (now - last_walk_probe >= 1000)) {
            last_walk_probe = now;
            wt::RangeReq req;
            req.token = s_range_token;
            req.seq = ++s_walk_seq;
            req.total = 0;
            s_walk_pending = true;
            s_walk_probe_sent_ms = now;
            radio::send(wt::MSG_RANGE_REQ, reinterpret_cast<const uint8_t*>(&req), sizeof(req));
        }
        if (s_walk_mode && s_walk_pending && (now - s_walk_probe_sent_ms > 400)) {
            s_walk_pending = false;
            audio::tone(250, 40, 350);  // low warning tone: missed probe
        }

        // Distress SOS periodic broadcasting & audible reminder
        if (s_distress_active) {
            if (now - s_last_distress_tx_ms >= 2000) {
                s_last_distress_tx_ms = now;
                send_distress_packet(true);
            }
            if (now - s_last_distress_beep_ms >= 2500) {
                s_last_distress_beep_ms = now;
                sound_distress_alert();
            }
        } else if (s_distress_peer != 0) {
            if (now - s_last_distress_beep_ms >= 3000) {
                s_last_distress_beep_ms = now;
                sound_distress_alert();
            }
        }

        update_led();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
