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
#include "esp_sleep.h"
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
static uint32_t s_last_distress_rx_ms = 0;
static volatile bool s_vm_recording = false;
static temperature_sensor_handle_t s_temp_sensor = nullptr;

// --- Audio Voice Menu & PTT state ---
enum MenuItem : uint8_t {
    MENU_WALK = 0,
    MENU_TALK_MODE,
    MENU_CHANNEL,
    MENU_RADAR,
    MENU_VOICE_NOTE,
    MENU_POWER_OFF,
    MENU_EXIT,
    MENU_COUNT
};

static bool s_ptt_mode = false;          // false = Hands-Free VOX, true = Classic PTT
static bool s_ptt_active = false;        // currently actively transmitting via PTT hold
static bool s_menu_active = false;
static MenuItem s_menu_item = MENU_WALK;
static uint32_t s_last_menu_activity_ms = 0;

void broadcast_state();
void enter_menu();
void exit_menu(bool play_tone = true);
void menu_next();
void menu_prev();
void menu_action();
void announce_main_menu_item();
void system_power_off();

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

void chime_peer_joined() {
    audio::tone(587, 25, 250);
    audio::tone(0, 15);
    audio::tone(880, 35, 250);
}

void chime_peer_left() {
    audio::tone(880, 25, 250);
    audio::tone(0, 15);
    audio::tone(587, 35, 250);
}

void sound_distress_alert() {
    audio::prompt_clear();
    audio::prompt(wt::PROMPT_EMERGENCY_RX);
}

void sound_all_clear() {
    audio::prompt_clear();
    audio::prompt(wt::PROMPT_EMERGENCY_OFF);
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

    if (s_menu_active) {
        led::set(Color::White);
        return;
    }

    // Voice Note recording: gentle soft Amber pulse to clearly show recording in progress
    if (s_vm_recording) {
        static uint32_t s_last_vm_flash = 0;
        static bool s_amber_on = false;
        uint32_t now = now_ms();
        if (now - s_last_vm_flash >= 300) {
            s_last_vm_flash = now;
            s_amber_on = !s_amber_on;
        }
        led::set(s_amber_on ? Color::Amber : Color::Off);
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
    audio::prompt_clear();
    if (s_distress_active) {
        printf("\n*** [DISTRESS ACTIVATED] Emergency SOS broadcasting! Click button to cancel. ***\n\n");
        audio::prompt(wt::PROMPT_EMERGENCY_ON);
        // Automatically enable microphone when emergency is activated!
        xSemaphoreTake(s_app_mtx, portMAX_DELAY);
        audio::set_talking(true);
        xSemaphoreGive(s_app_mtx);
        broadcast_state();
    } else {
        printf("\n*** [DISTRESS CANCELLED] All-clear broadcasted. ***\n\n");
        audio::prompt(wt::PROMPT_EMERGENCY_OFF);
        // Restore saved mic state from NVS
        uint8_t mic_saved = nvs_u8("mic", 0);
        xSemaphoreTake(s_app_mtx, portMAX_DELAY);
        audio::set_talking(!s_ptt_mode && (mic_saved != 0));
        xSemaphoreGive(s_app_mtx);
        broadcast_state();
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
    nvs_put_u8("mic", on ? 1 : 0);
    audio::prompt_clear();
    audio::prompt(on ? wt::PROMPT_MIC_ON : wt::PROMPT_MIC_OFF);
    broadcast_state();
    ESP_LOGI(TAG, "mic %s (saved to NVS)", on ? "OPEN" : "muted");
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
static bool s_vm_outbox_pending = false;
static VmStorage* s_vm_inbox = nullptr;
static bool s_vm_inbox_has_message = false;
static uint32_t s_last_received_vm_id = 0;
static TaskHandle_t s_vm_playback_task = nullptr;
static constexpr uint32_t kVmLocalSender = 0xFFFFFFFE;

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
    if (count == 0) printf("  No peers in range.\n");
    if (s_vm_outbox_pending && s_vm_storage && s_vm_storage->total_frames > 0) {
        printf("  [OUTBOX QUEUED] 1 Voice Note (%.1fs) pending delivery once in range.\n",
               s_vm_storage->total_frames * 0.04f);
    }
    printf("\n");

    // Voice prompt acoustic telemetry in headset:
    // Voice prompt acoustic telemetry in headset (single unified natural phrase):
    audio::prompt_clear();
    if (count == 0) {
        audio::prompt(wt::PROMPT_RADAR_NONE);
    } else if (count == 1) {
        if (has_direct && best_rssi >= -70) audio::prompt(wt::PROMPT_RADAR_1_NEAR);
        else if (has_direct && best_rssi >= -88) audio::prompt(wt::PROMPT_RADAR_1_MID);
        else audio::prompt(wt::PROMPT_RADAR_1_FAR);
    } else if (count == 2) {
        if (has_direct && best_rssi >= -70) audio::prompt(wt::PROMPT_RADAR_2_NEAR);
        else if (has_direct && best_rssi >= -88) audio::prompt(wt::PROMPT_RADAR_2_MID);
        else audio::prompt(wt::PROMPT_RADAR_2_FAR);
    } else {
        if (has_direct && best_rssi >= -70) audio::prompt(wt::PROMPT_RADAR_MANY_NEAR);
        else if (has_direct && best_rssi >= -88) audio::prompt(wt::PROMPT_RADAR_MANY_MID);
        else audio::prompt(wt::PROMPT_RADAR_MANY_FAR);
    }
}



// Simple, intuitive Low-Medium-High harmonic notes:
void chime_voicemail_prompt() {
    audio::prompt_clear();
    audio::prompt(wt::PROMPT_VM_RECORDING);
}

void chime_voicemail_send() {
    audio::prompt_clear();
    audio::prompt(wt::PROMPT_VM_SENT);
}

void chime_voicemail_received() {
    audio::prompt_clear();
    audio::prompt(wt::PROMPT_VM_INCOMING);
}

void on_voicemail_captured_frame(const wt::AudioPayload& a) {
    if (!s_vm_recording || !s_vm_storage) return;
    if (s_vm_storage->total_frames >= 250) {
        // 10 seconds maximum reached: stop recording automatically
        s_vm_recording = false;
        audio::set_recording_vm(false);
        audio::tone(700, 70, 400);
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

    uint32_t now = now_ms();
    int active_peers = 0;
    radio::peers_lock();
    for (const wt::Peer* p = radio::peers().begin(); p != radio::peers().end(); ++p) {
        if (p->used && (now - p->last_ms <= 15000)) active_peers++;
    }
    radio::peers_unlock();

    if (active_peers > 0) {
        chime_voicemail_send();
        flush_voicemail_to_mesh();
        led::flash(led::Color::Cyan, 150);
    } else {
        s_vm_outbox_pending = true;
        audio::prompt_clear();
        audio::prompt(wt::PROMPT_VM_OUTBOX);
        printf("[Voice Mail] Out of range (0 peers). Saved to outbox (%u frames, %.1fs). Will auto-send when a node is detected!\n",
               s_vm_storage->total_frames, s_vm_storage->total_frames * 0.04f);
    }
}

void vm_playback_worker(void* arg) {
    if (!s_vm_inbox || s_vm_inbox->total_frames == 0) {
        s_vm_playback_task = nullptr;
        vTaskDelete(nullptr);
        return;
    }
    uint8_t voice_id = static_cast<uint8_t>(s_vm_inbox->vm_id & 0xFF);
    uint16_t total = s_vm_inbox->total_frames;
    printf("[Voice Note] Playing %u frames (%.1fs)...\n", total, total * 0.04f);
    led::flash(led::Color::Cyan, 120);

    for (uint16_t i = 0; i < total; i++) {
        if (!s_vm_inbox_has_message) break;
        const VmFrame& f = s_vm_inbox->frames[i];
        wt::AudioPayload a;
        a.voice_id = voice_id;
        a.frame_no = i;
        a.adpcm_pred = f.adpcm_pred;
        a.adpcm_index = static_cast<uint8_t>(f.adpcm_index);
        a.data_len = f.data_len;
        a.data = f.data;
        audio::on_audio(kVmLocalSender, a);
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    printf("[Voice Note] Playback finished. Click 1x to repeat, 2x to remove/dismiss.\n");
    s_vm_playback_task = nullptr;
    vTaskDelete(nullptr);
}

void play_inbox_voicemail() {
    if (!s_vm_inbox || s_vm_inbox->total_frames == 0) return;
    if (s_vm_playback_task) {
        vTaskDelete(s_vm_playback_task);
        s_vm_playback_task = nullptr;
    }
    audio::clear_talker(kVmLocalSender);
    xTaskCreate(vm_playback_worker, "vm_play", 3072, nullptr, 4, &s_vm_playback_task);
}

void dismiss_inbox_voicemail() {
    if (s_vm_playback_task) {
        vTaskDelete(s_vm_playback_task);
        s_vm_playback_task = nullptr;
    }
    audio::clear_talker(kVmLocalSender);
    s_vm_inbox_has_message = false;
    if (s_vm_inbox) {
        free(s_vm_inbox);
        s_vm_inbox = nullptr;
    }
    // Subtle acknowledgment tone (descending two-tone) for dismiss
    audio::tone(500, 35, 300);
    audio::tone(0, 15);
    audio::tone(350, 45, 300);
    printf("[Voice Note] Dismissed / Removed.\n");
}

static uint32_t s_last_talk_click_ms = 0;
static uint8_t  s_talk_click_count = 0;
static constexpr uint32_t kMultiClickWindowMs = 500;

void handle_press(wt::Press p, int which) {
    // which: 0 = Headset Talk, 1 = Vol Up, 2 = Vol Down, 3 = ESP32 BOOT button
    if (p == wt::Press::None) return;
    ESP_LOGI(TAG, "Button %d event: %d", which, static_cast<int>(p));

    // --- ESP32 BOOT button (GPIO 0) -> Dedicated to Settings & Audio Menu ---
    if (which == 3) {
        if (s_distress_active) {
            // Any press on BOOT cancels distress immediately!
            toggle_distress();
            return;
        }

        if (!s_menu_active) {
            if (p == wt::Press::Click) {
                enter_menu();
            }
            return;
        }

        // Inside Menu (s_menu_active is true):
        s_last_menu_activity_ms = now_ms();

        if (p == wt::Press::Hold) {
            // Hold >= 2.5s: Exit menu immediately from anywhere!
            exit_menu(true);
            return;
        }

        if (p == wt::Press::Click) {
            // Click: Next menu item!
            menu_next();
        } else if (p == wt::Press::Long) {
            // Long press (~1.2s): Action / Toggle in-place!
            menu_action();
        }
        return;
    }

    // --- Headset Talk button (HID & kPinTalk) -> Walkie-Talkie & Emergency SOS ---
    if (which == 0) {
        if (s_distress_active) {
            // Any click or press while distress is active immediately cancels it!
            s_talk_click_count = 0;
            toggle_distress();
            return;
        }

        if (s_vm_recording) {
            // In voice mail recording mode: click means STOP & SEND!
            if (p == wt::Press::Click) {
                s_talk_click_count = 0;
                send_voicemail();
            }
            return;
        }

        if (p == wt::Press::Click) {
            uint32_t now = now_ms();
            if (s_vm_inbox_has_message) {
                if (s_talk_click_count > 0 && (now - s_last_talk_click_ms <= kMultiClickWindowMs)) {
                    // Double click: dismiss / remove incoming voicemail
                    s_talk_click_count = 0;
                    dismiss_inbox_voicemail();
                } else {
                    // Single click: will trigger play_inbox_voicemail() when window expires
                    s_talk_click_count = 1;
                    s_last_talk_click_ms = now;
                }
                return;
            }

            // Normal standby operation:
            if (!s_ptt_mode) {
                // Mode Call (Hands-Free VOX): 1x click toggles mic (mute / unmute)!
                toggle_talk();
            } else {
                // Mode PTT: Quick tap gives a subtle chirp acknowledgment
                audio::tone(700, 20, 300);
            }
        } else if (p == wt::Press::Long) {
            // Pre-warning soft ticks at 1.2s indicating button is being held for distress SOS
            audio::tone(650, 20, 350);
            audio::tone(0, 40);
            audio::tone(750, 25, 350);
        } else if (p == wt::Press::Hold) {
            // 2.5s hold: trigger distress SOS!
            s_talk_click_count = 0;
            toggle_distress();
        }
        return;
    }

    // --- Vol Up / Vol Down ---
    if (which == 1) { // Vol Up
        if (s_menu_active) {
            menu_prev();
        } else if (p == wt::Press::Click) {
            volume_step(+10);
        }
    }
    if (which == 2) { // Vol Down
        if (s_menu_active) {
            menu_next();
        } else if (p == wt::Press::Click) {
            volume_step(-10);
        }
    }
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
                    bool is_new = (s_distress_peer != dp.sender);
                    s_distress_peer = dp.sender;
                    s_last_distress_rx_ms = now_ms();
                    strncpy(s_distress_peer_name, dp.name, sizeof(s_distress_peer_name) - 1);
                    s_distress_peer_name[sizeof(s_distress_peer_name) - 1] = 0;
                    if (is_new) {
                        printf("\n*** [DISTRESS ALERT] Emergency SOS from %s (%08lx)! RSSI: %d dBm ***\n\n",
                               dp.name, (unsigned long)dp.sender, rx.rssi);
                        sound_distress_alert();
                        s_last_distress_beep_ms = now_ms();
                    }
                } else {
                    if (s_distress_peer == dp.sender) {
                        s_distress_peer = 0;
                        printf("\n[DISTRESS CLEARED] Emergency resolved by %s (%08lx).\n\n",
                               dp.name, (unsigned long)dp.sender);
                        sound_all_clear();
                    }
                }
            }
            break;
        }
        case wt::MSG_VOICEMAIL: {
            if (rx.len >= sizeof(wt::VoicemailPayload)) {
                wt::VoicemailPayload vp;
                memcpy(&vp, rx.plain, sizeof(vp));
                vp.name[sizeof(vp.name) - 1] = 0;
                if (vp.vm_id != s_last_received_vm_id) {
                    s_last_received_vm_id = vp.vm_id;
                    if (s_vm_playback_task) {
                        vTaskDelete(s_vm_playback_task);
                        s_vm_playback_task = nullptr;
                    }
                    audio::clear_talker(kVmLocalSender);
                    if (!s_vm_inbox) {
                        s_vm_inbox = static_cast<VmStorage*>(malloc(sizeof(VmStorage)));
                    }
                    if (s_vm_inbox) {
                        s_vm_inbox->vm_id = vp.vm_id;
                        s_vm_inbox->total_frames = 0;
                        memcpy(s_vm_inbox->name, vp.name, sizeof(s_vm_inbox->name));
                    }
                    s_vm_inbox_has_message = true;
                    chime_voicemail_received();
                    led::flash(led::Color::Cyan, 200);
                    printf("\n*** [VOICE NOTE RECEIVED] from %-11s (%u frames, %.1fs) ***\n",
                           vp.name[0] ? vp.name : "?", vp.total_frames, vp.total_frames * 0.04f);
                    printf(">>> Click 1x to hear, click 1x more to repeat, 2x to remove/dismiss <<<\n\n");
                }
                if (s_vm_inbox && vp.frame_index < 250) {
                    VmFrame& f = s_vm_inbox->frames[vp.frame_index];
                    f.adpcm_pred = vp.adpcm_pred;
                    f.adpcm_index = vp.adpcm_index;
                    f.data_len = vp.data_len > wt::kAdpcmBytes ? wt::kAdpcmBytes : vp.data_len;
                    memcpy(f.data, vp.data, f.data_len);
                    if (vp.frame_index + 1 > s_vm_inbox->total_frames) {
                        s_vm_inbox->total_frames = vp.frame_index + 1;
                    }
                }
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
    nvs_put_u8("walk", s_walk_mode ? 1 : 0);
    if (s_walk_mode) {
        s_range_token = esp_random();
        s_walk_seq = 0;
        printf("\nWALK RANGE MODE: ON (emitting 1 probe/sec with acoustic pitch)\n");
        printf("Put on headset and walk away. Tone pitch indicates signal strength!\n\n");
    } else {
        printf("\nWALK RANGE MODE: OFF\n\n");
    }
}

// --- Audio Voice Menu Engine -----------------------------------------------------------

void system_power_off() {
    printf("\n*** [SYSTEM] Powering off into Deep Sleep... ***\n\n");
    nvs_put_u8("pwr_state", 0);
    audio::prompt_clear();
    audio::prompt(wt::PROMPT_POWER_OFF);
    vTaskDelay(pdMS_TO_TICKS(650));
    led::set(led::Color::Off);

    // Wakeup on BOOT button (GPIO 0) and optional external Talk button (GPIO 4)
    esp_sleep_enable_ext0_wakeup(cfg::kPinBoot, 0);
    uint64_t mask = (1ULL << cfg::kPinBoot);
    if (cfg::kPinTalk != GPIO_NUM_NC) {
        mask |= (1ULL << cfg::kPinTalk);
    }
    esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_LOW);
    esp_deep_sleep_start();
}

void exit_menu(bool play_tone) {
    if (!s_menu_active) return;
    s_menu_active = false;
    if (play_tone) {
        audio::prompt_clear();
        audio::tone(600, 25, 300);
        audio::tone(0, 15);
        audio::tone(450, 35, 300);
    }
    printf("[Menu] Exited.\n");
}

void announce_main_menu_item() {
    audio::prompt_clear();
    switch (s_menu_item) {
        case MENU_WALK:
            audio::prompt(s_walk_mode ? wt::PROMPT_WALK_ON : wt::PROMPT_WALK_OFF);
            break;
        case MENU_TALK_MODE:
            audio::prompt(s_ptt_mode ? wt::PROMPT_PTT_ON : wt::PROMPT_HANDS_FREE_ON);
            break;
        case MENU_CHANNEL:
            audio::prompt(wt::PROMPT_MENU_CHANNEL);
            if (radio::channel() >= 1 && radio::channel() <= 13) {
                audio::prompt(static_cast<wt::PromptId>(wt::PROMPT_NUM_1 + (radio::channel() - 1)));
            }
            break;
        case MENU_RADAR:
            audio::prompt(wt::PROMPT_MENU_RADAR);
            break;
        case MENU_VOICE_NOTE:
            audio::prompt(wt::PROMPT_MENU_VOICE_NOTE);
            break;
        case MENU_POWER_OFF:
            audio::prompt(wt::PROMPT_POWER_OFF);
            break;
        case MENU_EXIT:
            audio::prompt(wt::PROMPT_MENU_EXIT);
            break;
        default:
            break;
    }
}

void enter_menu() {
    s_menu_active = true;
    s_menu_item = MENU_WALK;
    s_last_menu_activity_ms = now_ms();
    audio::prompt_clear();
    audio::prompt(wt::PROMPT_MENU);
    audio::prompt(s_walk_mode ? wt::PROMPT_WALK_ON : wt::PROMPT_WALK_OFF);
    printf("[Menu] Entered Audio Menu. Click: next, Hold: action/toggle, Hold 2.5s: exit.\n");
}

void menu_next() {
    s_last_menu_activity_ms = now_ms();
    s_menu_item = static_cast<MenuItem>((s_menu_item + 1) % MENU_COUNT);
    printf("[Menu] Next item: %u\n", s_menu_item);
    announce_main_menu_item();
}

void menu_prev() {
    s_last_menu_activity_ms = now_ms();
    s_menu_item = static_cast<MenuItem>((s_menu_item + MENU_COUNT - 1) % MENU_COUNT);
    printf("[Menu] Prev item: %u\n", s_menu_item);
    announce_main_menu_item();
}

void menu_action() {
    s_last_menu_activity_ms = now_ms();
    switch (s_menu_item) {
        case MENU_WALK:
            toggle_walk_mode();
            audio::prompt_clear();
            audio::prompt(s_walk_mode ? wt::PROMPT_WALK_ON : wt::PROMPT_WALK_OFF);
            printf("[Menu] Walk mode toggled: %s\n", s_walk_mode ? "ON" : "OFF");
            break;

        case MENU_TALK_MODE:
            s_ptt_mode = !s_ptt_mode;
            nvs_put_u8("ptt", s_ptt_mode ? 1 : 0);
            if (s_ptt_mode && audio::talking()) {
                xSemaphoreTake(s_app_mtx, portMAX_DELAY);
                audio::set_talking(false);
                xSemaphoreGive(s_app_mtx);
                broadcast_state();
            } else if (!s_ptt_mode) {
                uint8_t mic_saved = nvs_u8("mic", 0);
                xSemaphoreTake(s_app_mtx, portMAX_DELAY);
                audio::set_talking(mic_saved != 0);
                xSemaphoreGive(s_app_mtx);
                broadcast_state();
            }
            audio::prompt_clear();
            audio::prompt(s_ptt_mode ? wt::PROMPT_PTT_ON : wt::PROMPT_HANDS_FREE_ON);
            printf("[Menu] Talk mode toggled: %s\n", s_ptt_mode ? "Classic PTT" : "Hands-Free VOX");
            break;

        case MENU_CHANNEL: {
            uint8_t next_ch = (radio::channel() % 13) + 1;
            radio::set_channel(next_ch);
            nvs_put_u8("ch", next_ch);
            audio::prompt_clear();
            audio::prompt(static_cast<wt::PromptId>(wt::PROMPT_NUM_1 + (next_ch - 1)));
            printf("[Menu] Channel changed to: %u\n", next_ch);
            break;
        }

        case MENU_RADAR:
            printf("[Menu] Radar check triggered (staying in menu). Hold again to scan again, Click for next item.\n");
            radar_check();
            break;

        case MENU_VOICE_NOTE:
            exit_menu(false);
            start_voicemail_recording();
            break;

        case MENU_POWER_OFF:
            system_power_off();
            break;

        case MENU_EXIT:
            exit_menu(true);
            break;

        default:
            exit_menu(true);
            break;
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
    printf("\n%s  node %08lx  ch %u  vol %u%%  mic %s  mode %s%s\n", name, (unsigned long)radio::self_id(),
           radio::channel(), audio::volume(), audio::talking() ? "OPEN" : "muted",
           s_ptt_mode ? "PTT" : "VOX Hands-Free",
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
    printf("  system: Temp %.1f C | RAM Free %lu KB (min %lu KB)%s%s%s%s\n",
           temp_c, (unsigned long)free_ram, (unsigned long)min_ram,
           s_menu_active ? "  [MENU ACTIVE]" : "",
           s_distress_active ? "  [DISTRESS ACTIVE]" : "",
           s_distress_peer != 0 ? "  [REMOTE SOS HEARD]" : "",
           s_vm_outbox_pending ? "  [VOICE NOTE IN OUTBOX]" : "");
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
    printf("\n  t          talk on/off (same as BOOT or headset button click in VOX mode)\n"
           "  m / menu   open/close audio voice menu (same as double click)\n"
           "  ptt        toggle Classic PTT vs Hands-Free VOX mode\n"
           "  r / c      radar: check peer count and distance (same as triple click)\n"
           "  vm         record voice mail / send\n"
           "  send       send recorded voice mail (or flush outbox)\n"
           "  sos        distress SOS on/off (same as holding button 2.5s)\n"
           "  range [N]  run range test with N probes (default 20, measures loss %% & RSSI)\n"
           "  walk       toggle walk range mode (1 probe/sec with acoustic tone)\n"
           "  + / -      volume\n"
           "  p          ping all nodes\n"
           "  s          status, peers, counters\n"
           "  ch N       radio channel 1..13 (all nodes must match)\n"
           "  name X     set this node's name\n"
           "  loop       loopback test: hear your own voice through the codec\n"
           "  selftest   check headset streams and the audio path\n"
           "  off        power off / deep sleep (wakes on BOOT press)\n"
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
        else if (!strcmp(line, "m") || !strcmp(line, "menu")) {
            if (s_menu_active) exit_menu();
            else enter_menu();
        }
        else if (!strcmp(line, "ptt")) {
            s_ptt_mode = !s_ptt_mode;
            nvs_put_u8("ptt", s_ptt_mode ? 1 : 0);
            if (audio::talking()) toggle_talk();
            printf("Talk Mode: %s\n", s_ptt_mode ? "Classic PTT" : "Hands-Free VOX");
            audio::prompt_clear();
            audio::prompt(s_ptt_mode ? wt::PROMPT_PTT_ON : wt::PROMPT_HANDS_FREE_ON);
        }
        else if (!strcmp(line, "off") || !strcmp(line, "poweroff") || !strcmp(line, "shutdown")) {
            system_power_off();
        }
        else if (!strcmp(line, "r") || !strcmp(line, "c")) radar_check();
        else if (!strcmp(line, "vm") || !strcmp(line, "record")) {
            if (s_vm_recording) send_voicemail();
            else start_voicemail_recording();
        }
        else if (!strcmp(line, "send")) send_voicemail();
        else if (!strcmp(line, "play") || !strcmp(line, "pvm")) play_inbox_voicemail();
        else if (!strcmp(line, "dismiss") || !strcmp(line, "dvm")) dismiss_inbox_voicemail();
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

    esp_reset_reason_t rst_reason = esp_reset_reason();
    uint8_t pwr_state = nvs_u8("pwr_state", 0); // default 0: shutdown until BOOT is pressed

    if (rst_reason == ESP_RST_DEEPSLEEP) {
        // Woke up from Deep Sleep because user pressed BOOT!
        pwr_state = 1;
        nvs_put_u8("pwr_state", 1);
        ESP_LOGI(TAG, "Woke up from Deep Sleep via button press -> System Online");
    } else {
        // Cold boot, power-on, brownout, panic, watchdog, or restart
        if (pwr_state == 0) {
            // Deliberately shut down earlier; returning immediately to Deep Sleep until BOOT is pressed
            ESP_LOGI(TAG, "System was turned off; returning to Deep Sleep. Press BOOT to power on.");
            for (gpio_num_t p : {cfg::kPinBoot, cfg::kPinTalk}) init_gpio(p);
            esp_sleep_enable_ext0_wakeup(cfg::kPinBoot, 0);
            uint64_t mask = (1ULL << cfg::kPinBoot);
            if (cfg::kPinTalk != GPIO_NUM_NC) mask |= (1ULL << cfg::kPinTalk);
            esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_LOW);
            esp_deep_sleep_start();
        } else {
            // Was running online when power was disconnected or glitched -> auto-recover online!
            ESP_LOGI(TAG, "Auto-recovery: restoring online state after reset (reason %d)", rst_reason);
            pwr_state = 1;
            nvs_put_u8("pwr_state", 1);
        }
    }

    s_app_mtx = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    size_t len = sizeof s_name;
    if (nvs_get_str(s_nvs, "name", s_name, &len) != ESP_OK)
        snprintf(s_name, sizeof s_name, "node-%04x", static_cast<unsigned>(esp_random() & 0xFFFF));
    uint8_t ch = nvs_u8("ch", cfg::kDefaultChannel);
    audio::set_volume(nvs_u8("vol", cfg::kDefaultVolume));
    s_ptt_mode = (nvs_u8("ptt", 0) != 0);
    s_walk_mode = (nvs_u8("walk", 0) != 0);
    uint8_t mic_saved = nvs_u8("mic", 0);

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
    if (!s_ptt_mode && mic_saved) {
        audio::set_talking(true);
        ESP_LOGI(TAG, "Restored mic state: OPEN (Hands-Free VOX)");
    }
    if (s_walk_mode) {
        s_range_token = esp_random();
        s_walk_seq = 0;
        ESP_LOGI(TAG, "Restored Walk Mode: ACTIVE");
    }
    temperature_sensor_config_t temp_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&temp_cfg, &s_temp_sensor) == ESP_OK) {
        temperature_sensor_enable(s_temp_sensor);
    }
    usb_headset::start(on_hid_buttons, on_headset_state);
    xTaskCreatePinnedToCore(console_task, "console", 8192, nullptr, 3, nullptr, 0);
    printf("\n%s ready on channel %u (mode: %s). Type 'h' for commands.\n", s_name, ch,
           s_ptt_mode ? "Classic PTT" : "Hands-Free VOX");

    wt::ButtonDetector headset_btn, boot_btn, up, down;
    uint8_t last_toggle = 0;
    uint8_t last_hid = 0;
    uint32_t hid_talk_start = 0;
    bool hid_talk_long_sent = false;
    bool hid_talk_hold_sent = false;
    uint32_t last_beacon = 0, talk_since = 0;
    for (;;) {
        uint32_t now = now_ms();
        uint8_t hid = s_hid_down;

        static bool s_ready_announced = false;
        if (!s_ready_announced && usb_headset::spk_ready()) {
            s_ready_announced = true;
            audio::prompt(wt::PROMPT_READY);
        }

        // Latching hook/mute control: every change is one click
        uint8_t tog = hid & wt::HID_BTN_TALK_TOGGLE;
        static bool s_first_hid = true;
        if (s_first_hid) {
            s_first_hid = false;
            last_toggle = tog;
            last_hid = hid;
        } else if (tog != last_toggle) {
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
        // which == 0 (zero menu functions)
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

        // Physical GPIO buttons:
        // BOOT button on ESP32 board (dedicated detector for settings & menu, which == 3)
        handle_press(boot_btn.update(low(cfg::kPinBoot), now), 3);

        // Headset Talk external pin (which == 0)
        if (cfg::kPinTalk != GPIO_NUM_NC) {
            handle_press(headset_btn.update(low(cfg::kPinTalk), now), 0);
        }

        handle_press(up.update(low(cfg::kPinVolUp), now), 1);
        handle_press(down.update(low(cfg::kPinVolDown), now), 2);

        // Classic PTT mode: press and hold HEADSET button to talk, release to mute with roger beep
        // Note: strictly HEADSET button (not BOOT button)
        bool headset_raw_down = (hid & wt::HID_BTN_TALK) ||
                                (cfg::kPinTalk != GPIO_NUM_NC && low(cfg::kPinTalk));
        static bool s_last_headset_raw_down = false;
        static uint32_t s_talk_down_ms = 0;
        if (s_ptt_mode && !s_menu_active && !s_vm_recording && !s_distress_active) {
            if (headset_raw_down && !s_last_headset_raw_down) {
                s_talk_down_ms = now;
            }
            if (headset_raw_down && !s_ptt_active && (now - s_talk_down_ms >= 120)) {
                // Held >= 120ms: activate PTT transmission
                s_ptt_active = true;
                if (!audio::talking()) {
                    xSemaphoreTake(s_app_mtx, portMAX_DELAY);
                    audio::set_talking(true);
                    xSemaphoreGive(s_app_mtx);
                    broadcast_state();
                    audio::tone(880, 25, 350); // PTT talk start chirp
                    ESP_LOGI(TAG, "PTT: TX active (held)");
                }
            }
            if (!headset_raw_down && s_last_headset_raw_down) {
                if (s_ptt_active) {
                    s_ptt_active = false;
                    if (audio::talking()) {
                        xSemaphoreTake(s_app_mtx, portMAX_DELAY);
                        audio::set_talking(false);
                        xSemaphoreGive(s_app_mtx);
                        broadcast_state();
                        // Roger beep on release (descending two-tone)
                        audio::tone(587, 25, 300);
                        audio::tone(0, 10);
                        audio::tone(880, 35, 300);
                        ESP_LOGI(TAG, "PTT: TX ended (roger beep)");
                    }
                }
            }
        }
        s_last_headset_raw_down = headset_raw_down;

        // Handle pending click evaluation (for voicemail inbox 1x/2x click)
        if (s_talk_click_count > 0 && (now - s_last_talk_click_ms > kMultiClickWindowMs)) {
            s_talk_click_count = 0;
            if (s_vm_inbox_has_message) {
                play_inbox_voicemail();
            }
        }

        // Audio Voice Menu inactivity timeout (relaxed 30 seconds)
        if (s_menu_active && (now - s_last_menu_activity_ms >= 30000)) {
            exit_menu(true);
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
                audio::prompt_clear();
                audio::prompt(wt::PROMPT_VM_SENT);
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
            if (now - s_last_distress_beep_ms >= 10000) {
                s_last_distress_beep_ms = now;
                audio::tone(800, 30, 250);
            }
        } else if (s_distress_peer != 0) {
            if (now - s_last_distress_rx_ms > 8000) {
                s_distress_peer = 0;
                sound_all_clear();
            } else if (now - s_last_distress_beep_ms >= 12000) {
                s_last_distress_beep_ms = now;
                sound_distress_alert();
            }
        }

        update_led();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
