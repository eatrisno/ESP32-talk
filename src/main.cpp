// ESP32-S3 mesh walkie-talkie: USB-C headset, ESP-NOW, every node in range joins one
// conference automatically. See README.md.
#include <cmath>
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
enum WalkZone : uint8_t {
    WALK_ZONE_UNKNOWN = 0,
    WALK_ZONE_NEAR,     // RSSI >= -75 dBm (Strong)
    WALK_ZONE_MID,      // -76 to -90 dBm (Medium)
    WALK_ZONE_FAR,      // < -90 dBm (Weak / Edge)
    WALK_ZONE_OUT       // Link lost / Out of range
};
static WalkZone s_walk_zone = WALK_ZONE_UNKNOWN;
static uint8_t s_walk_miss_count = 0;
static uint32_t s_walk_last_spoken_ms = 0;

static volatile bool s_distress_active = false;
static uint32_t s_distress_activated_ms = 0;
static volatile uint32_t s_distress_peer = 0;
static char s_distress_peer_name[12] = "";
static uint32_t s_last_distress_tx_ms = 0;
static uint32_t s_last_distress_beep_ms = 0;
static uint32_t s_last_distress_rx_ms = 0;
static volatile bool s_vm_recording = false;
static temperature_sensor_handle_t s_temp_sensor = nullptr;

// --- Audio Voice Menu & PTT state ---
enum MenuItem : uint8_t {
    MENU_VOICE_NOTE = 0,
    MENU_TALK_MODE,
    MENU_RADAR,
    MENU_WALK,
    MENU_EXIT,
    MENU_COUNT
};

static bool s_ptt_mode = false;          // false = Hands-Free VOX, true = Classic PTT
static bool s_ptt_active = false;        // currently actively transmitting via PTT hold
static uint32_t s_last_ptt_ended_ms = 0; // timestamp when last PTT hold ended
static bool s_menu_active = false;
static MenuItem s_menu_item = MENU_VOICE_NOTE;
static uint32_t s_last_menu_activity_ms = 0;

// Collaborative Channel & Backup Watchdog state
static uint8_t  s_primary_channel = cfg::kDefaultChannel;
static uint8_t  s_backup_channel = cfg::kDefaultBackupChannel;
static uint32_t s_last_peer_rx_ms = 0;
static bool     s_fallback_active = false;
static uint32_t s_fallback_switch_ms = 0;

static volatile bool s_spectrum_collecting = false;
static uint32_t s_spectrum_token = 0;
static constexpr int kMaxSpectrumReports = 8;
static wt::SpectrumReportPayload s_spectrum_reports[kMaxSpectrumReports];
static int      s_num_spectrum_reports = 0;
static uint32_t s_last_spectrum_scan_ms = 0;
static uint16_t s_cached_local_congestion[14] = {0};
static bool     s_has_cached_spectrum = false;

// Auto-Join Channel Hunting on Boot & Auto Re-Consensus
static bool     s_boot_hunting_active = false;
static bool     s_boot_hunting_done = false;
static uint32_t s_boot_ms = 0;
static uint32_t s_last_hunt_step_ms = 0;
static uint8_t  s_hunt_ch_index = 0;
static uint32_t s_recovered_on_backup_ms = 0;

void broadcast_state();
void enter_menu();
void exit_menu(bool play_tone = true);
void menu_next();
void menu_prev();
void menu_action();
void announce_main_menu_item();
void measure_local_spectrum(uint16_t out_congestion[14]);
void collaborative_channel_scan(bool quiet = false);
void sync_channel_to_mesh(uint8_t primary_ch, uint8_t backup_ch, bool quiet = false);

uint32_t now_ms() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

// Debug Audio setting & anti-chatter audio alerts
static bool     s_debug_audio = false; // false = Quiet Snowboard Mode, true = Verbose Debug Alerts
static uint32_t s_last_connect_prompt_ms = 0;
static uint32_t s_last_disconnect_prompt_ms = 0;

void notify_connected(bool force_voice = false) {
    uint32_t now = now_ms();
    if ((force_voice || s_debug_audio) && (now - s_last_connect_prompt_ms >= 30000)) {
        s_last_connect_prompt_ms = now;
        audio::prompt_clear();
        audio::prompt(wt::PROMPT_CONNECTED);
    } else {
        // Subtle soft 2-tone chirp (pleasant and non-intrusive on slopes)
        audio::tone(700, 18, 90);
        audio::tone(0, 10);
        audio::tone(950, 22, 110);
    }
}

void notify_disconnected(bool force_voice = false) {
    uint32_t now = now_ms();
    if ((force_voice || s_debug_audio) && (now - s_last_disconnect_prompt_ms >= 30000)) {
        s_last_disconnect_prompt_ms = now;
        audio::prompt_clear();
        audio::prompt(wt::PROMPT_DISCONNECTED);
    } else {
        audio::tone(250, 25, 180);
    }
}

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
        s_distress_activated_ms = now_ms();
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

void toggle_ptt() {
    xSemaphoreTake(s_app_mtx, portMAX_DELAY);
    s_ptt_active = !s_ptt_active;
    bool on = s_ptt_active;
    audio::set_ptt_active(on);
    audio::set_talking(on);
    xSemaphoreGive(s_app_mtx);
    broadcast_state();
    if (on) {
        audio::tone(880, 25, 350);  // Crisp walkie-talkie TX start chirp
        ESP_LOGI(TAG, "PTT: TX ON (Toggle / 1x Click)");
    } else {
        // Roger beep on TX end (587Hz -> 880Hz)
        audio::tone(587, 25, 300);
        audio::tone(0, 10);
        audio::tone(880, 35, 300);
        s_last_ptt_ended_ms = now_ms();
        ESP_LOGI(TAG, "PTT: TX OFF (Toggle / 1x Click - Roger beep sent)");
    }
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
static constexpr uint32_t kMultiClickWindowMs = 600;

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
            } else if (p == wt::Press::Long || p == wt::Press::Hold) {
                // Fast shortcut: Hold BOOT button toggles Talk Mode (PTT <-> Hands-Free VOX) immediately!
                s_ptt_mode = !s_ptt_mode;
                nvs_put_u8("ptt", s_ptt_mode ? 1 : 0);
                if (s_ptt_mode && audio::talking()) {
                    xSemaphoreTake(s_app_mtx, portMAX_DELAY);
                    audio::set_talking(false);
                    audio::set_ptt_active(false);
                    xSemaphoreGive(s_app_mtx);
                    broadcast_state();
                } else if (!s_ptt_mode) {
                    uint8_t mic_saved = nvs_u8("mic", 0);
                    xSemaphoreTake(s_app_mtx, portMAX_DELAY);
                    audio::set_talking(mic_saved != 0);
                    audio::set_ptt_active(false);
                    xSemaphoreGive(s_app_mtx);
                    broadcast_state();
                }
                audio::prompt_clear();
                audio::prompt(s_ptt_mode ? wt::PROMPT_PTT_ON : wt::PROMPT_HANDS_FREE_ON);
                led::flash(s_ptt_mode ? led::Color::Green : led::Color::Blue, 250);
                printf("[Fast Toggle] Talk mode changed to %s\n", s_ptt_mode ? "Classic PTT" : "Hands-Free VOX");
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
        uint32_t now = now_ms();
        if (s_distress_active) {
            // Guard window: ignore any extra clicks in the initial burst of 5+ clicks
            if (now - s_distress_activated_ms < 1500) {
                return;
            }
            // After guard window, any click or press cancels distress!
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
            if (s_talk_click_count > 0 && (now - s_last_talk_click_ms <= kMultiClickWindowMs)) {
                s_talk_click_count++;
            } else {
                s_talk_click_count = 1;
            }
            s_last_talk_click_ms = now;

            if (s_talk_click_count >= 5) {
                s_talk_click_count = 0;
                ESP_LOGI(TAG, "5 rapid clicks detected -> Emergency SOS activated!");
                toggle_distress();
                return;
            }

            if (s_vm_inbox_has_message) {
                // Voicemail inbox action will be evaluated when multi-click window expires
                return;
            }

            // Normal standby operation:
            if (!s_ptt_mode) {
                // Mode Call (Hands-Free VOX): 1x click toggles mic (mute / unmute)!
                toggle_talk();
            } else {
                // Mode PTT (Option A): 1x click toggles TX (start talk / stop talk + Roger Beep)!
                toggle_ptt();
            }
        } else if (p == wt::Press::Long || p == wt::Press::Hold) {
            // Long and Hold on headset button are ignored (PTT talk can be held indefinitely;
            // Emergency SOS is strictly triggered by 5 rapid clicks).
            return;
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
    s_last_peer_rx_ms = now_ms();
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
                wt::RangeResp resp = {};
                resp.token = req.token;
                resp.seq = req.seq;
                resp.req_rssi = rx.rssi;
                xSemaphoreTake(s_app_mtx, portMAX_DELAY);
                strncpy(resp.name, s_name, sizeof(resp.name) - 1);
                xSemaphoreGive(s_app_mtx);
                radio::send(wt::MSG_RANGE_RESP, reinterpret_cast<const uint8_t*>(&resp), sizeof(resp));
                led::flash(led::Color::Cyan, 40);
            }
            break;
        }
        case wt::MSG_RANGE_RESP: {
            if (rx.len >= sizeof(wt::RangeResp)) {
                wt::RangeResp resp;
                memcpy(&resp, rx.plain, sizeof(resp));
                resp.name[sizeof(resp.name) - 1] = 0;
                if (resp.token == s_range_token) {
                    s_walk_pending = false;
                    s_walk_miss_count = 0;
                    s_range_stats.add_sample(resp.req_rssi, rx.rssi);

                    // Signal Quality percentage (-45 dBm = 100%, -105 dBm = 5%)
                    int pct = (rx.rssi - (-105)) * 100 / (-45 - (-105));
                    if (pct > 100) pct = 100;
                    if (pct < 5) pct = 5;

                    WalkZone new_zone;
                    if (rx.rssi >= -75) new_zone = WALK_ZONE_NEAR;
                    else if (rx.rssi >= -90) new_zone = WALK_ZONE_MID;
                    else new_zone = WALK_ZONE_FAR;

                    uint32_t now = now_ms();
                    if (s_walk_zone == WALK_ZONE_OUT) {
                        // Reconnected after losing signal!
                        notify_connected();
                        s_walk_last_spoken_ms = now;
                    } else if (new_zone != s_walk_zone && (now - s_walk_last_spoken_ms >= 5000)) {
                        // Zone transition: voice announcement of distance
                        s_walk_last_spoken_ms = now;
                        audio::prompt_clear();
                        if (new_zone == WALK_ZONE_NEAR) audio::prompt(wt::PROMPT_RADAR_1_NEAR);
                        else if (new_zone == WALK_ZONE_MID) audio::prompt(wt::PROMPT_RADAR_1_MID);
                        else if (new_zone == WALK_ZONE_FAR) audio::prompt(wt::PROMPT_RADAR_1_FAR);
                    }
                    s_walk_zone = new_zone;

                    // Real-time acoustic pitch (dramatically distinct tones)
                    if (!audio::prompt_busy()) {
                        if (new_zone == WALK_ZONE_NEAR) {
                            // High bright double-chirp: ding-ding! (800Hz -> 1000Hz)
                            audio::tone(800, 25, 400);
                            audio::tone(0, 10);
                            audio::tone(1000, 35, 450);
                        } else if (new_zone == WALK_ZONE_MID) {
                            // Solid single mid-tone: 600Hz
                            audio::tone(600, 45, 380);
                        } else {
                            // Deep low warning pip: 380Hz
                            audio::tone(380, 55, 400);
                        }
                    }

                    // Bidirectional RSSI average (combines Downlink and Uplink to eliminate antenna orientation mismatch)
                    float eff_rssi = (static_cast<float>(rx.rssi) + static_cast<float>(resp.req_rssi)) / 2.0f;

                    // Exponential Moving Average filter to smooth out indoor multipath reflections
                    static float s_smoothed_rssi = -50.0f;
                    static bool s_smoothed_init = false;
                    if (!s_smoothed_init) {
                        s_smoothed_rssi = eff_rssi;
                        s_smoothed_init = true;
                    } else {
                        s_smoothed_rssi = 0.65f * s_smoothed_rssi + 0.35f * eff_rssi;
                    }

                    // Calibrated Log-Distance Path-Loss for ESP32-S3 PCB antenna (+21 dBm TX):
                    // Reference RSSI at 1 meter: ~ -50 dBm
                    // Path loss exponent n = 2.4 (indoor/outdoor mixed environment)
                    float est_m = powf(10.0f, (-50.0f - s_smoothed_rssi) / 24.0f);
                    if (est_m < 0.2f) est_m = 0.2f;

                    char who[20] = "";
                    if (resp.name[0]) {
                        snprintf(who, sizeof(who), "%s", resp.name);
                    } else {
                        radio::peers_lock();
                        if (const wt::Peer* p = radio::peers().find(id)) {
                            if (p->name[0]) snprintf(who, sizeof(who), "%s", p->name);
                            else snprintf(who, sizeof(who), "node-%04lx", (unsigned long)(id & 0xFFFF));
                        } else {
                            snprintf(who, sizeof(who), "node-%04lx", (unsigned long)(id & 0xFFFF));
                        }
                        radio::peers_unlock();
                    }

                    if (est_m < 1.0f) {
                        printf("  probe #%-2u from [%-10s]: OK | %3d%% Link | RSSI: %4d dBm (est. ~%d cm) | Uplink: %4d dBm | Zone: %s\n",
                               resp.seq, who, pct, rx.rssi, static_cast<int>(est_m * 100), resp.req_rssi,
                               new_zone == WALK_ZONE_NEAR ? "NEAR (Strong)" : (new_zone == WALK_ZONE_MID ? "MID (Medium)" : "FAR (Weak)"));
                    } else {
                        printf("  probe #%-2u from [%-10s]: OK | %3d%% Link | RSSI: %4d dBm (est. ~%.1f m) | Uplink: %4d dBm | Zone: %s\n",
                               resp.seq, who, pct, rx.rssi, est_m, resp.req_rssi,
                               new_zone == WALK_ZONE_NEAR ? "NEAR (Strong)" : (new_zone == WALK_ZONE_MID ? "MID (Medium)" : "FAR (Weak)"));
                    }
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
        case wt::MSG_CHANNEL_SWITCH: {
            if (rx.len >= sizeof(wt::ChannelSwitchPayload)) {
                wt::ChannelSwitchPayload csp;
                memcpy(&csp, rx.plain, sizeof(csp));
                if (csp.target_channel >= 1 && csp.target_channel <= 13) {
                    s_primary_channel = csp.target_channel;
                    s_backup_channel = (csp.backup_channel >= 1 && csp.backup_channel <= 13) ? csp.backup_channel : 0;
                    s_fallback_active = false;
                    nvs_put_u8("ch", s_primary_channel);
                    nvs_put_u8("backup_ch", s_backup_channel);

                    if (csp.target_channel != radio::channel()) {
                        printf("\n*** [CHANNEL SYNC] Remote command from %08lx: switching to Primary Ch %u (Backup: %u) ***\n\n",
                               (unsigned long)csp.sender, csp.target_channel, csp.backup_channel);
                        radio::set_channel(csp.target_channel);
                        if (s_debug_audio) {
                            audio::prompt_clear();
                            audio::prompt(wt::PROMPT_CHANGED_TO_CHANNEL);
                            audio::prompt(static_cast<wt::PromptId>(wt::PROMPT_NUM_1 + (csp.target_channel - 1)));
                        } else {
                            // Soft 2-tone melodic pip (no loud voice prompt while riding)
                            audio::tone(500, 20, 80);
                            audio::tone(0, 10);
                            audio::tone(750, 20, 100);
                        }
                        led::flash(led::Color::Green, 250);
                    }
                }
            }
            break;
        }
        case wt::MSG_SPECTRUM_PROBE: {
            if (rx.len >= sizeof(wt::SpectrumProbePayload)) {
                wt::SpectrumProbePayload spp;
                memcpy(&spp, rx.plain, sizeof(spp));
                if (spp.sender != radio::self_id()) {
                    uint32_t now = now_ms();
                    if (!s_has_cached_spectrum || (now - s_last_spectrum_scan_ms > 30000)) {
                        measure_local_spectrum(s_cached_local_congestion);
                        s_last_spectrum_scan_ms = now;
                        s_has_cached_spectrum = true;
                    }
                    wt::SpectrumReportPayload rep;
                    rep.token = spp.token;
                    rep.sender = radio::self_id();
                    memcpy(rep.congestion, s_cached_local_congestion, sizeof(rep.congestion));
                    radio::send(wt::MSG_SPECTRUM_REPORT, reinterpret_cast<const uint8_t*>(&rep), sizeof(rep));
                }
            }
            break;
        }
        case wt::MSG_SPECTRUM_REPORT: {
            if (s_spectrum_collecting && rx.len >= sizeof(wt::SpectrumReportPayload)) {
                wt::SpectrumReportPayload rep;
                memcpy(&rep, rx.plain, sizeof(rep));
                if (rep.token == s_spectrum_token && s_num_spectrum_reports < kMaxSpectrumReports) {
                    s_spectrum_reports[s_num_spectrum_reports++] = rep;
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
        s_walk_zone = WALK_ZONE_UNKNOWN;
        s_walk_miss_count = 0;
        s_walk_last_spoken_ms = 0;
        printf("\nWALK RANGE MODE: ON (emitting 1 probe/sec with acoustic pitch)\n");
        printf("Put on headset and walk away. Tone pitch indicates signal strength!\n\n");
    } else {
        printf("\nWALK RANGE MODE: OFF\n\n");
    }
}

// --- Audio Voice Menu Engine -----------------------------------------------------------

void measure_local_spectrum(uint16_t out_congestion[14]) {
    memset(out_congestion, 0, sizeof(uint16_t) * 14);
    wifi_scan_config_t scan_config = {};
    scan_config.show_hidden = true;
    scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan_config.scan_time.active.min = 40;
    scan_config.scan_time.active.max = 80;

    uint8_t cur_ch = radio::channel();
    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WiFi scan start failed (%d)", err);
        radio::set_channel(cur_ch);
        return;
    }

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);

    uint32_t congestion[14] = {0}; // channels 1..13
    if (ap_count > 0) {
        wifi_ap_record_t* ap_list = static_cast<wifi_ap_record_t*>(malloc(sizeof(wifi_ap_record_t) * ap_count));
        if (ap_list) {
            esp_wifi_scan_get_ap_records(&ap_count, ap_list);
            for (uint16_t i = 0; i < ap_count; i++) {
                uint8_t ch = ap_list[i].primary;
                int8_t rssi = ap_list[i].rssi;
                uint32_t weight = static_cast<uint32_t>(rssi > -100 ? (rssi + 105) : 5);
                if (ch >= 1 && ch <= 13) {
                    // Full 20MHz bandwidth (+/- 2 ch) plus spectral sideband bleed (+/- 3 ch).
                    // An active router on Ch 2 occupies 1..4 and bleeds sideband into Ch 5.
                    congestion[ch] += weight * 5;
                    if (ch >= 2) congestion[ch - 1] += weight * 4;
                    if (ch <= 12) congestion[ch + 1] += weight * 4;
                    if (ch >= 3) congestion[ch - 2] += weight * 3;
                    if (ch <= 11) congestion[ch + 2] += weight * 3;
                    if (ch >= 4) congestion[ch - 3] += weight * 2;
                    if (ch <= 10) congestion[ch + 3] += weight * 2;
                }
            }
            free(ap_list);
        }
    }

    // Always restore current channel after scan finishes
    radio::set_channel(cur_ch);

    for (uint8_t c = 1; c <= 13; c++) {
        out_congestion[c] = (congestion[c] > 65535) ? 65535 : static_cast<uint16_t>(congestion[c]);
    }
}

void collaborative_channel_scan(bool quiet) {
    if (!quiet) {
        audio::prompt_clear();
        audio::prompt(wt::PROMPT_SCANNING_CHANNEL);
        printf("\n*** [COLLABORATIVE SCAN] Probing mesh peers & scanning 2.4GHz spectrum... ***\n");
    } else {
        printf("\n*** [AUTO RE-CONSENSUS] Background spectrum scan triggered... ***\n");
    }

    s_spectrum_collecting = true;
    s_spectrum_token = esp_random();
    s_num_spectrum_reports = 0;

    // Send probe to mesh peers so they start scanning
    wt::SpectrumProbePayload probe;
    probe.token = s_spectrum_token;
    probe.sender = radio::self_id();
    for (int i = 0; i < 3; i++) {
        radio::send(wt::MSG_SPECTRUM_PROBE, reinterpret_cast<const uint8_t*>(&probe), sizeof(probe));
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    // Measure local spectrum
    measure_local_spectrum(s_cached_local_congestion);
    s_last_spectrum_scan_ms = now_ms();
    s_has_cached_spectrum = true;

    // Wait up to 1.5s for peer reports to arrive
    uint32_t wait_start = now_ms();
    while (now_ms() - wait_start < 1500) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    s_spectrum_collecting = false;

    printf("\n=== [SPECTRUM CONSENSUS ANALYSIS (Local + %d Peer(s))] ===\n", s_num_spectrum_reports);
    printf("  Ch   Local  ");
    for (int p = 0; p < s_num_spectrum_reports; p++) {
        printf("Peer%d  ", p + 1);
    }
    printf("Total Congestion\n");

    uint32_t total_congestion[14] = {0};
    for (uint8_t c = 1; c <= 13; c++) {
        total_congestion[c] = s_cached_local_congestion[c];
        printf("  %2u:  %5u  ", c, s_cached_local_congestion[c]);
        for (int p = 0; p < s_num_spectrum_reports; p++) {
            uint16_t peer_score = s_spectrum_reports[p].congestion[c];
            total_congestion[c] += peer_score;
            printf("%5u  ", peer_score);
        }
        printf("--> %lu\n", (unsigned long)total_congestion[c]);
    }

    // Channel selection prioritizing clean high-band (Ch 13 / 11) and orthogonal backup (Ch 6)
    // Preference order for tie-breaking: 13, 11, 6, 12, 10, 9, 8, 7, 1, 5, 4, 3, 2
    static const uint8_t kPreferredChannels[] = {13, 11, 6, 12, 10, 9, 8, 7, 1, 5, 4, 3, 2};
    uint8_t best_ch = 13;
    uint32_t min1 = UINT32_MAX;
    for (uint8_t c : kPreferredChannels) {
        if (total_congestion[c] < min1) {
            min1 = total_congestion[c];
            best_ch = c;
        }
    }

    // Find Second Best Channel with orthogonal guard band (at least 4 channels away from best_ch)
    uint8_t backup_ch = (best_ch == 13 || best_ch == 11) ? 6 : 13;
    uint32_t min2 = UINT32_MAX;
    for (uint8_t c : kPreferredChannels) {
        if (c == best_ch) continue;
        if (abs(static_cast<int>(c) - static_cast<int>(best_ch)) < 4) continue;
        if (total_congestion[c] < min2) {
            min2 = total_congestion[c];
            backup_ch = c;
        }
    }

    uint32_t cur_congestion = total_congestion[s_primary_channel];
    bool need_primary_switch = false;

    // Switch criteria:
    // 1) Fallback was active or we just recovered on backup channel
    // 2) Or current channel congestion is substantially worse than best channel
    if (s_fallback_active || (s_recovered_on_backup_ms > 0)) {
        need_primary_switch = (best_ch != s_primary_channel);
    } else if (best_ch != s_primary_channel) {
        // Hysteresis: only switch if current channel is congested (>= 25)
        // AND best channel is better by at least 25 points.
        if (cur_congestion >= 25 && cur_congestion > min1 + 25) {
            need_primary_switch = true;
            printf("*** [AUTO-CONSENSUS] Area congestion detected on Ch %u (Score: %lu vs best Ch %u: %lu). Initiating switch... ***\n",
                   s_primary_channel, (unsigned long)cur_congestion, best_ch, (unsigned long)min1);
        } else {
            // Retain current channel to avoid unnecessary hops
            best_ch = s_primary_channel;
            printf("*** [AUTO-CONSENSUS] Current Ch %u remains adequate (Score: %lu vs best Ch: %lu). Retaining channel. ***\n",
                   s_primary_channel, (unsigned long)cur_congestion, (unsigned long)min1);
        }
    }

    if (need_primary_switch) {
        printf("\n>>> CONSENSUS DECISION FOR MESH: <<<\n");
        printf("  [PRIMARY] Channel %u (Score: %lu) - Best for all nodes\n", best_ch, (unsigned long)min1);
        printf("  [BACKUP ] Channel %u (Score: %lu) - Second best for recovery (orthogonal)\n\n", backup_ch, (unsigned long)min2);
        sync_channel_to_mesh(best_ch, backup_ch, quiet);
    } else if (backup_ch != s_backup_channel) {
        printf("*** [AUTO-CONSENSUS] Updating Backup Channel to %u (Primary remains Ch %u) ***\n",
               backup_ch, s_primary_channel);
        s_backup_channel = backup_ch;
        nvs_put_u8("backup_ch", backup_ch);
        wt::ChannelSwitchPayload csp;
        csp.sender = radio::self_id();
        csp.target_channel = s_primary_channel;
        csp.backup_channel = backup_ch;
        csp.seq = esp_random();
        for (int i = 0; i < 3; i++) {
            radio::send(wt::MSG_CHANNEL_SWITCH, reinterpret_cast<const uint8_t*>(&csp), sizeof(csp));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } else {
        printf("*** [AUTO-CONSENSUS] Mesh optimal on Ch %u (Backup: %u). No change needed. ***\n",
               s_primary_channel, s_backup_channel);
    }
}

void sync_channel_to_mesh(uint8_t primary_ch, uint8_t backup_ch, bool quiet) {
    wt::ChannelSwitchPayload csp;
    csp.sender = radio::self_id();
    csp.target_channel = primary_ch;
    csp.backup_channel = backup_ch;
    csp.seq = esp_random();

    printf("\n*** [CHANNEL SYNC] Broadcasting switch to Primary Ch %u (Backup: %u) to mesh... ***\n",
           primary_ch, backup_ch);

    // Send 5 packets on current channel so all peers on this channel receive the switch command
    for (int i = 0; i < 5; i++) {
        radio::send(wt::MSG_CHANNEL_SWITCH, reinterpret_cast<const uint8_t*>(&csp), sizeof(csp));
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    // Switch local channel and persist to NVS
    radio::set_channel(primary_ch);
    s_primary_channel = primary_ch;
    s_backup_channel = backup_ch;
    s_fallback_active = false;
    nvs_put_u8("ch", primary_ch);
    nvs_put_u8("backup_ch", backup_ch);

    // Send 3 confirmation packets on new channel
    for (int i = 0; i < 3; i++) {
        radio::send(wt::MSG_CHANNEL_SWITCH, reinterpret_cast<const uint8_t*>(&csp), sizeof(csp));
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    if (!quiet && s_debug_audio) {
        audio::prompt_clear();
        audio::prompt(wt::PROMPT_CHANGED_TO_CHANNEL);
        audio::prompt(static_cast<wt::PromptId>(wt::PROMPT_NUM_1 + (primary_ch - 1)));
    } else {
        // Soft 2-tone melodic pip (no loud voice prompt while riding)
        audio::tone(500, 20, 80);
        audio::tone(0, 10);
        audio::tone(750, 20, 100);
    }
    led::flash(led::Color::Green, 200);
    printf("*** [CHANNEL SYNC] Local node active on Primary Ch %u (Backup: %u) ***\n\n", primary_ch, backup_ch);
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
        case MENU_VOICE_NOTE:
            audio::prompt(wt::PROMPT_MENU_VOICE_NOTE);
            break;
        case MENU_TALK_MODE:
            audio::prompt(s_ptt_mode ? wt::PROMPT_PTT_ON : wt::PROMPT_HANDS_FREE_ON);
            break;
        case MENU_RADAR:
            audio::prompt(wt::PROMPT_MENU_RADAR);
            break;
        case MENU_WALK:
            audio::prompt(s_walk_mode ? wt::PROMPT_WALK_ON : wt::PROMPT_WALK_OFF);
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
    s_menu_item = MENU_VOICE_NOTE;
    s_last_menu_activity_ms = now_ms();
    audio::prompt_clear();
    audio::prompt(wt::PROMPT_MENU);
    audio::prompt(wt::PROMPT_MENU_VOICE_NOTE);
    printf("[Menu] Entered Audio Menu. 1st: Voice Note. Click: next, Hold: action, Hold 2.5s: exit.\n");
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
        case MENU_VOICE_NOTE:
            exit_menu(false);
            start_voicemail_recording();
            break;

        case MENU_TALK_MODE:
            s_ptt_mode = !s_ptt_mode;
            nvs_put_u8("ptt", s_ptt_mode ? 1 : 0);
            if (s_ptt_mode && audio::talking()) {
                xSemaphoreTake(s_app_mtx, portMAX_DELAY);
                audio::set_talking(false);
                audio::set_ptt_active(false);
                xSemaphoreGive(s_app_mtx);
                broadcast_state();
            } else if (!s_ptt_mode) {
                uint8_t mic_saved = nvs_u8("mic", 0);
                xSemaphoreTake(s_app_mtx, portMAX_DELAY);
                audio::set_talking(mic_saved != 0);
                audio::set_ptt_active(false);
                xSemaphoreGive(s_app_mtx);
                broadcast_state();
            }
            audio::prompt_clear();
            audio::prompt(s_ptt_mode ? wt::PROMPT_PTT_ON : wt::PROMPT_HANDS_FREE_ON);
            printf("[Menu] Talk mode toggled: %s\n", s_ptt_mode ? "Classic PTT" : "Hands-Free VOX");
            break;

        case MENU_RADAR:
            printf("[Menu] Radar check triggered (staying in menu). Hold again to scan again, Click for next item.\n");
            radar_check();
            break;

        case MENU_WALK:
            toggle_walk_mode();
            audio::prompt_clear();
            audio::prompt(s_walk_mode ? wt::PROMPT_WALK_ON : wt::PROMPT_WALK_OFF);
            printf("[Menu] Walk mode toggled: %s\n", s_walk_mode ? "ON" : "OFF");
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
    printf("\n%s  node %08lx  ch %u (pri %u, bkp %u%s)  vol %u%%  mic %s  mode %s%s%s\n", name, (unsigned long)radio::self_id(),
           radio::channel(), s_primary_channel, s_backup_channel, s_fallback_active ? " [FALLBACK]" : "",
           audio::volume(), audio::talking() ? "OPEN" : "muted",
           s_ptt_mode ? "PTT" : "VOX Hands-Free",
           s_debug_audio ? "  [DEBUG AUDIO]" : "  [QUIET AUDIO]",
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
           "  scan       collaborative spectrum scan: consensus best channel & backup for all nodes\n"
           "  ftm        measure precise distance via Wi-Fi FTM (Fine Timing Measurement)\n"
           "  debug      toggle quiet snowboard mode vs verbose debug alerts\n"
           "  + / -      volume\n"
           "  p          ping all nodes\n"
           "  s          status, peers, counters\n"
           "  ch N       radio channel 1..13 (all nodes must match)\n"
           "  backup N   set backup channel 0..13 (0 = disabled)\n"
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
        if (!strcmp(line, "t")) {
            if (s_ptt_mode) toggle_ptt();
            else toggle_talk();
        }
        else if (!strcmp(line, "m") || !strcmp(line, "menu")) {
            if (s_menu_active) exit_menu();
            else enter_menu();
        }
        else if (!strcmp(line, "ptt")) {
            s_ptt_mode = !s_ptt_mode;
            nvs_put_u8("ptt", s_ptt_mode ? 1 : 0);
            if (audio::talking() || s_ptt_active) {
                xSemaphoreTake(s_app_mtx, portMAX_DELAY);
                s_ptt_active = false;
                audio::set_ptt_active(false);
                audio::set_talking(false);
                xSemaphoreGive(s_app_mtx);
                broadcast_state();
            }
            printf("Talk Mode: %s\n", s_ptt_mode ? "Toggle PTT" : "Hands-Free VOX");
            audio::prompt_clear();
            audio::prompt(s_ptt_mode ? wt::PROMPT_PTT_ON : wt::PROMPT_HANDS_FREE_ON);
        }
        else if (!strcmp(line, "debug")) {
            s_debug_audio = !s_debug_audio;
            nvs_put_u8("dbg", s_debug_audio ? 1 : 0);
            printf("\nDebug Audio / Verbose Alerts: %s\n", s_debug_audio ? "ON (Verbose Alerts)" : "OFF (Quiet Snowboard Mode)");
            printf("  When OFF: Anti-chatter enabled. No spammy 'Connected' voice prompts; gentle chimes only.\n");
            printf("  When ON: Full spoken voice alerts on every reconnect and channel scan.\n\n");
            audio::prompt_clear();
            audio::tone(s_debug_audio ? 800 : 400, 30, 200);
        }
        else if (!strcmp(line, "scan")) {
            collaborative_channel_scan();
        }
        else if (!strcmp(line, "ftm")) {
            uint8_t direct_mac[6] = {0};
            bool found_peer = false;
            radio::peers_lock();
            for (const wt::Peer* p = radio::peers().begin(); p != radio::peers().end(); ++p) {
                if (p->used && p->hops == 0) {
                    for (int i = 0; i < 6; i++) {
                        if (p->mac[i] != 0) {
                            memcpy(direct_mac, p->mac, 6);
                            found_peer = true;
                            break;
                        }
                    }
                    if (found_peer) break;
                }
            }
            radio::peers_unlock();
            if (found_peer) {
                wifi_ftm_initiator_cfg_t ftmi_cfg = {};
                memcpy(ftmi_cfg.resp_mac, direct_mac, 6);
                ftmi_cfg.channel = radio::channel();
                ftmi_cfg.frm_count = 16;
                ftmi_cfg.burst_period = 2;
                printf("[FTM] Measuring nanosecond RTT distance to peer %02x:%02x:%02x:%02x:%02x:%02x on Ch %u...\n",
                       direct_mac[0], direct_mac[1], direct_mac[2], direct_mac[3], direct_mac[4], direct_mac[5], radio::channel());
                esp_err_t err = esp_wifi_ftm_initiate_session(&ftmi_cfg);
                if (err != ESP_OK) {
                    printf("[FTM] Failed to initiate session: %s\n", esp_err_to_name(err));
                }
            } else {
                printf("[FTM] No direct peer MAC available in peer table. Send a ping or walk probe first!\n");
            }
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
            int target_ch = atoi(line + 3);
            if (target_ch >= 1 && target_ch <= 13) {
                sync_channel_to_mesh(static_cast<uint8_t>(target_ch), s_backup_channel);
            } else printf("channel must be 1..13\n");
        } else if (!strncmp(line, "backup ", 7)) {
            int bkp = atoi(line + 7);
            if (bkp >= 0 && bkp <= 13) {
                s_backup_channel = static_cast<uint8_t>(bkp);
                nvs_put_u8("backup_ch", s_backup_channel);
                printf("Backup channel set to: %u\n", s_backup_channel);
            } else printf("backup channel must be 0 (none) or 1..13\n");
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

static void ftm_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_FTM_REPORT) {
        wifi_event_ftm_report_t *event = static_cast<wifi_event_ftm_report_t *>(event_data);
        if (event->status == FTM_STATUS_SUCCESS) {
            uint32_t dist_cm = event->dist_est;
            uint32_t rtt_ns = event->rtt_est;
            printf("\n*** [FTM PRECISION DISTANCE] Peer %02x:%02x:%02x:%02x:%02x:%02x -> Distance: %lu cm (%.2f m) | RTT: %lu ns ***\n\n",
                   event->peer_mac[0], event->peer_mac[1], event->peer_mac[2],
                   event->peer_mac[3], event->peer_mac[4], event->peer_mac[5],
                   (unsigned long)dist_cm, dist_cm / 100.0f, (unsigned long)rtt_ns);
        } else {
            printf("[FTM] Session status: %d (FTM measurement unavailable, using RF SNR)\n", event->status);
        }
    }
}

}  // namespace

extern "C" void app_main() {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_open("wt", NVS_READWRITE, &s_nvs));

    ESP_LOGI(TAG, "ESP32 Walkie-Talkie Online (reset reason: %d)", esp_reset_reason());

    s_app_mtx = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_FTM_REPORT, ftm_event_handler, NULL, NULL);

    size_t len = sizeof s_name;
    if (nvs_get_str(s_nvs, "name", s_name, &len) != ESP_OK)
        snprintf(s_name, sizeof s_name, "node-%04x", static_cast<unsigned>(esp_random() & 0xFFFF));
    uint8_t ch = nvs_u8("ch", cfg::kDefaultChannel);
    s_primary_channel = ch;
    s_backup_channel = nvs_u8("backup_ch", cfg::kDefaultBackupChannel);
    audio::set_volume(nvs_u8("vol", cfg::kDefaultVolume));
    s_ptt_mode = (nvs_u8("ptt", 1) != 0);
    s_walk_mode = (nvs_u8("walk", 0) != 0);
    s_debug_audio = (nvs_u8("dbg", 0) != 0);
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
    printf("\n%s ready on channel %u (mode: %s, audio: %s). Type 'h' for commands.\n", s_name, ch,
           s_ptt_mode ? "Classic PTT" : "Hands-Free VOX",
           s_debug_audio ? "Debug" : "Quiet Snowboard");

    s_boot_ms = now_ms();
    wt::ButtonDetector headset_btn, boot_btn, up, down;
    uint8_t last_toggle = 0;
    uint8_t last_hid = 0;
    uint32_t hid_talk_start = 0;
    bool hid_talk_long_sent = false;
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
        // Only active in Hands-Free mode (not PTT mode, to prevent interference)
        uint8_t tog = hid & wt::HID_BTN_TALK_TOGGLE;
        static bool s_first_hid = true;
        if (s_first_hid) {
            s_first_hid = false;
            last_toggle = tog;
            last_hid = hid;
        } else if (tog != last_toggle) {
            last_toggle = tog;
            if (!s_ptt_mode) {  // HID hook/toggle only acts in Hands-Free mode
                handle_press(wt::Press::Click, 0);
            }
        }

        // USB HID buttons: packets are digital and bounce-free; act directly on edge
        if ((hid & wt::HID_BTN_VOL_UP) && !(last_hid & wt::HID_BTN_VOL_UP)) {
            handle_press(wt::Press::Click, 1);
        }
        if ((hid & wt::HID_BTN_VOL_DOWN) && !(last_hid & wt::HID_BTN_VOL_DOWN)) {
            handle_press(wt::Press::Click, 2);
        }

        // Talk / Media button on headset (Play/Pause, Voice Command, Mute):
        // In Hands-Free VOX mode: 1x click toggles mic (VOX gating active).
        // In PTT mode (Option A): 1x click toggles TX (start talk / stop talk + Roger Beep).
        if ((hid & wt::HID_BTN_TALK) && !(last_hid & wt::HID_BTN_TALK)) {
            hid_talk_start = now;
            hid_talk_long_sent = false;
        }
        if ((hid & wt::HID_BTN_TALK) && !hid_talk_long_sent && (now - hid_talk_start >= 400)) {
            hid_talk_long_sent = true;
        }
        if (!(hid & wt::HID_BTN_TALK) && (last_hid & wt::HID_BTN_TALK)) {
            // On HID TALK release: fire click event for both Hands-Free and PTT Toggle mode
            if (!hid_talk_long_sent) {
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

        // Optional physical GPIO 4 momentary PTT (if external switch is connected to Pin 4)
        bool gpio_ptt_down = (cfg::kPinTalk != GPIO_NUM_NC && low(cfg::kPinTalk));
        static bool s_last_gpio_ptt_down = false;
        static uint32_t s_gpio_down_ms = 0;
        if (s_ptt_mode && !s_menu_active && !s_vm_recording && !s_distress_active) {
            if (gpio_ptt_down) {
                if (!s_last_gpio_ptt_down) s_gpio_down_ms = now;
                if (!s_ptt_active && (now - s_gpio_down_ms >= 30)) {
                    s_ptt_active = true;
                    xSemaphoreTake(s_app_mtx, portMAX_DELAY);
                    audio::set_ptt_active(true);
                    audio::set_talking(true);
                    xSemaphoreGive(s_app_mtx);
                    broadcast_state();
                    audio::tone(880, 25, 350); // PTT talk start chirp
                    ESP_LOGI(TAG, "PTT: TX active (GPIO 4 held)");
                }
            } else if (s_last_gpio_ptt_down && s_ptt_active) {
                s_ptt_active = false;
                s_last_ptt_ended_ms = now;
                xSemaphoreTake(s_app_mtx, portMAX_DELAY);
                audio::set_ptt_active(false);
                audio::set_talking(false);
                xSemaphoreGive(s_app_mtx);
                broadcast_state();
                audio::tone(587, 25, 300);
                audio::tone(0, 10);
                audio::tone(880, 35, 300);
                ESP_LOGI(TAG, "PTT: TX ended (GPIO 4 released - Roger beep)");
            }
        }
        s_last_gpio_ptt_down = gpio_ptt_down;

        // Handle pending click evaluation (for voicemail inbox 1x/2x click)
        if (s_talk_click_count > 0 && (now - s_last_talk_click_ms > kMultiClickWindowMs)) {
            if (s_vm_inbox_has_message) {
                if (s_talk_click_count == 1) {
                    play_inbox_voicemail();
                } else if (s_talk_click_count >= 2) {
                    dismiss_inbox_voicemail();
                }
            }
            s_talk_click_count = 0;
        }

        // Audio Voice Menu inactivity timeout (relaxed 60 seconds, paused while voice prompt is playing)
        if (s_menu_active) {
            if (audio::prompt_busy()) {
                s_last_menu_activity_ms = now;
            } else if (now - s_last_menu_activity_ms >= 60000) {
                exit_menu(true);
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
            s_walk_miss_count++;
            if (s_walk_miss_count < 3) {
                // Low descending dual error buzz (200Hz -> 140Hz)
                audio::tone(200, 35, 350);
                audio::tone(0, 15);
                audio::tone(140, 50, 350);
                printf("  probe #%-2u: [DROPPED] (miss %u/3)\n", s_walk_seq, s_walk_miss_count);
            } else if (s_walk_miss_count == 3) {
                s_walk_zone = WALK_ZONE_OUT;
                notify_disconnected();
                printf("  probe #%-2u: [OUT OF RANGE] Link lost!\n", s_walk_seq);
            } else {
                if (s_walk_miss_count % 2 == 0) {
                    audio::tone(140, 30, 200);
                }
                printf("  probe #%-2u: [OUT OF RANGE]\n", s_walk_seq);
            }
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

        // Auto-Join Channel Hunting on Boot:
        // If a newly powered-on node doesn't hear any mesh peer after 6s on its boot channel,
        // it hunts through priority channels [13, 6, 11, 1, 5, 8, 12, 10, 7, 9, 2, 3, 4]
        // so it seamlessly joins any running mesh network.
        if (!s_boot_hunting_done) {
            if (s_last_peer_rx_ms > 0) {
                if (s_boot_hunting_active) {
                    uint8_t cur = radio::channel();
                    printf("\n*** [AUTO-JOIN] Discovered and joined active mesh on Channel %u! ***\n\n", cur);
                    s_primary_channel = cur;
                    nvs_put_u8("ch", cur);
                    notify_connected();
                    broadcast_state();
                }
                s_boot_hunting_done = true;
                s_boot_hunting_active = false;
            } else if (now - s_boot_ms > 6000) {
                static const uint8_t kHuntChannels[] = {13, 6, 11, 1, 5, 8, 12, 10, 7, 9, 2, 3, 4};
                static constexpr size_t kNumHuntChannels = sizeof(kHuntChannels) / sizeof(kHuntChannels[0]);
                if (!s_boot_hunting_active) {
                    s_boot_hunting_active = true;
                    s_hunt_ch_index = 0;
                    s_last_hunt_step_ms = now;
                    printf("\n*** [AUTO-JOIN] No mesh peers on boot channel %u. Hunting channels to auto-join mesh... ***\n", radio::channel());
                } else if (now - s_last_hunt_step_ms >= 180) {
                    s_last_hunt_step_ms = now;
                    s_hunt_ch_index = (s_hunt_ch_index + 1) % kNumHuntChannels;
                    radio::set_channel(kHuntChannels[s_hunt_ch_index]);
                }
            }
        }

        // Auto-Recovery Watchdog: if mesh link lost on primary channel for > 15s, switch to backup channel
        static uint32_t last_watchdog_check = 0;
        uint32_t cur_now = now_ms();
        if (cur_now - last_watchdog_check >= 1000) {
            last_watchdog_check = cur_now;
            if (s_backup_channel >= 1 && s_backup_channel <= 13 &&
                s_primary_channel >= 1 && s_primary_channel <= 13 &&
                s_backup_channel != s_primary_channel) {
                uint32_t rx_ms = s_last_peer_rx_ms;
                uint32_t time_since_peer = (rx_ms > 0 && cur_now >= rx_ms) ? (cur_now - rx_ms) : 0;
                if (rx_ms > 0 && cur_now >= rx_ms && time_since_peer > 15000) {
                    if (!s_fallback_active) {
                        s_fallback_active = true;
                        s_fallback_switch_ms = cur_now;
                        radio::set_channel(s_backup_channel);
                        printf("\n*** [WATCHDOG] Link lost for %lu s on Primary Ch %u! Falling back to Backup Ch %u... ***\n\n",
                               (unsigned long)(time_since_peer / 1000), s_primary_channel, s_backup_channel);
                        audio::tone(400, 35, 300);
                        audio::tone(0, 15);
                        audio::tone(300, 45, 300);
                    } else if (cur_now - s_fallback_switch_ms > 15000) {
                        // Alternate between primary and backup every 15s to re-establish
                        s_fallback_switch_ms = cur_now;
                        uint8_t next_ch = (radio::channel() == s_primary_channel) ? s_backup_channel : s_primary_channel;
                        radio::set_channel(next_ch);
                        printf("*** [WATCHDOG] Hunting for mesh peers on alternate Ch %u ***\n", next_ch);
                    }
                } else if (s_fallback_active && time_since_peer < 3000) {
                    // Peer heard! Link restored!
                    s_fallback_active = false;
                    s_recovered_on_backup_ms = cur_now;
                    uint8_t cur = radio::channel();
                    printf("\n*** [WATCHDOG] Reconnected with mesh peer on Backup Channel %u! ***\n\n", cur);
                    notify_connected();
                }
            }
        }

        // Auto Re-Consensus on Recovery: 10s after recovering on backup channel, coordinator triggers quiet re-consensus
        if (s_recovered_on_backup_ms > 0 && (cur_now - s_recovered_on_backup_ms >= 10000)) {
            s_recovered_on_backup_ms = 0;
            bool is_coordinator = true;
            radio::peers_lock();
            for (const wt::Peer* p = radio::peers().begin(); p != radio::peers().end(); ++p) {
                if (p->used && p->id < radio::self_id()) {
                    is_coordinator = false;
                    break;
                }
            }
            radio::peers_unlock();
            if (is_coordinator) {
                printf("\n*** [AUTO RE-CONSENSUS] Mesh link stable on backup channel. Calculating fresh optimal channels... ***\n");
                collaborative_channel_scan(true /* quiet */);
            }
        }

        // Snowboard Dynamic Auto-Consensus:
        // Periodically (every 90s) while active, or proactively if link quality degrades (no peer for > 7s),
        // the coordinator quietly evaluates mesh RF spectrum and switches channel ONLY if the current
        // channel becomes congested (e.g. riding into a crowded lodge or ski lift area).
        static uint32_t s_last_snowboard_consensus_ms = 0;
        uint32_t rx_peer_ms = s_last_peer_rx_ms;
        uint32_t peer_elapsed = (rx_peer_ms > 0 && cur_now >= rx_peer_ms) ? (cur_now - rx_peer_ms) : 0;
        bool link_degrading = (rx_peer_ms > 0 && cur_now >= rx_peer_ms && peer_elapsed >= 7000 && !s_fallback_active);
        bool periodic_due = (cur_now - s_last_snowboard_consensus_ms >= 90000);

        if ((periodic_due || link_degrading) &&
            !audio::talking() && !audio::receiving() && !audio::prompt_busy() &&
            (now - talk_since > 8000) && !s_vm_recording && !s_menu_active &&
            !s_distress_active && (s_distress_peer == 0) &&
            !s_boot_hunting_active && (s_recovered_on_backup_ms == 0)) {

            bool is_coordinator = true;
            int peer_count = 0;
            radio::peers_lock();
            for (const wt::Peer* p = radio::peers().begin(); p != radio::peers().end(); ++p) {
                if (p->used) {
                    peer_count++;
                    if (p->id < radio::self_id()) {
                        is_coordinator = false;
                        break;
                    }
                }
            }
            radio::peers_unlock();

            if (is_coordinator && peer_count > 0) {
                s_last_snowboard_consensus_ms = now;
                if (link_degrading) {
                    printf("\n*** [SNOWBOARD AUTO-CONSENSUS] RF link degrading (no peer for %lu s). Proactively checking spectrum... ***\n",
                           (unsigned long)(peer_elapsed / 1000));
                } else {
                    printf("\n*** [SNOWBOARD AUTO-CONSENSUS] Periodic background RF evaluation (Snowboard mode)... ***\n");
                }
                collaborative_channel_scan(true /* quiet */);
            } else if (periodic_due) {
                s_last_snowboard_consensus_ms = now;
            }
        }

        update_led();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
