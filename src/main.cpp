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
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hid_buttons.hpp"
#include "led.hpp"
#include "nvs.h"
#include "nvs_flash.h"
#include "radio.hpp"
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

// Same chimes for every input (BOOT, external button, headset button, console).
void chime_join() { audio::tone(800, 50); audio::tone(0, 20); audio::tone(1200, 70); }
void chime_leave() { audio::tone(1100, 50); audio::tone(0, 20); audio::tone(600, 70); }
void chime_peer_joined() { audio::tone(1000, 40); }
void chime_peer_left() { audio::tone(500, 40); }

void update_led() {
    using led::Color;
    if (audio::loopback()) led::set(Color::White);
    else if (audio::sending()) led::set(Color::Red);
    else if (audio::receiving()) led::set(Color::Green);
    else if (audio::talking()) led::set(Color::Amber);  // mic open, waiting for speech
    else if (!usb_headset::spk_ready()) led::set(Color::Blue);
    else led::set(Color::Off);
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
    audio::tone(1800, 30);
    printf("ping sent\n");
}

void volume_step(int d) {
    int v = audio::volume() + d;
    bool limit = v < 0 || v > 100;
    v = v < 0 ? 0 : (v > 100 ? 100 : v);
    audio::set_volume(static_cast<uint8_t>(v));
    nvs_put_u8("vol", static_cast<uint8_t>(v));
    audio::tone(limit ? (d > 0 ? 2200 : 400) : (d > 0 ? 1500 : 750), 35);
}

void handle_press(wt::Press p, int which) {
    // which: 0 = talk button (BOOT, external, headset), 1 = vol up, 2 = vol down
    if (p == wt::Press::None) return;
    if (which == 0) p == wt::Press::Click ? toggle_talk() : send_ping();
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
    printf("  radio: tx %lu rx %lu relayed %lu dup %lu rx-drop %lu auth-fail %lu tx-fail %lu\n",
           (unsigned long)r.tx, (unsigned long)r.rx, (unsigned long)r.relayed, (unsigned long)r.dup,
           (unsigned long)r.rx_drop, (unsigned long)r.auth_fail, (unsigned long)r.tx_fail);
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
    printf("\n  t        talk on/off (same as BOOT or headset button click)\n"
           "  + / -    volume\n"
           "  p        ping all nodes (same as long press)\n"
           "  s        status, peers, counters\n"
           "  ch N     radio channel 1..13 (all nodes must match)\n"
           "  name X   set this node's name\n"
           "  loop     loopback test: hear your own voice through the codec\n"
           "  selftest check headset streams and the audio path\n"
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

void on_hid_buttons(uint8_t down) { s_hid_down = down; }

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
    usb_headset::start(on_hid_buttons, on_headset_state);
    xTaskCreatePinnedToCore(console_task, "console", 8192, nullptr, 3, nullptr, 0);
    printf("\n%s ready on channel %u. Type 'h' for commands.\n", s_name, ch);

    wt::ButtonDetector talk, up, down;
    uint8_t last_toggle = 0;
    uint32_t last_beacon = 0, talk_since = 0;
    for (;;) {
        uint32_t now = now_ms();
        uint8_t hid = s_hid_down;
        uint8_t tog = hid & wt::HID_BTN_TALK_TOGGLE;
        if (tog != last_toggle) {  // latching hook/mute control: every change is one click
            last_toggle = tog;
            handle_press(wt::Press::Click, 0);
        }
        handle_press(talk.update(low(cfg::kPinBoot) || low(cfg::kPinTalk) ||
                                     (hid & wt::HID_BTN_TALK), now), 0);
        handle_press(up.update(low(cfg::kPinVolUp) || (hid & wt::HID_BTN_VOL_UP), now), 1);
        handle_press(down.update(low(cfg::kPinVolDown) || (hid & wt::HID_BTN_VOL_DOWN), now), 2);

        // Auto-mute a mic that has been left open too long.
        if (audio::talking()) {
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
        update_led();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
