#include "usb_headset.hpp"

#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hid_buttons.hpp"
#include "usb/hid_host.h"
#include "usb/uac_host.h"
#include "usb/usb_host.h"

namespace usb_headset {
namespace {

const char* TAG = "usb";

struct Stream {
    uac_host_device_handle_t h = nullptr;
    wt::StreamFormat fmt;
    volatile bool ready = false;
    volatile bool error = false;   // transfer error: worker restarts the stream
    uint16_t vid = 0, pid = 0;
};
Stream s_mic, s_spk;

struct HidIf {
    hid_host_device_handle_t h = nullptr;
    wt::HidButtons parser;
    uint8_t down = 0;          // merged over all report ids of this interface
};
constexpr int kMaxHid = 3;
HidIf s_hid[kMaxHid];

ButtonCb s_on_buttons;
StateCb s_on_state;

enum class Ev : uint8_t { UacConnected, UacDisconnected, UacError, HidConnected };
struct Msg {
    Ev ev;
    uint8_t addr, iface;
    bool tx;  // speaker
    void* handle;
};
QueueHandle_t s_q;

void usb_lib_task(void*) {
    for (;;) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}

// --- UAC -------------------------------------------------------------------------------

void post(const Msg& m) {
    // Driver-task context. The worker can be busy for a few seconds (control transfers), so
    // wait for room rather than drop: a lost disconnect would leave a dead handle forever.
    if (xQueueSend(s_q, &m, pdMS_TO_TICKS(2000)) != pdTRUE) ESP_LOGE(TAG, "event queue stuck, event lost");
}

void uac_dev_cb(uac_host_device_handle_t h, const uac_host_device_event_t ev, void*) {
    if (ev == UAC_HOST_DRIVER_EVENT_DISCONNECTED || ev == UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR) {
        // Stop the audio tasks using it right now, before the worker gets to the message.
        for (Stream* s : {&s_mic, &s_spk})
            if (s->h == h) s->ready = false;
        post(Msg{ev == UAC_HOST_DRIVER_EVENT_DISCONNECTED ? Ev::UacDisconnected : Ev::UacError, 0, 0, false, h});
    }
}

void uac_drv_cb(uint8_t addr, uint8_t iface, const uac_host_driver_event_t ev, void*) {
    post(Msg{Ev::UacConnected, addr, iface, ev == UAC_HOST_DRIVER_EVENT_TX_CONNECTED, nullptr});
}

uac_host_device_handle_t open_dev(const Msg& m, uint32_t buffer_size) {
    uac_host_device_config_t dc = {};
    dc.addr = m.addr;
    dc.iface_num = m.iface;
    dc.callback = uac_dev_cb;
    dc.buffer_size = buffer_size;
    dc.buffer_threshold = 0;  // driver default (size / 4)
    uac_host_device_handle_t h = nullptr;
    if (uac_host_device_open(&dc, &h) != ESP_OK) return nullptr;
    return h;
}

void open_uac(const Msg& m) {
    Stream& s = m.tx ? s_spk : s_mic;
    if (s.h) return;
    // The ring buffer size is fixed at open, but the format is only known after open.
    // Open small to read the formats, then reopen with a buffer for the chosen one.
    uac_host_device_handle_t h = open_dev(m, 4096);
    if (!h) {
        ESP_LOGW(TAG, "%s open failed", m.tx ? "speaker" : "mic");
        return;
    }
    uac_host_dev_info_t info;
    uac_host_get_device_info(h, &info);
    wt::AltFormat alts[8];
    size_t n = 0;
    for (uint8_t alt = 1; alt <= info.iface_alt_num && n < 8; alt++) {
        uac_host_dev_alt_param_t p;
        if (uac_host_get_device_alt_param(h, alt, &p) != ESP_OK) continue;
        if (p.format != UAC_TYPE_I_PCM) continue;  // only plain PCM (wFormatTag 1)
        wt::AltFormat& a = alts[n++];
        a.channels = p.channels;
        a.subframe_bytes = p.subframe_size;
        a.bits = p.bit_resolution;
        a.freq_count = p.sample_freq_type;
        if (p.sample_freq_type == 0) {
            a.freq_min = p.sample_freq_lower;
            a.freq_max = p.sample_freq_upper;
        } else {
            for (int i = 0; i < p.sample_freq_type && i < UAC_FREQ_NUM_MAX && i < 8; i++)
                a.freqs[i] = p.sample_freq[i];
        }
        ESP_LOGI(TAG, "%s alt %u: %u ch, %u-bit (%u B), %s %lu..%lu Hz", m.tx ? "spk" : "mic", alt,
                 p.channels, p.bit_resolution, p.subframe_size,
                 p.sample_freq_type ? "list" : "range",
                 (unsigned long)(p.sample_freq_type ? p.sample_freq[0] : p.sample_freq_lower),
                 (unsigned long)(p.sample_freq_type ? p.sample_freq[p.sample_freq_type - 1]
                                                    : p.sample_freq_upper));
    }
    wt::StreamFormat f = wt::choose_format(alts, n);
    uac_host_device_close(h);
    if (!f.valid()) {
        ESP_LOGE(TAG, "%s: no usable format (need PCM, 1-2 ch, 16/24/32-bit)", m.tx ? "speaker" : "mic");
        return;
    }
    // The driver starts the LAST alt with matching channels and bit resolution and ignores the
    // subframe size. If another PCM alt has the same channels+bits but a different subframe
    // size (e.g. 24-bit in 3 and in 4 bytes), we cannot tell which one it would stream: refuse
    // rather than play noise.
    for (size_t i = 0; i < n; i++)
        if (alts[i].channels == f.channels && alts[i].bits == f.bits && alts[i].subframe_bytes != f.subframe_bytes) {
            ESP_LOGE(TAG, "%s: ambiguous %u-bit formats (%u and %u byte), not supported", m.tx ? "speaker" : "mic",
                     f.bits, f.subframe_bytes, alts[i].subframe_bytes);
            return;
        }
    // Speaker: 100 ms. It is also the playback clock (the speaker task blocks on it), so it
    // bounds the output latency. Mic: 100 ms, read every 10 ms.
    uint32_t bytes_per_ms = f.rate / 1000 * static_cast<uint32_t>(f.frame_bytes()) + f.frame_bytes();
    h = open_dev(m, bytes_per_ms * 100);
    if (!h) {
        ESP_LOGE(TAG, "%s reopen failed (%lu B buffer)", m.tx ? "speaker" : "mic",
                 (unsigned long)(bytes_per_ms * 100));
        return;
    }
    uac_host_stream_config_t sc = {};
    sc.channels = f.channels;
    sc.bit_resolution = f.bits;
    sc.sample_freq = f.rate;
    if (uac_host_device_start(h, &sc) != ESP_OK) {
        ESP_LOGE(TAG, "%s start failed at %lu Hz", m.tx ? "speaker" : "mic", (unsigned long)f.rate);
        uac_host_device_close(h);
        return;
    }
    // Hardware volume high and unmuted; level is set in software so every headset
    // behaves the same. Not all devices have these controls, errors are fine.
    uac_host_device_set_mute(h, false);
    uac_host_device_set_volume(h, m.tx ? 90 : 80);
    s.h = h;
    s.fmt = f;
    s.vid = info.VID;
    s.pid = info.PID;
    s.ready = true;
    ESP_LOGI(TAG, "%s %04x:%04x started: %lu Hz, %u ch, %u-bit", m.tx ? "speaker" : "mic", info.VID,
             info.PID, (unsigned long)f.rate, f.channels, f.bits);
    if (m.tx && s_on_state) s_on_state(true);  // once per headset, when it can play the chime
}

void restart_uac(uac_host_device_handle_t h) {
    // A failed isochronous transfer is not resubmitted by the driver: the stream would stay
    // silent until replug. Stop and start it again with the same format.
    for (Stream* s : {&s_mic, &s_spk}) {
        if (s->h != h) continue;
        vTaskDelay(pdMS_TO_TICKS(100));  // let the audio task leave the driver
        uac_host_device_stop(h);
        uac_host_stream_config_t sc = {};
        sc.channels = s->fmt.channels;
        sc.bit_resolution = s->fmt.bits;
        sc.sample_freq = s->fmt.rate;
        if (uac_host_device_start(h, &sc) == ESP_OK) {
            s->ready = true;
            ESP_LOGW(TAG, "%s transfer error, stream restarted", s == &s_spk ? "speaker" : "mic");
        } else {
            ESP_LOGE(TAG, "%s transfer error, restart failed: replug the headset", s == &s_spk ? "speaker" : "mic");
        }
    }
}

void close_uac(uac_host_device_handle_t h) {
    for (Stream* s : {&s_mic, &s_spk}) {
        if (s->h != h) continue;
        s->ready = false;
        // Let a read/write already inside the driver return (their timeouts are <= 80 ms).
        vTaskDelay(pdMS_TO_TICKS(100));
        uac_host_device_close(h);
        s->h = nullptr;
        ESP_LOGI(TAG, "%s disconnected", s == &s_spk ? "speaker" : "mic");
    }
    if (!s_mic.h && !s_spk.h && s_on_state) s_on_state(false);
}

// --- HID -------------------------------------------------------------------------------

void report_buttons() {
    uint8_t all = 0;
    for (auto& x : s_hid)
        if (x.h) all |= x.down;
    static uint8_t last = 0;
    if (all != last) {
        last = all;
        if (s_on_buttons) s_on_buttons(all);
    }
}

void hid_if_cb(hid_host_device_handle_t h, const hid_host_interface_event_t ev, void*) {
    HidIf* d = nullptr;
    for (auto& x : s_hid)
        if (x.h == h) d = &x;
    if (!d) {
        // An interface we opened only to read its descriptor and then closed: the driver
        // asks for the second close here.
        if (ev == HID_HOST_INTERFACE_EVENT_DISCONNECTED) hid_host_device_close(h);
        return;
    }
    if (ev == HID_HOST_INTERFACE_EVENT_INPUT_REPORT) {
        uint8_t r[64];
        size_t len = 0;
        if (hid_host_device_get_raw_input_report_data(h, r, sizeof r, &len) != ESP_OK) return;
        uint8_t mask = 0;
        uint8_t now = d->parser.decode(r, len, &mask);
        // A report only says something about the buttons it carries: keep the others.
        d->down = static_cast<uint8_t>((d->down & ~mask) | now);
        report_buttons();
    } else if (ev == HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR) {
        // Not resubmitted by the driver: restart polling, release anything held.
        d->down = 0;
        report_buttons();
        hid_host_device_stop(h);
        hid_host_device_start(h);
    } else if (ev == HID_HOST_INTERFACE_EVENT_DISCONNECTED) {
        hid_host_device_close(h);
        d->h = nullptr;
        d->down = 0;
        report_buttons();
    }
}

void hid_drv_cb(hid_host_device_handle_t h, const hid_host_driver_event_t ev, void*) {
    if (ev != HID_HOST_DRIVER_EVENT_CONNECTED) return;
    Msg m{Ev::HidConnected, 0, 0, false, h};
    xQueueSend(s_q, &m, 0);
}

void open_hid(hid_host_device_handle_t h) {
    HidIf* d = nullptr;
    for (auto& x : s_hid)
        if (!x.h) { d = &x; break; }
    if (!d) return;
    hid_host_device_config_t dc = {};
    dc.callback = hid_if_cb;
    if (hid_host_device_open(h, &dc) != ESP_OK) return;
    size_t dlen = 0;
    const uint8_t* desc = hid_host_get_report_descriptor(h, &dlen);
    if (!desc || !d->parser.parse(desc, dlen)) {
        // Keyboard/mouse/vendor interface with no headset buttons: not ours.
        hid_host_device_close(h);
        return;
    }
    d->h = h;
    if (hid_host_device_start(h) != ESP_OK) {
        hid_host_device_close(h);
        *d = HidIf();
        return;
    }
    ESP_LOGI(TAG, "headset buttons found (%d fields)", d->parser.field_count());
}

void worker(void*) {
    Msg m;
    for (;;) {
        if (xQueueReceive(s_q, &m, portMAX_DELAY) != pdTRUE) continue;
        switch (m.ev) {
            case Ev::UacConnected: open_uac(m); break;
            case Ev::UacDisconnected: close_uac(static_cast<uac_host_device_handle_t>(m.handle)); break;
            case Ev::UacError: restart_uac(static_cast<uac_host_device_handle_t>(m.handle)); break;
            case Ev::HidConnected: open_hid(static_cast<hid_host_device_handle_t>(m.handle)); break;
        }
    }
}

}  // namespace

void start(ButtonCb on_buttons, StateCb on_state) {
    s_on_buttons = on_buttons;
    s_on_state = on_state;
    s_q = xQueueCreate(16, sizeof(Msg));
    usb_host_config_t hc = {};
    hc.skip_phy_setup = false;
    hc.intr_flags = ESP_INTR_FLAG_LOWMED;
    ESP_ERROR_CHECK(usb_host_install(&hc));
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, nullptr, 10, nullptr, 0);

    uac_host_driver_config_t uc = {};
    uc.create_background_task = true;
    uc.task_priority = 9;
    uc.stack_size = 4096;
    uc.core_id = 0;
    uc.callback = uac_drv_cb;
    ESP_ERROR_CHECK(uac_host_install(&uc));

    hid_host_driver_config_t hcfg = {};
    hcfg.create_background_task = true;
    hcfg.task_priority = 5;
    hcfg.stack_size = 4096;
    hcfg.core_id = 0;
    hcfg.callback = hid_drv_cb;
    ESP_ERROR_CHECK(hid_host_install(&hcfg));

    xTaskCreatePinnedToCore(worker, "usb_ev", 4096, nullptr, 6, nullptr, 0);
}

bool mic_ready() { return s_mic.ready; }
bool spk_ready() { return s_spk.ready; }
wt::StreamFormat mic_format() { return s_mic.fmt; }
wt::StreamFormat spk_format() { return s_spk.fmt; }
bool has_buttons() {
    for (auto& x : s_hid)
        if (x.h) return true;
    return false;
}

size_t mic_read(uint8_t* buf, size_t max, uint32_t timeout_ms) {
    if (!s_mic.ready) return 0;
    uint32_t got = 0;
    if (uac_host_device_read(s_mic.h, buf, max, &got, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) return 0;
    return got;
}

bool spk_write(const uint8_t* buf, size_t len, uint32_t timeout_ms) {
    if (!s_spk.ready) return false;
    return uac_host_device_write(s_spk.h, const_cast<uint8_t*>(buf), len, pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}

void print_info() {
    auto line = [](const char* name, const Stream& s) {
        if (s.ready)
            printf("  %-7s %04x:%04x  %lu Hz  %u ch  %u-bit\n", name, s.vid, s.pid,
                   (unsigned long)s.fmt.rate, s.fmt.channels, s.fmt.bits);
        else
            printf("  %-7s not connected\n", name);
    };
    line("mic", s_mic);
    line("speaker", s_spk);
    printf("  buttons %s\n", has_buttons() ? "yes (USB HID)" : "none found");
}

}  // namespace usb_headset
