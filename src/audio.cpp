#include "audio.hpp"

#include <cstdlib>
#include <cstring>

#include "adpcm.hpp"
#include "config.hpp"
#include "dsp.hpp"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "format.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "jitter.hpp"
#include "radio.hpp"
#include "usb_headset.hpp"
#include "voice_gate.hpp"

namespace audio {
namespace {

const char* TAG = "audio";

struct Talker {
    bool used = false;
    uint32_t sender = 0;
    uint32_t last_ms = 0;
    wt::JitterBuffer jb;
};

SemaphoreHandle_t s_mtx;           // guards talkers, tones, loopback buffer
Talker s_talkers[kMaxTalkers];
wt::ToneQueue s_tones;
wt::PromptQueue s_prompts;
wt::JitterBuffer s_loop;           // loopback path
volatile bool s_talking = false, s_ptt_active = false, s_sending = false, s_receiving = false, s_loopback = false;
volatile bool s_recording_vm = false;
VoicemailRecordCb s_vm_cb = nullptr;
volatile uint8_t s_volume = 70;
Stats s_stats;

uint32_t now_ms() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

// --- mic -> radio ----------------------------------------------------------------------

void mic_task(void*) {
    static uint8_t raw[4096];
    static int16_t mono[1024];
    static int16_t v8[512];
    static int16_t frame[wt::kFrameSamples];
    size_t fill = 0;
    wt::Resampler rs;
    wt::DcBlocker dc;
    wt::Agc agc;
    wt::VoiceGate gate;
    wt::StereoPick pick;
    int16_t peak_acc = 0;
    int peak_blocks = 0;
    wt::AdpcmState enc;
    uint32_t cur_rate = 0;
    uint8_t voice_id = static_cast<uint8_t>(esp_random());
    uint16_t frame_no = 0;
    bool was_sending = false;
    bool was_ptt_active = false;      // task-local: resets on task restart (B4)
    int ptt_lead_mute_frames = 0;     // task-local: resets on task restart (B4)
    for (;;) {
        if (!usb_headset::mic_ready()) {
            cur_rate = 0;
            if (was_sending) {  // headset gone mid-sentence
                was_sending = false;
                s_sending = false;
                voice_id++;
                enc = wt::AdpcmState();
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        wt::StreamFormat f = usb_headset::mic_format();
        if (f.rate != cur_rate) {  // new headset (or first one): fresh state for its stream
            rs.init(f.rate, wt::kVoiceRate);
            cur_rate = f.rate;
            fill = 0;
            pick = wt::StereoPick();
            gate = wt::VoiceGate();
            dc = wt::DcBlocker();
        }
        // About 10 ms per read, whole frames only.
        size_t want = f.frame_bytes() * (f.rate / 100);
        if (want > sizeof raw) want = sizeof raw - sizeof raw % f.frame_bytes();
        size_t got = usb_headset::mic_read(raw, want, 50);
        if (!got) {
            vTaskDelay(pdMS_TO_TICKS(5));  // read can fail at once (stream stopping): never spin
            continue;
        }
        s_stats.mic_bytes += got;
        size_t n = wt::pcm_to_mono(raw, got, f, mono, 1024, &pick);
        for (size_t i = 0; i < n; i++) {
            // Use int32_t abs to avoid UB: negating INT16_MIN as int16_t overflows. (B3)
            int16_t a = static_cast<int16_t>(std::abs(static_cast<int32_t>(mono[i])));
            if (a > peak_acc) peak_acc = a;
        }
        if (++peak_blocks >= 100) {  // ~1 s
            s_stats.mic_peak = peak_acc;
            peak_acc = 0;
            peak_blocks = 0;
        }
        size_t m = rs.process(mono, n, v8, 512);
        bool ptt = s_ptt_active;
        bool active = s_talking || ptt || s_loopback || s_recording_vm;
        // Task-local (not static): resets correctly if the task were ever restarted. (B4)
        if (ptt && !was_ptt_active) {
            ptt_lead_mute_frames = 1;  // Discard first 1 frame (~40ms) to eliminate mechanical button click sound without extra lag
        }
        was_ptt_active = ptt;
        for (size_t i = 0; i < m; i++) {
            frame[fill++] = v8[i];
            if (fill < wt::kFrameSamples) continue;
            fill = 0;
            for (auto& s : frame) s = wt::clip16(dc.run(s));
            // Gate first, on the frame before gain, so a gain change never opens or closes it.
            bool voice = gate.update(frame, wt::kFrameSamples);  // learns the noise floor even when muted
            agc.process(frame, wt::kFrameSamples, gate.speech_now());
            if (!active) continue;
            if (ptt && ptt_lead_mute_frames > 0) {
                ptt_lead_mute_frames--;
                continue;
            }
            if (s_recording_vm) {
                wt::AudioPayload a;
                uint8_t bytes[wt::kAdpcmBytes];
                a.voice_id = voice_id;
                a.frame_no = frame_no++;
                a.adpcm_pred = enc.predictor;
                a.adpcm_index = static_cast<uint8_t>(enc.index);
                a.data_len = wt::adpcm_encode(frame, wt::kFrameSamples, bytes, enc);
                a.data = bytes;
                if (s_vm_cb) s_vm_cb(a);
                continue;
            }
            bool should_gate = !cfg::kContinuousStream;
            if (should_gate && (!voice || s_prompts.busy()) && !s_loopback) {
                s_stats.frames_gated++;
                if (was_sending) {
                    was_sending = false;
                    s_sending = false;
                    voice_id++;              // next burst starts a new over at the receiver
                    enc = wt::AdpcmState();
                }
                continue;
            }
            was_sending = true;
            s_sending = true;
            wt::AudioPayload a;
            uint8_t bytes[wt::kAdpcmBytes];
            a.voice_id = voice_id;
            a.frame_no = frame_no++;
            a.adpcm_pred = enc.predictor;
            a.adpcm_index = static_cast<uint8_t>(enc.index);
            a.data_len = wt::adpcm_encode(frame, wt::kFrameSamples, bytes, enc);
            a.data = bytes;
            if (s_loopback) {
                xSemaphoreTake(s_mtx, portMAX_DELAY);
                s_loop.put(a);
                xSemaphoreGive(s_mtx);
                continue;
            }
            uint8_t pkt[wt::kMaxPlain];
            size_t len = wt::write_audio(a, pkt, sizeof pkt);
            if (radio::send(wt::MSG_AUDIO, pkt, len)) s_stats.frames_sent++;
        }
        if (!active && was_sending) {
            was_sending = false;
            s_sending = false;
            voice_id++;
            enc = wt::AdpcmState();
        }
        s_stats.agc_gain = agc.gain();
        s_stats.noise_floor = gate.noise_floor();
    }
}

// --- radio -> speaker ------------------------------------------------------------------

void spk_task(void*) {
    static float bus[wt::kFrameSamples];
    static int16_t mix[wt::kFrameSamples], one[wt::kFrameSamples];
    static int16_t up[wt::kFrameSamples * 13];
    static uint8_t raw[wt::kFrameSamples * 13 * 8];
    wt::Resampler rs;
    uint32_t cur_rate = 0;
    for (;;) {
        if (!usb_headset::spk_ready()) {
            cur_rate = 0;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        wt::StreamFormat f = usb_headset::spk_format();
        if (f.rate != cur_rate) {
            rs.init(wt::kVoiceRate, f.rate);
            cur_rate = f.rate;
        }
        for (auto& b : bus) b = 0.f;
        bool any = false;
        int talkers = 0;
        uint32_t now = now_ms();
        xSemaphoreTake(s_mtx, portMAX_DELAY);
        for (auto& t : s_talkers) {
            if (!t.used) continue;
            if (t.jb.pop(one)) {
                wt::mix_into(bus, one, wt::kFrameSamples);
                any = true;
                talkers++;
            } else if (!t.jb.active() && now - t.last_ms > 2000) {
                s_stats.lost += t.jb.lost();
                s_stats.late += t.jb.late();
                t.used = false;  // not `t = Talker()`: that builds an 11 KB temporary on this stack
            }
        }
        if (s_loopback && s_loop.pop(one)) {
            wt::mix_into(bus, one, wt::kFrameSamples);
            any = true;
        }
        // Sum in float, then volume, then one soft limiter: several loud talkers compress
        // smoothly instead of clipping.
        float g = wt::volume_gain(s_volume);
        for (size_t i = 0; i < wt::kFrameSamples; i++) mix[i] = wt::clip16(wt::soft_limit(bus[i] * g));
        if (any) s_stats.frames_played++;
        float tone_vol = 0.3f + 0.7f * g;  // scales with volume, but stays softly audible
        s_tones.render(mix, wt::kFrameSamples, tone_vol);
        s_prompts.render(mix, wt::kFrameSamples, tone_vol);
        xSemaphoreGive(s_mtx);
        s_receiving = talkers > 0;
        s_stats.talkers = talkers;
        // Always feed the speaker (silence when idle): keeps the USB stream clock steady
        // and avoids the pop some headsets make when the stream restarts.
        size_t n = rs.process(mix, wt::kFrameSamples, up, sizeof up / 2);
        size_t bytes = wt::mono_to_pcm(up, n, f, raw, sizeof raw);
        // Blocks until the 100 ms speaker buffer has room, so the USB frame clock (1 kHz SOF
        // from the ESP32) paces playback and latency stays bounded. ponytail: no feedback
        // endpoint support in the driver, so an asynchronous headset whose DAC clock differs
        // from USB timing will slip a sample now and then (a faint click every few minutes).
        if (!usb_headset::spk_write(raw, bytes, 80)) {
            s_stats.spk_underrun++;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

}  // namespace

void start() {
    s_mtx = xSemaphoreCreateMutex();
    // Core 1: audio only. Radio and USB host run on core 0.
    xTaskCreatePinnedToCore(mic_task, "mic", 6144, nullptr, 8, nullptr, 1);
    xTaskCreatePinnedToCore(spk_task, "spk", 6144, nullptr, 9, nullptr, 1);
}

void set_talking(bool on) { s_talking = on; }
bool talking() { return s_talking; }
void set_ptt_active(bool on) { s_ptt_active = on; }
bool ptt_active() { return s_ptt_active; }
bool sending() { return s_sending; }
bool receiving() { return s_receiving; }

void set_loopback(bool on) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_loop.reset();
    s_loopback = on;
    xSemaphoreGive(s_mtx);
}
bool loopback() { return s_loopback; }

void set_volume(uint8_t p) { s_volume = p > 100 ? 100 : p; }
uint8_t volume() { return s_volume; }

void set_recording_vm(bool on) { s_recording_vm = on; }
bool recording_vm() { return s_recording_vm; }
void set_voicemail_cb(VoicemailRecordCb cb) { s_vm_cb = cb; }

void tone(uint16_t freq, uint16_t ms, uint16_t amp) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_tones.push(freq, ms, amp);
    xSemaphoreGive(s_mtx);
}

void prompt(wt::PromptId id) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_prompts.push(id);
    xSemaphoreGive(s_mtx);
}

void prompt_clear() {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_prompts.clear();
    xSemaphoreGive(s_mtx);
}

bool prompt_busy() {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    bool b = s_prompts.busy();
    xSemaphoreGive(s_mtx);
    return b;
}

void on_audio(uint32_t sender, const wt::AudioPayload& a) {
    uint32_t now = now_ms();
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    Talker* t = nullptr;
    for (auto& x : s_talkers)
        if (x.used && x.sender == sender) t = &x;
    if (!t)
        for (auto& x : s_talkers)
            if (!x.used) { t = &x; break; }
    if (t) {
        if (!t->used) {
            t->used = true;
            t->sender = sender;
            t->jb.reset();
        }
        t->last_ms = now;
        t->jb.put(a);
    }
    // ponytail: a 5th simultaneous talker is dropped. Add priority by RSSI if groups grow.
    xSemaphoreGive(s_mtx);
}

void clear_talker(uint32_t sender) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    for (auto& x : s_talkers) {
        if (x.used && x.sender == sender) {
            x.jb.reset();
            x.used = false;
        }
    }
    xSemaphoreGive(s_mtx);
}

Stats stats() { return s_stats; }

}  // namespace audio
