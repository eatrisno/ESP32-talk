#include "voice_gate.hpp"

#include <cmath>
#include <cstring>

#include "adpcm.hpp"
#include "dsp.hpp"
#include "jitter.hpp"

namespace wt {

bool VoiceGate::update(const int16_t* x, size_t n) {
    double acc = 0;
    for (size_t i = 0; i < n; i++) acc += static_cast<double>(x[i]) * x[i];
    rms_ = n ? sqrtf(static_cast<float>(acc / n)) : 0.f;
    if (frames_ < 25) {  // first second: take the quietest frame as the floor, send nothing
        floor_ = frames_ == 0 || rms_ < floor_ ? rms_ : floor_;
        if (floor_ < 30.f) floor_ = 30.f;
        frames_++;
        speech_now_ = false;
        return false;
    }
    // Noise floor: falls fast. It rises (about 3 dB/s) only while the gate is closed, so
    // sustained speech or singing cannot lift the floor up to itself and close the gate;
    // a steady fan or hiss still gets learned within a few seconds of the gate closing.
    if (rms_ < floor_) floor_ += (rms_ - floor_) * 0.3f;
    else if (hang_ == 0) floor_ *= 1.0139f;
    else if (open_frames_ > kMaxOpenFrames) floor_ *= 1.0139f;  // stuck open on noise
    if (floor_ < 30.f) floor_ = 30.f;
    const float kAbsMin = 150.f;              // about -47 dBFS
    float thr = floor_ * 2.5f;                // 8 dB above the floor
    if (thr < kAbsMin) thr = kAbsMin;
    speech_now_ = rms_ > thr;
    if (speech_now_) hang_ = kHangFrames;
    else if (hang_ > 0) hang_--;
    open_frames_ = hang_ > 0 ? open_frames_ + 1 : 0;
    return hang_ > 0;
}

float pipeline_selftest(uint32_t mic_rate, uint32_t spk_rate) {
    // Streams one second of a 1 kHz tone through the path and fits the output on the fly,
    // so it needs only a few KB (it also runs on the device).
    static int16_t mic[960], v8[400], pending[kFrameSamples], frame[kFrameSamples];
    static int16_t up[kFrameSamples * 13];
    const double kHz = 1000.0, kAmp = 12000.0;
    // Static, not on the stack: the jitter buffer alone is 11 KB and the console task that
    // runs this on the device has 8 KB. Not reentrant (one self-test at a time).
    static Resampler down, upr;
    static JitterBuffer jb;
    down.init(mic_rate, kVoiceRate);
    upr.init(kVoiceRate, spk_rate);
    jb.reset();
    AdpcmState enc;
    size_t acc = 0, out_i = 0;
    uint16_t fno = 0;
    uint64_t t = 0;
    const size_t skip = spk_rate / 5;  // 200 ms for the filters to settle
    double ss = 0, cc = 0, sc = 0, xs = 0, xc = 0, xx = 0;
    // 37-sample blocks on purpose: they never divide the rates evenly, like real USB
    // packets (44.1 kHz arrives as 44/45 samples per ms). Even 10 ms blocks would hide a
    // resampler that loses its phase between blocks.
    const size_t block = 37;
    for (uint64_t total = 0; total < mic_rate; total += block) {
        for (size_t i = 0; i < block; i++, t++)
            mic[i] = static_cast<int16_t>(kAmp * sin(2 * M_PI * kHz * static_cast<double>(t % mic_rate) / mic_rate));
        size_t m = down.process(mic, block, v8, 400);
        for (size_t i = 0; i < m; i++) {
            pending[acc++] = v8[i];
            if (acc < kFrameSamples) continue;
            acc = 0;
            AudioPayload a;
            uint8_t bytes[kAdpcmBytes];
            a.voice_id = 1;
            a.frame_no = fno++;
            a.adpcm_pred = enc.predictor;
            a.adpcm_index = static_cast<uint8_t>(enc.index);
            a.data_len = adpcm_encode(pending, kFrameSamples, bytes, enc);
            a.data = bytes;
            jb.put(a);
            if (!jb.pop(frame)) continue;
            size_t n = upr.process(frame, kFrameSamples, up, sizeof(up) / sizeof(up[0]));
            for (size_t k = 0; k < n; k++, out_i++) {
                if (out_i < skip) continue;
                double w = 2 * M_PI * kHz * static_cast<double>(out_i) / spk_rate;
                double s = sin(w), c = cos(w), x = up[k];
                ss += s * s; cc += c * c; sc += s * c;
                xs += x * s; xc += x * c; xx += x * x;
            }
        }
    }
    if (out_i <= skip + spk_rate / 10) return -1.f;
    double det = ss * cc - sc * sc;
    double A = (xs * cc - xc * sc) / det, B = (xc * ss - xs * sc) / det;
    double sig = A * A * ss + 2 * A * B * sc + B * B * cc;   // energy of the fitted tone
    double err = xx - 2 * (A * xs + B * xc) + sig;          // energy of everything else
    double amp = sqrt(A * A + B * B);
    if (amp < kAmp * 0.85 || amp > kAmp * 1.15) return -2.f;
    return static_cast<float>(10 * log10(sig / (err + 1e-9)));
}

}  // namespace wt
