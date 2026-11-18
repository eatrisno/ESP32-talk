#include "dsp.hpp"

#include <cmath>

namespace wt {

int16_t clip16(float x) {
    if (x > 32767.f) return 32767;
    if (x < -32768.f) return -32768;
    return static_cast<int16_t>(lrintf(x));
}

void Biquad::lowpass(float fs, float fc, float q) {
    // RBJ audio EQ cookbook low-pass
    float w = 2.f * static_cast<float>(M_PI) * fc / fs;
    float alpha = sinf(w) / (2.f * q);
    float c = cosf(w);
    float a0 = 1.f + alpha;
    b0 = (1.f - c) / 2.f / a0;
    b1 = (1.f - c) / a0;
    b2 = b0;
    a1 = -2.f * c / a0;
    a2 = (1.f - alpha) / a0;
    z1 = z2 = 0;
}

void LowPass8::design(float fs, float fc) {
    static const float kQ[4] = {0.50980f, 0.60134f, 0.89998f, 2.56292f};  // Butterworth n=8
    for (int i = 0; i < 4; i++) s[i].lowpass(fs, fc, kQ[i]);
}

void Resampler::init(uint32_t in_rate, uint32_t out_rate) {
    in_ = in_rate ? in_rate : kVoiceRate;
    out_ = out_rate ? out_rate : kVoiceRate;
    pos_ = 0;
    prev_ = 0;
    filter_in_ = in_ > out_;
    filter_out_ = in_ < out_;
    // Cut-off at 42.5% of the lower rate: 3.4 kHz for the 8 kHz voice path.
    float lower = static_cast<float>(in_ < out_ ? in_ : out_);
    if (filter_in_) lp_.design(static_cast<float>(in_), 0.425f * lower);
    if (filter_out_) lp_.design(static_cast<float>(out_), 0.425f * lower);
}

size_t Resampler::process(const int16_t* in, size_t n, int16_t* out, size_t out_max) {
    if (in_ == out_) {
        size_t m = n < out_max ? n : out_max;
        for (size_t i = 0; i < m; i++) out[i] = in[i];
        return m;
    }
    // buf_[0] is the last sample of the previous block, buf_[1..m] the new ones.
    // pos_ / out_ is the integer input index, pos_ % out_ the fraction: exact, no drift.
    size_t o = 0;
    while (n > 0) {
        size_t m = n < kChunk ? n : kChunk;
        buf_[0] = prev_;
        for (size_t i = 0; i < m; i++) {
            float x = static_cast<float>(in[i]);
            buf_[i + 1] = filter_in_ ? lp_.run(x) : x;
        }
        while (pos_ / out_ + 1 <= m) {
            if (o < out_max) {
                size_t idx = static_cast<size_t>(pos_ / out_);
                float frac = static_cast<float>(pos_ % out_) / static_cast<float>(out_);
                float y = buf_[idx] + (buf_[idx + 1] - buf_[idx]) * frac;
                if (filter_out_) y = lp_.run(y);
                out[o++] = clip16(y);
            }
            pos_ += in_;
        }
        prev_ = buf_[m];
        pos_ -= static_cast<uint64_t>(m) * out_;
        in += m;
        n -= m;
    }
    return o;
}

float soft_limit(float y) {
    // Linear up to the knee (-3 dBFS), then a tanh curve that never reaches full scale.
    const float kKnee = 23000.f, kRoom = 32000.f - kKnee;
    float a = fabsf(y);
    if (a <= kKnee) return y;
    float l = kKnee + kRoom * tanhf((a - kKnee) / kRoom);
    return y < 0 ? -l : l;
}

void Agc::process(int16_t* x, size_t n) {
    const float kTarget = 0.25f * 32768.f;   // -12 dBFS speech peaks
    const float kGate = 0.004f * 32768.f;    // about -48 dBFS: below this, hold the gain
    const float kMax = 8.0f, kMin = 1.0f;
    for (size_t i = 0; i < n; i++) {
        float v = static_cast<float>(x[i]);
        float a = fabsf(v);
        env_ = a > env_ ? a : env_ * 0.9995f;  // instant attack, ~250 ms release at 8 kHz
        if (env_ > kGate) {
            float want = kTarget / env_;
            if (want > kMax) want = kMax;
            if (want < kMin) want = kMin;
            // fast down, slow up
            gain_ += (want - gain_) * (want < gain_ ? 0.01f : 0.0005f);
        }
        x[i] = clip16(soft_limit(v * gain_));
    }
}

float volume_gain(uint8_t percent) {
    if (percent > 100) percent = 100;
    float p = percent / 100.f;
    return p * p;
}

void mix_into(int16_t* dst, const int16_t* src, size_t n) {
    for (size_t i = 0; i < n; i++) {
        int32_t s = static_cast<int32_t>(dst[i]) + src[i];
        dst[i] = static_cast<int16_t>(s > 32767 ? 32767 : (s < -32768 ? -32768 : s));
    }
}

void ToneQueue::push(uint16_t freq, uint16_t ms) {
    int next = (tail_ + 1) % kCap;
    if (next == head_ || ms == 0) return;
    q_[tail_] = {freq, ms};
    tail_ = next;
}

bool ToneQueue::render(int16_t* out, size_t n) {
    bool any = false;
    size_t i = 0;
    while (i < n && head_ != tail_) {
        const Seg& s = q_[head_];
        uint32_t total = static_cast<uint32_t>(s.ms) * kVoiceRate / 1000;
        uint32_t ramp = kVoiceRate / 200;  // 5 ms fade in/out, no clicks
        for (; i < n && pos_ < total; i++, pos_++) {
            if (s.freq == 0) continue;
            float amp = 9000.f;
            if (pos_ < ramp) amp *= static_cast<float>(pos_) / ramp;
            if (total - pos_ < ramp) amp *= static_cast<float>(total - pos_) / ramp;
            phase_ += 2.f * static_cast<float>(M_PI) * s.freq / kVoiceRate;
            if (phase_ > 2.f * static_cast<float>(M_PI)) phase_ -= 2.f * static_cast<float>(M_PI);
            int32_t v = out[i] + static_cast<int32_t>(sinf(phase_) * amp);
            out[i] = static_cast<int16_t>(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
            any = true;
        }
        if (pos_ >= total) {
            head_ = (head_ + 1) % kCap;
            pos_ = 0;
        }
    }
    return any;
}

}  // namespace wt
