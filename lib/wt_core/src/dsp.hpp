#pragma once
// Hardware-free signal processing: sample-rate conversion, filters, gain, tones.
#include <cstddef>
#include <cstdint>

namespace wt {

constexpr uint32_t kVoiceRate = 8000;

int16_t clip16(float x);
// Smooth limiter: identity below -3 dBFS, always stays under 32000.
float soft_limit(float y);

struct Biquad {
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    float z1 = 0, z2 = 0;
    void lowpass(float fs, float fc, float q);
    float run(float x) {
        float y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
};

// 8th-order Butterworth low-pass (4 cascaded biquads), about -45 dB one octave above fc.
struct LowPass8 {
    Biquad s[4];
    void design(float fs, float fc);
    float run(float x) {
        for (auto& b : s) x = b.run(x);
        return x;
    }
};

// Streaming rate converter for mono int16. Exact rational step (no drift across blocks),
// anti-alias filter before decimation and anti-image filter after interpolation.
// Output is identical whether the input arrives in one block or many.
class Resampler {
public:
    void init(uint32_t in_rate, uint32_t out_rate);
    // Returns number of samples written (never more than out_max).
    size_t process(const int16_t* in, size_t n, int16_t* out, size_t out_max);
    // Upper bound of output samples for n input samples.
    size_t max_out(size_t n) const { return (n * out_ + in_ - 1) / in_ + 1; }
    uint32_t in_rate() const { return in_; }
    uint32_t out_rate() const { return out_; }

private:
    static constexpr size_t kChunk = 256;
    uint32_t in_ = kVoiceRate, out_ = kVoiceRate;
    uint64_t pos_ = 0;   // next output position in input samples * out_; 0 = prev_
    float prev_ = 0;     // last (filtered) input sample of the previous block
    float buf_[kChunk + 1];
    LowPass8 lp_;
    bool filter_in_ = false, filter_out_ = false;
};

// One-pole DC / rumble blocker (about 65 Hz at 8 kHz).
struct DcBlocker {
    float x1 = 0, y1 = 0;
    float run(float x) {
        float y = x - x1 + 0.95f * y1;
        x1 = x;
        y1 = y;
        return y;
    }
};

// Slow automatic gain for the microphone. Raises quiet speech up to +18 dB and never clips.
// `speech` comes from the voice gate (measured on the frame before gain): the gain only adapts
// on speech, and holds in pauses, so background hiss is never pumped up between words.
class Agc {
public:
    void process(int16_t* x, size_t n, bool speech = true);
    float gain() const { return gain_; }

private:
    float gain_ = 2.0f;
    float env_ = 0;
};

// Speaker volume 0..100 to linear gain (square law: 50% = -12 dB).
float volume_gain(uint8_t percent);

// Sums src into a float mix bus (no clipping: limit once at the end with soft_limit).
void mix_into(float* dst, const int16_t* src, size_t n);

// Feedback beeps rendered at kVoiceRate and mixed into the speaker path, so there is only
// one writer to the USB speaker.
class ToneQueue {
public:
    // Appends a tone; freq 0 = silence gap. Drops the tone if the queue is full.
    void push(uint16_t freq, uint16_t ms);
    void clear() { head_ = tail_ = 0; pos_ = 0; }
    bool busy() const { return head_ != tail_; }
    // Adds tone samples into out. Returns true if anything was rendered.
    bool render(int16_t* out, size_t n);

private:
    struct Seg { uint16_t freq, ms; };
    static constexpr int kCap = 8;
    Seg q_[kCap] = {};
    int head_ = 0, tail_ = 0;
    uint32_t pos_ = 0;
    float phase_ = 0;
};

}  // namespace wt
