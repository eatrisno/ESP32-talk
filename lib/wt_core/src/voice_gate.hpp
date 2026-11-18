#pragma once
// Decides per 40 ms frame whether the mic carries speech worth sending. Tracks the
// background noise floor, opens instantly on speech, closes after a hangover. Saves
// airtime (three open mics would otherwise fill the channel) and keeps fan or wind noise
// out of everyone's ears.
#include <cstddef>
#include <cstdint>

namespace wt {

class VoiceGate {
public:
    static constexpr int kHangFrames = 10;     // 400 ms after the last speech frame
    // Returns true if this frame should be sent.
    bool update(const int16_t* x, size_t n);
    float noise_floor() const { return floor_; }
    float last_rms() const { return rms_; }

private:
    float floor_ = 200.f;
    float rms_ = 0.f;
    int hang_ = 0;
};

// Runs a 1 kHz test tone through the whole voice path (mic rate -> 8 kHz -> ADPCM -> frame
// -> jitter buffer -> speaker rate) and returns the output SNR in dB, or a negative value
// if the tone did not come out at the expected level. Used by the host tests and by the
// on-device `selftest` command.
float pipeline_selftest(uint32_t mic_rate, uint32_t spk_rate);

}  // namespace wt
