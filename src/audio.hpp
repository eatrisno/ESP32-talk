#pragma once
// Voice engine: mic -> 8 kHz frames -> radio, and radio -> per-talker jitter buffers ->
// mixer -> speaker. Every node in range is mixed automatically (conference), up to
// kMaxTalkers at once.
#include <cstddef>
#include <cstdint>

#include "protocol.hpp"

namespace audio {

constexpr int kMaxTalkers = 4;

void start();

// Talk on/off (open mic). The voice gate still keeps silence off the air.
void set_talking(bool on);
bool talking();
bool sending();          // mic open and voice detected right now
bool receiving();        // somebody is being played

// Local test: mic goes through the codec and jitter buffer straight back to the speaker.
void set_loopback(bool on);
bool loopback();

void set_volume(uint8_t percent);
uint8_t volume();

// Feedback beep, mixed into the speaker output.
void tone(uint16_t freq, uint16_t ms);

// From the radio task.
void on_audio(uint32_t sender, const wt::AudioPayload& a);

struct Stats {
    uint32_t frames_sent, frames_gated, frames_played, lost, late, mic_bytes,
             spk_underrun;  // speaker writes that timed out (stream stalled)
    float agc_gain, noise_floor;
    int talkers;
};
Stats stats();

}  // namespace audio
