#pragma once
// Picks a USB audio stream format from what the headset advertises, and converts
// between the headset's raw PCM layout and mono int16.
#include <cstddef>
#include <cstdint>

namespace wt {

// One alternate setting as advertised by the device (UAC 1.0 Type I PCM).
struct AltFormat {
    uint8_t  channels = 0;
    uint8_t  subframe_bytes = 0;   // 2, 3 or 4
    uint8_t  bits = 0;             // 16, 24, 32 (bit resolution)
    uint8_t  freq_count = 0;       // 0 = continuous range [freq_min, freq_max]
    uint32_t freqs[8] = {};
    uint32_t freq_min = 0, freq_max = 0;
};

struct StreamFormat {
    uint8_t  channels = 0;
    uint8_t  subframe_bytes = 0;
    uint8_t  bits = 0;
    uint32_t rate = 0;
    bool valid() const { return rate && channels && subframe_bytes; }
    size_t frame_bytes() const { return static_cast<size_t>(channels) * subframe_bytes; }
};

// Best format for 8 kHz voice. Rate preference: 48k, 16k, 32k, 24k, 8k, 96k, 44.1k, 22.05k,
// 11.025k, then anything else (exact integer ratios first, they cost the least CPU and
// alias the least). Fewer channels and 16-bit win ties. Returns invalid if nothing usable.
StreamFormat choose_format(const AltFormat* alts, size_t n);

// Raw interleaved little-endian PCM to mono int16. Stereo is averaged; if one channel is
// silent over the whole block (single-sided mic), the live channel is used at full level.
size_t pcm_to_mono(const uint8_t* raw, size_t bytes, const StreamFormat& f, int16_t* out,
                   size_t out_max);

// Mono int16 to raw interleaved PCM (same sample on every channel).
size_t mono_to_pcm(const int16_t* in, size_t n, const StreamFormat& f, uint8_t* raw,
                   size_t raw_max);

}  // namespace wt
