#include "format.hpp"

#include <cstdlib>

namespace wt {
namespace {

const uint32_t kRatePref[] = {48000, 16000, 32000, 24000, 8000, 96000, 44100, 22050, 11025};

int rate_score(uint32_t r) {
    for (size_t i = 0; i < sizeof(kRatePref) / sizeof(kRatePref[0]); i++)
        if (kRatePref[i] == r) return static_cast<int>(i);
    return 100;
}

bool supports(const AltFormat& a, uint32_t r) {
    if (a.freq_count == 0) return r >= a.freq_min && r <= a.freq_max;
    for (int i = 0; i < a.freq_count && i < 8; i++)
        if (a.freqs[i] == r) return true;
    return false;
}

int32_t read_sample(const uint8_t* p, uint8_t sub) {
    // Returns the sample scaled to 16 bits (top 16 bits of the container).
    switch (sub) {
        case 2: return static_cast<int16_t>(p[0] | (p[1] << 8));
        case 3: return static_cast<int16_t>(p[1] | (p[2] << 8));
        case 4: return static_cast<int16_t>(p[2] | (p[3] << 8));
        default: return 0;
    }
}

void write_sample(uint8_t* p, uint8_t sub, int16_t s) {
    // Left-justified in the container: the device's bit resolution is the MSB part.
    uint16_t u = static_cast<uint16_t>(s);
    for (uint8_t b = 0; b + 2 < sub; b++) p[b] = 0;
    p[sub - 2] = static_cast<uint8_t>(u & 0xFF);
    p[sub - 1] = static_cast<uint8_t>(u >> 8);
}

}  // namespace

StreamFormat choose_format(const AltFormat* alts, size_t n) {
    StreamFormat best;
    long best_score = 1L << 30;
    for (size_t i = 0; i < n; i++) {
        const AltFormat& a = alts[i];
        if (a.channels < 1 || a.channels > 2) continue;
        if (a.subframe_bytes < 2 || a.subframe_bytes > 4) continue;
        uint32_t pick = 0;
        int rs = 1000;
        for (uint32_t r : kRatePref) {
            if (supports(a, r) && rate_score(r) < rs) {
                rs = rate_score(r);
                pick = r;
            }
        }
        if (!pick) {  // nothing from the list: take the closest to 48k
            if (a.freq_count == 0) {
                pick = a.freq_max < 48000 ? a.freq_max : a.freq_min;
            } else {
                for (int k = 0; k < a.freq_count && k < 8; k++)
                    if (!pick || labs(static_cast<long>(a.freqs[k]) - 48000) <
                                     labs(static_cast<long>(pick) - 48000))
                        pick = a.freqs[k];
            }
            if (pick < 8000 || pick > 96000) continue;
        }
        long score = rs * 100L + (a.channels - 1) * 10L + (a.subframe_bytes - 2);
        if (score < best_score) {
            best_score = score;
            best.channels = a.channels;
            best.subframe_bytes = a.subframe_bytes;
            best.bits = a.bits;
            best.rate = pick;
        }
    }
    return best;
}

size_t pcm_to_mono(const uint8_t* raw, size_t bytes, const StreamFormat& f, int16_t* out,
                   size_t out_max, StereoPick* pick) {
    if (!f.valid()) return 0;
    size_t fb = f.frame_bytes();
    size_t frames = bytes / fb;
    if (frames > out_max) frames = out_max;
    if (f.channels == 1) {
        for (size_t i = 0; i < frames; i++)
            out[i] = static_cast<int16_t>(read_sample(raw + i * fb, f.subframe_bytes));
        return frames;
    }
    int mode = 2;
    if (pick) {
        int32_t peak0 = 0, peak1 = 0;
        for (size_t i = 0; i < frames; i++) {
            int32_t a = abs(read_sample(raw + i * fb, f.subframe_bytes));
            int32_t b = abs(read_sample(raw + i * fb + f.subframe_bytes, f.subframe_bytes));
            if (a > peak0) peak0 = a;
            if (b > peak1) peak1 = b;
        }
        // Decide by level ratio with a wide dead band, so noise near any fixed threshold
        // cannot flip it: one channel 26 dB below the other = single-sided mic; both within
        // 12 dB = real stereo; in between, or both near silence (no one speaking) = keep.
        const int32_t kQuiet = 200;  // about -44 dBFS: below this nobody is speaking
        int want = -1;  // -1 = this block says nothing (quiet, or in the dead band)
        int32_t hi = peak0 > peak1 ? peak0 : peak1;
        if (hi > kQuiet) {
            if (peak0 > peak1 * 20) want = 0;
            else if (peak1 > peak0 * 20) want = 1;
            else if (peak0 * 4 > peak1 && peak1 * 4 > peak0) want = 2;
        }
        if (want < 0) {
            // no vote either way
        } else if (want == pick->mode) {
            pick->votes = 0;
        } else {
            if (want != pick->pending) {
                pick->pending = want;
                pick->votes = 0;
            }
            if (++pick->votes >= 50) {  // ~0.5 s of 10 ms blocks
                pick->mode = want;
                pick->votes = 0;
            }
        }
        mode = pick->mode;
    }
    for (size_t i = 0; i < frames; i++) {
        int32_t a = read_sample(raw + i * fb, f.subframe_bytes);
        int32_t b = read_sample(raw + i * fb + f.subframe_bytes, f.subframe_bytes);
        out[i] = static_cast<int16_t>(mode == 0 ? a : mode == 1 ? b : (a + b) / 2);
    }
    return frames;
}

size_t mono_to_pcm(const int16_t* in, size_t n, const StreamFormat& f, uint8_t* raw,
                   size_t raw_max) {
    if (!f.valid()) return 0;
    size_t fb = f.frame_bytes();
    size_t frames = raw_max / fb;
    if (frames > n) frames = n;
    for (size_t i = 0; i < frames; i++)
        for (uint8_t c = 0; c < f.channels; c++)
            write_sample(raw + i * fb + c * f.subframe_bytes, f.subframe_bytes, in[i]);
    return frames * fb;
}

}  // namespace wt
