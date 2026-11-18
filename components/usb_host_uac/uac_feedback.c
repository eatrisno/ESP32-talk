/*
 * Asynchronous-sink rate feedback, see uac_feedback.h.
 *
 * Rates are kept as samples per 1 ms frame in 32.32 fixed point. The nominal rate is exact
 * (sample_rate / 1000 has at most 3 decimals, so it is held as an integer remainder), and
 * device feedback (10.14 or 16.16) fits 32.32 exactly, so neither path drifts over hours.
 */
#include "uac_feedback.h"

void uac_fb_init(uac_fb_t *fb, uint32_t sample_rate, uint32_t frame_bytes, uint32_t ep_mps)
{
    fb->sample_rate = sample_rate;
    fb->nominal_q16 = (uint32_t)(((uint64_t)sample_rate << 16) / 1000u);
    fb->rate_q16 = fb->nominal_q16;
    fb->use_feedback = false;
    fb->rate_q32 = 0;
    fb->accum_q32 = 0;
    fb->nominal_rem = 0;
    fb->frame_bytes = frame_bytes ? frame_bytes : 1;
    fb->max_frames = ep_mps / fb->frame_bytes;
    fb->accepted = 0;
    fb->rejected = 0;
}

bool uac_fb_update(uac_fb_t *fb, const uint8_t *d, uint32_t len, bool high_speed)
{
    uint64_t q32;  /* samples per 1 ms frame, 32.32 */
    if (!high_speed) {
        if (len < 3) {
            return false;
        }
        uint32_t v = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16);
        q32 = (uint64_t)v << 18;  /* 10.14 -> 32.32 */
        /* Some full-speed devices send 16.16 in 4 bytes instead of 10.14 in 3. */
        if (len >= 4) {
            uint32_t w = v | ((uint32_t)d[3] << 24);
            uint64_t alt = (uint64_t)w << 16;
            uint64_t nom = (uint64_t)fb->nominal_q16 << 16;
            uint64_t d1 = q32 > nom ? q32 - nom : nom - q32;
            uint64_t d2 = alt > nom ? alt - nom : nom - alt;
            if (d2 < d1) {
                q32 = alt;
            }
        }
    } else {
        if (len < 4) {
            return false;
        }
        uint32_t v = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
        q32 = (uint64_t)v << 19;  /* 16.16 per 125 us microframe -> per 1 ms, 32.32 */
    }
    uint64_t nom = (uint64_t)fb->nominal_q16 << 16;
    if (q32 < nom - nom / 100u || q32 > nom + nom / 100u) {
        fb->rejected++;   /* 0 before the device locks, or nonsense: keep the last good rate */
        return false;
    }
    fb->rate_q32 = q32;
    fb->rate_q16 = (uint32_t)(q32 >> 16);
    fb->use_feedback = true;
    fb->accepted++;
    return true;
}

uint32_t uac_fb_next_packet(uac_fb_t *fb)
{
    uint32_t frames;
    if (!fb->use_feedback) {
        /* Exact nominal: sample_rate/1000 frames per ms, remainder carried in units of 1/1000. */
        frames = fb->sample_rate / 1000u;
        fb->nominal_rem += fb->sample_rate % 1000u;
        if (fb->nominal_rem >= 1000u) {
            fb->nominal_rem -= 1000u;
            frames++;
        }
    } else {
        uint64_t total = fb->accum_q32 + fb->rate_q32;
        frames = (uint32_t)(total >> 32);
        fb->accum_q32 = total - ((uint64_t)frames << 32);
    }
    if (frames > fb->max_frames) {
        frames = fb->max_frames;  /* never more than the endpoint can take */
    }
    return frames * fb->frame_bytes;
}

bool uac_fb_is_feedback_ep(uint8_t data_ep_addr, uint8_t ep_addr, uint8_t ep_attr)
{
    if ((ep_attr & 0x03) != 0x01) {
        return false;  /* feedback endpoints are isochronous */
    }
    if ((ep_attr & 0x30) == 0x10) {
        return true;   /* usage type: feedback (UAC 1.0 devices often leave this 0) */
    }
    /* UAC 1.0: an IN endpoint right after an OUT data endpoint in the same alt. */
    return data_ep_addr != 0 && !(data_ep_addr & 0x80) && (ep_addr & 0x80);
}

bool uac_fb_may_follow(uint8_t data_ep_addr, uint8_t data_ep_attr)
{
    return data_ep_addr != 0 && !(data_ep_addr & 0x80) && (data_ep_attr & 0x0C) == 0x04;
}

int uac_alt_pick(const uac_alt_desc_t *alts, int n, uint8_t channels, uint8_t bit_resolution,
                 uint8_t subframe_size, uint32_t rate, uint8_t alt_idx)
{
    for (int i = 0; i < n; i++) {
        const uac_alt_desc_t *a = &alts[i];
        if (alt_idx && a->alt_idx != alt_idx) {
            continue;
        }
        if (a->channels != channels || a->bit_resolution != bit_resolution) {
            continue;
        }
        if (subframe_size && a->subframe_size != subframe_size) {
            continue;
        }
        if (a->freq_type == 0) {
            if (rate >= a->freq_lower && rate <= a->freq_upper) {
                return i;
            }
            continue;
        }
        for (int j = 0; j < a->freq_type && j < 8; j++) {
            if (a->freqs[j] == rate) {
                return i;
            }
        }
    }
    return -1;
}
