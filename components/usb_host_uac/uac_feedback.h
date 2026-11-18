/*
 * Asynchronous-sink rate feedback for UAC 1.0 OUT streams (USB 2.0 spec 5.12.4.2).
 *
 * An asynchronous speaker runs its DAC from its own crystal and reports, on a feedback IN
 * endpoint, how many samples per 1 ms frame it actually consumes, as 10.14 fixed point
 * (3 bytes on full speed). The host must size each OUT packet from that value, not from the
 * nominal rate, or the device's buffer slowly over- or underflows and it clicks.
 *
 * Plain C, no ESP-IDF calls: also built by the host tests.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t sample_rate;   /* nominal Hz */
    uint32_t nominal_q16;   /* nominal samples per frame, 16.16 (for range checks only) */
    uint32_t rate_q16;      /* current samples per frame, 16.16 (reporting) */
    bool     use_feedback;  /* a valid feedback value has been received */
    uint64_t rate_q32;      /* samples per frame from feedback, 32.32 */
    uint64_t accum_q32;     /* fractional samples carried between packets, 32.32 */
    uint32_t nominal_rem;   /* nominal path: carried remainder, in 1/1000 sample */
    uint32_t frame_bytes;   /* bytes per sample frame (channels x subframe) */
    uint32_t max_frames;    /* largest packet the endpoint allows, in sample frames */
    uint32_t accepted;      /* feedback values taken */
    uint32_t rejected;      /* feedback values outside +-1% of nominal (ignored) */
} uac_fb_t;

/* sample_rate in Hz; ep_mps = wMaxPacketSize of the OUT endpoint. */
void uac_fb_init(uac_fb_t *fb, uint32_t sample_rate, uint32_t frame_bytes, uint32_t ep_mps);

/* Feed one feedback packet (len = bytes received: 3 on full speed, 4 on high speed).
 * Values more than 1% from nominal are ignored (devices send 0 before they lock).
 * Returns true if the value was used. */
bool uac_fb_update(uac_fb_t *fb, const uint8_t *data, uint32_t len, bool high_speed);

/* Bytes for the next 1 ms OUT packet, always a whole number of sample frames. */
uint32_t uac_fb_next_packet(uac_fb_t *fb);

/* Is ep (bEndpointAddress, bmAttributes) the feedback endpoint of the OUT data endpoint
 * already parsed in this alt (data_ep_addr, 0 if none yet)? */
bool uac_fb_is_feedback_ep(uint8_t data_ep_addr, uint8_t ep_addr, uint8_t ep_attr);

/* Keep parsing after the data endpoint's class descriptor? Only for an asynchronous OUT. */
bool uac_fb_may_follow(uint8_t data_ep_addr, uint8_t data_ep_attr);

/* One alternate setting as the driver sees it, for alt selection. */
typedef struct {
    uint8_t  alt_idx;        /* bAlternateSetting */
    uint8_t  channels, bit_resolution, subframe_size;
    uint8_t  freq_type;      /* 0 = range */
    uint32_t freqs[8];
    uint32_t freq_lower, freq_upper;
} uac_alt_desc_t;

/* Index into alts of the FIRST alt matching everything requested (subframe_size 0 = any,
 * alt_idx 0 = any), or -1. */
int uac_alt_pick(const uac_alt_desc_t *alts, int n, uint8_t channels, uint8_t bit_resolution,
                 uint8_t subframe_size, uint32_t rate, uint8_t alt_idx);

#ifdef __cplusplus
}
#endif
