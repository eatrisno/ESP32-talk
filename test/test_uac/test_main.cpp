// Host tests for the patched USB audio driver logic: `pio test -e native`.
// The rate-feedback math is plain C, so it runs here exactly as on the device.
#include <unity.h>

#include <cstdint>
#include <initializer_list>

extern "C" {
#include "uac_feedback.h"
}

namespace {

// 10.14 fixed point, 3 bytes little-endian, as a full-speed async device sends it.
void fb_bytes(double samples_per_ms, uint8_t out[3]) {
    uint32_t v = static_cast<uint32_t>(samples_per_ms * (1 << 14) + 0.5);
    out[0] = v & 0xFF; out[1] = (v >> 8) & 0xFF; out[2] = (v >> 16) & 0xFF;
}

// Runs `ms` milliseconds of OUT packets and returns total sample frames sent.
uint64_t run(uac_fb_t& fb, uint32_t ms) {
    uint64_t frames = 0;
    for (uint32_t i = 0; i < ms; i++) frames += uac_fb_next_packet(&fb) / fb.frame_bytes;
    return frames;
}

void test_nominal_rate_without_feedback() {
    uac_fb_t fb;
    uac_fb_init(&fb, 44100, 4, 196);
    // 44.1 kHz: 44 or 45 frames per ms, exactly 44100 per second, every packet whole frames.
    uint64_t n = run(fb, 1000);
    TEST_ASSERT_EQUAL_UINT64(44100, n);
    for (int i = 0; i < 50; i++) {
        uint32_t b = uac_fb_next_packet(&fb);
        TEST_ASSERT_EQUAL_UINT32(0, b % 4);
        TEST_ASSERT_TRUE(b == 44 * 4 || b == 45 * 4);
    }
}

void test_follows_fast_and_slow_dac() {
    // Async DAC crystal +150 ppm and -200 ppm. Over 10 minutes we must send what it consumes;
    // the nominal rate would be off by 4320 / 5760 samples (90 / 120 ms of audio: a click
    // every few seconds once the device buffer is gone).
    for (double ppm : {150.0, -200.0}) {
        uac_fb_t fb;
        uac_fb_init(&fb, 48000, 4, 200);
        uint8_t v[3];
        double rate = 48.0 * (1 + ppm * 1e-6);
        fb_bytes(rate, v);
        TEST_ASSERT_TRUE(uac_fb_update(&fb, v, 3, false));
        uint64_t sent = run(fb, 600000);
        double want = rate * 600000;
        // 10.14 resolution is 1/16384 sample per ms = 0.06 samples/s: under 40 over 10 min.
        TEST_ASSERT_TRUE(sent > want - 40 && sent < want + 40);
        uac_fb_t nominal;
        uac_fb_init(&nominal, 48000, 4, 200);
        uint64_t nom = run(nominal, 600000);
        TEST_ASSERT_TRUE((nom > sent ? nom - sent : sent - nom) > 4000);  // proves feedback mattered
    }
}

void test_rejects_garbage_and_zero() {
    uac_fb_t fb;
    uac_fb_init(&fb, 48000, 4, 200);
    uint8_t zero[3] = {0, 0, 0}, v[3];
    TEST_ASSERT_FALSE(uac_fb_update(&fb, zero, 3, false));   // devices send 0 before lock
    fb_bytes(52.0, v);                                        // +8 %: nonsense
    TEST_ASSERT_FALSE(uac_fb_update(&fb, v, 3, false));
    TEST_ASSERT_FALSE(uac_fb_update(&fb, v, 2, false));       // short packet
    TEST_ASSERT_EQUAL_UINT32(48000, run(fb, 1000));           // still nominal
    TEST_ASSERT_EQUAL_UINT32(2, fb.rejected);
}

void test_16_16_format_on_full_speed_is_understood() {
    // Some full-speed devices send 16.16 in 4 bytes instead of 10.14 in 3.
    uac_fb_t fb;
    uac_fb_init(&fb, 48000, 4, 200);
    uint32_t q = static_cast<uint32_t>(48.001 * 65536);
    uint8_t v[4] = {static_cast<uint8_t>(q), static_cast<uint8_t>(q >> 8), static_cast<uint8_t>(q >> 16), static_cast<uint8_t>(q >> 24)};
    TEST_ASSERT_TRUE(uac_fb_update(&fb, v, 4, false));
    uint64_t n = run(fb, 1000);
    TEST_ASSERT_TRUE(n >= 48000 && n <= 48002);
}

void test_high_speed_format() {
    uac_fb_t fb;
    uac_fb_init(&fb, 48000, 4, 200);
    uint32_t q = static_cast<uint32_t>(6.0 * 65536);   // 6 samples per 125 us microframe
    uint8_t v[4] = {static_cast<uint8_t>(q), static_cast<uint8_t>(q >> 8), static_cast<uint8_t>(q >> 16), static_cast<uint8_t>(q >> 24)};
    TEST_ASSERT_TRUE(uac_fb_update(&fb, v, 4, true));
    TEST_ASSERT_EQUAL_UINT64(48000, run(fb, 1000));
}

void test_never_exceeds_endpoint_size() {
    uac_fb_t fb;
    uac_fb_init(&fb, 48000, 4, 192);   // MPS holds exactly 48 frames
    uint8_t v[3];
    fb_bytes(48.4, v);                  // device wants 48.4/ms (within 1 %)
    TEST_ASSERT_TRUE(uac_fb_update(&fb, v, 3, false));
    for (int i = 0; i < 1000; i++) TEST_ASSERT_LESS_OR_EQUAL(192, uac_fb_next_packet(&fb));
}


// --- issue 3: alt selection -------------------------------------------------------------

uac_alt_desc_t alt(uint8_t idx, uint8_t ch, uint8_t bits, uint8_t sub, std::initializer_list<uint32_t> rates) {
    uac_alt_desc_t a = {};
    a.alt_idx = idx; a.channels = ch; a.bit_resolution = bits; a.subframe_size = sub;
    a.freq_type = static_cast<uint8_t>(rates.size());
    int i = 0;
    for (uint32_t r : rates) a.freqs[i++] = r;
    return a;
}

void test_alt_pick_24bit_in_3_and_4_bytes() {
    // The headset the old firmware refused: 24-bit offered in a 3-byte and a 4-byte container.
    // Upstream matched only channels + bits and took the LAST alt, so asking for the 3-byte
    // layout streamed the 4-byte one (noise). Each layout must map to its own alt.
    uac_alt_desc_t a[2] = {alt(1, 2, 24, 3, {48000}), alt(2, 2, 24, 4, {48000})};
    TEST_ASSERT_EQUAL_INT(0, uac_alt_pick(a, 2, 2, 24, 3, 48000, 0));
    TEST_ASSERT_EQUAL_INT(1, uac_alt_pick(a, 2, 2, 24, 4, 48000, 0));
    TEST_ASSERT_EQUAL_INT(1, uac_alt_pick(a, 2, 2, 24, 0, 48000, 2));   // explicit alt wins
    TEST_ASSERT_EQUAL_INT(-1, uac_alt_pick(a, 2, 2, 24, 3, 48000, 2));  // contradictory request
}

void test_alt_pick_first_match_and_rates() {
    uac_alt_desc_t a[3] = {alt(1, 2, 16, 2, {44100, 48000}), alt(2, 2, 16, 2, {48000}), alt(3, 1, 16, 2, {16000})};
    TEST_ASSERT_EQUAL_INT(0, uac_alt_pick(a, 3, 2, 16, 2, 48000, 0));   // first, not last
    TEST_ASSERT_EQUAL_INT(2, uac_alt_pick(a, 3, 1, 16, 2, 16000, 0));
    TEST_ASSERT_EQUAL_INT(-1, uac_alt_pick(a, 3, 1, 16, 2, 48000, 0));
    uac_alt_desc_t range = {};
    range.alt_idx = 1; range.channels = 1; range.bit_resolution = 16; range.subframe_size = 2;
    range.freq_type = 0; range.freq_lower = 8000; range.freq_upper = 48000;
    TEST_ASSERT_EQUAL_INT(0, uac_alt_pick(&range, 1, 1, 16, 2, 32000, 0));
    TEST_ASSERT_EQUAL_INT(-1, uac_alt_pick(&range, 1, 1, 16, 2, 96000, 0));
}

// --- issue 1: feedback endpoint detection ---------------------------------------------

void test_feedback_endpoint_detection() {
    // Async speaker: OUT 0x01 (iso, async = 0x05), then IN 0x81 (iso, usage 0 or feedback).
    TEST_ASSERT_TRUE(uac_fb_may_follow(0x01, 0x05));
    TEST_ASSERT_TRUE(uac_fb_is_feedback_ep(0x01, 0x81, 0x01));        // UAC 1.0, usage left 0
    TEST_ASSERT_TRUE(uac_fb_is_feedback_ep(0x01, 0x81, 0x11));        // usage: feedback
    // Adaptive speaker (0x09) has no feedback: stop parsing after its data endpoint.
    TEST_ASSERT_FALSE(uac_fb_may_follow(0x01, 0x09));
    // A microphone's IN data endpoint is never mistaken for feedback.
    TEST_ASSERT_FALSE(uac_fb_may_follow(0x82, 0x05));
    TEST_ASSERT_FALSE(uac_fb_is_feedback_ep(0, 0x82, 0x05));           // first ep of a mic alt
    TEST_ASSERT_FALSE(uac_fb_is_feedback_ep(0x01, 0x83, 0x03));        // interrupt ep (HID)
}

}  // namespace

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_nominal_rate_without_feedback);
    RUN_TEST(test_follows_fast_and_slow_dac);
    RUN_TEST(test_rejects_garbage_and_zero);
    RUN_TEST(test_16_16_format_on_full_speed_is_understood);
    RUN_TEST(test_high_speed_format);
    RUN_TEST(test_never_exceeds_endpoint_size);
    RUN_TEST(test_alt_pick_24bit_in_3_and_4_bytes);
    RUN_TEST(test_alt_pick_first_match_and_rates);
    RUN_TEST(test_feedback_endpoint_detection);
    return UNITY_END();
}
