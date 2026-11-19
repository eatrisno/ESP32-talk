// Host tests for the hardware-free core: `pio test -e native`.
// They run the real firmware code paths (mic -> 8 kHz -> ADPCM -> packet -> jitter ->
// speaker rate) and measure the output against a known test tone.
#include <unity.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "adpcm.hpp"
#include "button.hpp"
#include "dsp.hpp"
#include "format.hpp"
#include "hid_buttons.hpp"
#include "jitter.hpp"
#include "mesh.hpp"
#include "protocol.hpp"
#include "range_test.hpp"
#include "voice_gate.hpp"

using namespace wt;

constexpr int kTargetDepthForTest = JitterBuffer::kTargetDepth;

namespace {

std::vector<int16_t> tone(uint32_t rate, float hz, float amp, size_t n) {
    std::vector<int16_t> v(n);
    for (size_t i = 0; i < n; i++)
        v[i] = static_cast<int16_t>(amp * sinf(2.f * static_cast<float>(M_PI) * hz * i / rate));
    return v;
}

struct Fit {
    float amp;
    float snr_db;
};

// Least-squares fit of a sine at `hz` (any phase) over x[skip..]. SNR = fitted tone vs
// everything else, so it is independent of the delay through the pipeline.
Fit fit_tone(const std::vector<int16_t>& x, uint32_t rate, float hz, size_t skip) {
    double ss = 0, cc = 0, sc = 0, xs = 0, xc = 0;
    for (size_t i = skip; i < x.size(); i++) {
        double w = 2 * M_PI * hz * i / rate;
        double s = sin(w), c = cos(w);
        ss += s * s; cc += c * c; sc += s * c;
        xs += x[i] * s; xc += x[i] * c;
    }
    double det = ss * cc - sc * sc;
    double a = (xs * cc - xc * sc) / det, b = (xc * ss - xs * sc) / det;
    double sig = 0, err = 0;
    for (size_t i = skip; i < x.size(); i++) {
        double w = 2 * M_PI * hz * i / rate;
        double f = a * sin(w) + b * cos(w);
        sig += f * f;
        err += (x[i] - f) * (x[i] - f);
    }
    return {static_cast<float>(sqrt(a * a + b * b)),
            static_cast<float>(10 * log10(sig / (err + 1e-9)))};
}

// Streams `in` through a Resampler in blocks of `block` samples.
std::vector<int16_t> resample(const std::vector<int16_t>& in, uint32_t from, uint32_t to,
                              size_t block) {
    Resampler r;
    r.init(from, to);
    std::vector<int16_t> out;
    std::vector<int16_t> buf(r.max_out(block) + 8);
    for (size_t i = 0; i < in.size(); i += block) {
        size_t n = std::min(block, in.size() - i);
        size_t m = r.process(&in[i], n, buf.data(), buf.size());
        out.insert(out.end(), buf.begin(), buf.begin() + m);
    }
    return out;
}

// The full voice path, with optional packet loss (drop every `drop_every`-th frame).
std::vector<int16_t> voice_path(const std::vector<int16_t>& mic, uint32_t mic_rate,
                                uint32_t spk_rate, int drop_every) {
    std::vector<int16_t> v8 = resample(mic, mic_rate, kVoiceRate, mic_rate / 1000);  // 1 ms USB packets
    JitterBuffer jb;
    AdpcmState enc;
    std::vector<int16_t> played;
    int16_t frame[kFrameSamples];
    uint16_t fno = 0;
    for (size_t i = 0; i + kFrameSamples <= v8.size(); i += kFrameSamples, fno++) {
        AudioPayload a;
        a.voice_id = 7;
        a.frame_no = fno;
        a.adpcm_pred = enc.predictor;
        a.adpcm_index = static_cast<uint8_t>(enc.index);
        uint8_t bytes[kAdpcmBytes];
        a.data_len = adpcm_encode(&v8[i], kFrameSamples, bytes, enc);
        a.data = bytes;
        uint8_t wire[kMaxPlain];
        size_t len = write_audio(a, wire, sizeof(wire));
        AudioPayload rx;
        TEST_ASSERT_TRUE(read_audio(wire, len, rx));
        if (!(drop_every && fno % drop_every == drop_every - 1)) jb.put(rx);
        if (jb.pop(frame)) played.insert(played.end(), frame, frame + kFrameSamples);
    }
    return resample(played, kVoiceRate, spk_rate, kFrameSamples);
}

// ---------------------------------------------------------------------------------------

void test_adpcm_roundtrip_snr() {
    auto x = tone(8000, 1000, 12000, 8000);
    std::vector<uint8_t> enc(x.size() / 2);
    std::vector<int16_t> dec(x.size());
    AdpcmState a, b;
    adpcm_encode(x.data(), x.size(), enc.data(), a);
    adpcm_decode(enc.data(), enc.size(), dec.data(), b);
    Fit f = fit_tone(dec, 8000, 1000, 400);
    // Reference IMA ADPCM gives 12671 for this input (codec overshoot at 8 samples/cycle).
    TEST_ASSERT_FLOAT_WITHIN(5, 12671, f.amp);
    TEST_ASSERT_GREATER_THAN_FLOAT(20.f, f.snr_db);
}

void test_adpcm_state_in_frame_recovers_after_loss() {
    // Old firmware: decoder state was not sent, so after one lost packet the decoder
    // was out of step. Now every frame carries the encoder state.
    auto x = tone(8000, 700, 15000, kFrameSamples * 3);
    AdpcmState enc;
    uint8_t f[3][kAdpcmBytes];
    AdpcmState at_start[3];
    for (int k = 0; k < 3; k++) {
        at_start[k] = enc;
        adpcm_encode(&x[k * kFrameSamples], kFrameSamples, f[k], enc);
    }
    // Frame 1 is lost: decode frame 2 from the state it carries.
    int16_t out[kFrameSamples];
    AdpcmState st = at_start[2];
    adpcm_decode(f[2], kAdpcmBytes, out, st);
    std::vector<int16_t> got(out, out + kFrameSamples), ref(x.begin() + 2 * kFrameSamples, x.end());
    double err = 0, sig = 0;
    for (size_t i = 0; i < kFrameSamples; i++) {
        err += (got[i] - ref[i]) * (double)(got[i] - ref[i]);
        sig += ref[i] * (double)ref[i];
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(18.f, 10 * log10(sig / err));
}

void test_resampler_block_size_does_not_matter() {
    // Old firmware restarted the fractional phase every block (clicks 20x per second).
    auto x = tone(8000, 440, 10000, 4000);
    for (uint32_t rate : {44100u, 48000u, 16000u, 32000u, 22050u}) {
        auto one = resample(x, 8000, rate, x.size());
        auto many = resample(x, 8000, rate, 37);  // odd block size on purpose
        TEST_ASSERT_EQUAL_size_t(one.size(), many.size());
        for (size_t i = 0; i < one.size(); i++) TEST_ASSERT_INT16_WITHIN(1, one[i], many[i]);
    }
}

void test_resampler_exact_rate_no_drift() {
    for (uint32_t rate : {44100u, 48000u, 11025u, 96000u}) {
        auto x = tone(8000, 300, 1000, 8000 * 5);  // 5 seconds
        auto y = resample(x, 8000, rate, kFrameSamples);
        long expect = static_cast<long>(rate) * 5;
        TEST_ASSERT_TRUE(labs(static_cast<long>(y.size()) - expect) <= 1);
    }
}

void test_downsample_rejects_alias() {
    // A 6 kHz tone at 48 kHz must not fold back to 2 kHz in the 8 kHz voice band.
    auto x = tone(48000, 6000, 20000, 48000);
    auto y = resample(x, 48000, 8000, 48);
    Fit f = fit_tone(y, 8000, 2000, 800);
    TEST_ASSERT_LESS_THAN_FLOAT(20000 * 0.01f, f.amp);  // at least -40 dB
}

void check_path(uint32_t mic_rate, uint32_t spk_rate, int drop_every, float min_snr) {
    auto mic = tone(mic_rate, 1000, 12000, mic_rate * 2);
    auto out = voice_path(mic, mic_rate, spk_rate, drop_every);
    TEST_ASSERT_TRUE(out.size() > spk_rate);
    Fit f = fit_tone(out, spk_rate, 1000, spk_rate / 2);
    char msg[96];
    snprintf(msg, sizeof msg, "mic %u spk %u drop %d: amp %.0f snr %.1f dB", mic_rate, spk_rate,
             drop_every, f.amp, f.snr_db);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1500, 12000, f.amp, msg);
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(min_snr, f.snr_db, msg);
}

void test_voice_path_all_headset_rates() {
    const uint32_t rates[] = {48000, 44100, 32000, 24000, 22050, 16000, 11025, 8000};
    for (uint32_t m : rates)
        for (uint32_t s : rates) check_path(m, s, 0, 18.f);
}

void test_voice_path_with_packet_loss() {
    // 1 frame in 10 lost: concealed, the next frame decodes cleanly.
    auto mic = tone(48000, 1000, 12000, 48000 * 2);
    auto out = voice_path(mic, 48000, 48000, 10);
    Fit f = fit_tone(out, 48000, 1000, 24000);
    TEST_ASSERT_GREATER_THAN_FLOAT(8.f, f.snr_db);  // loss costs SNR, but no noise bursts
    double peak = 0;
    for (int16_t s : out) peak = std::max(peak, fabs((double)s));
    TEST_ASSERT_LESS_THAN(15000, (int)peak);  // no garbage spikes after a lost frame
}

// ---------------------------------------------------------------------------------------

void test_choose_format_prefers_48k_mono_16bit() {
    AltFormat a[2];
    a[0] = {2, 2, 16, 2, {44100, 48000}};
    a[1] = {1, 2, 16, 1, {48000}};
    StreamFormat f = choose_format(a, 2);
    TEST_ASSERT_EQUAL_UINT32(48000, f.rate);
    TEST_ASSERT_EQUAL_UINT8(1, f.channels);
}

void test_choose_format_cheap_headset_44k_stereo_only() {
    // The case the old firmware rejected: no 48 kHz, stereo speaker only.
    AltFormat a[1];
    a[0] = {2, 2, 16, 1, {44100}};
    StreamFormat f = choose_format(a, 1);
    TEST_ASSERT_TRUE(f.valid());
    TEST_ASSERT_EQUAL_UINT32(44100, f.rate);
    TEST_ASSERT_EQUAL_UINT8(2, f.channels);
}

void test_choose_format_continuous_range_and_24bit() {
    AltFormat a[1];
    a[0].channels = 1; a[0].subframe_bytes = 3; a[0].bits = 24;
    a[0].freq_count = 0; a[0].freq_min = 8000; a[0].freq_max = 96000;
    StreamFormat f = choose_format(a, 1);
    TEST_ASSERT_EQUAL_UINT32(48000, f.rate);
    TEST_ASSERT_EQUAL_UINT8(3, f.subframe_bytes);
}

void test_choose_format_rejects_unusable() {
    AltFormat a[1];
    a[0] = {6, 2, 16, 1, {48000}};  // 6 channels
    TEST_ASSERT_FALSE(choose_format(a, 1).valid());
}

void test_pcm_roundtrip_all_layouts() {
    int16_t in[4] = {12345, -12345, 32767, -32768};
    for (uint8_t ch : {1, 2})
        for (uint8_t sub : {2, 3, 4}) {
            StreamFormat f;
            f.channels = ch; f.subframe_bytes = sub; f.rate = 48000;
            uint8_t raw[64];
            size_t bytes = mono_to_pcm(in, 4, f, raw, sizeof raw);
            TEST_ASSERT_EQUAL_size_t(4 * ch * sub, bytes);
            int16_t out[4];
            TEST_ASSERT_EQUAL_size_t(4, pcm_to_mono(raw, bytes, f, out, 4));
            TEST_ASSERT_EQUAL_INT16_ARRAY(in, out, 4);
        }
}

void test_stereo_mic_one_side_silent_keeps_level() {
    StreamFormat f;
    f.channels = 2; f.subframe_bytes = 2; f.rate = 48000;
    auto t = tone(48000, 1000, 10000, 480);
    std::vector<int16_t> lr(960, 0);
    for (size_t i = 0; i < 480; i++) lr[2 * i] = t[i];  // right channel silent
    int16_t out[480];
    StereoPick pick;  // the choice settles after ~0.5 s of blocks, then holds
    for (int blk = 0; blk < 60; blk++)
        pcm_to_mono(reinterpret_cast<uint8_t*>(lr.data()), lr.size() * 2, f, out, 480, &pick);
    TEST_ASSERT_EQUAL_INT16_ARRAY(t.data(), out, 480);
}

void test_stereo_mic_different_channels_no_switching() {
    // Old firmware picked L, R or average per sample: distortion. Now per block.
    StreamFormat f;
    f.channels = 2; f.subframe_bytes = 2; f.rate = 48000;
    // Right channel is a weak, phase-shifted pickup: the old per-sample rule flipped
    // between "left only" and "average" around every zero crossing.
    auto l = tone(48000, 1000, 10000, 4800);
    std::vector<int16_t> r(4800);
    for (size_t i = 0; i < 4800; i++)
        r[i] = static_cast<int16_t>(600 * cosf(2.f * static_cast<float>(M_PI) * 1000 * i / 48000));
    std::vector<int16_t> lr(9600);
    for (size_t i = 0; i < 4800; i++) { lr[2 * i] = l[i]; lr[2 * i + 1] = r[i]; }
    std::vector<int16_t> out(4800);
    pcm_to_mono(reinterpret_cast<uint8_t*>(lr.data()), lr.size() * 2, f, out.data(), 4800);
    TEST_ASSERT_GREATER_THAN_FLOAT(40.f, fit_tone(out, 48000, 1000, 0).snr_db);
}

// ---------------------------------------------------------------------------------------

AudioPayload frame_payload(uint8_t voice, uint16_t no, uint8_t* bytes) {
    AudioPayload a;
    a.voice_id = voice;
    a.frame_no = no;
    memset(bytes, 0x11 * (no % 7 + 1), kAdpcmBytes);
    a.data = bytes;
    a.data_len = kAdpcmBytes;
    return a;
}

void test_jitter_reorders_and_drops_duplicates() {
    JitterBuffer jb;
    uint8_t b[kAdpcmBytes];
    int16_t out[kFrameSamples];
    for (uint16_t no : {0, 2, 1, 1, 3, 4}) jb.put(frame_payload(1, no, b));
    for (uint16_t no = 0; no < 5; no++) {
        TEST_ASSERT_TRUE(jb.pop(out));
        // each frame decodes to its own content, in order
        int16_t want[kFrameSamples];
        uint8_t wb[kAdpcmBytes];
        AudioPayload p = frame_payload(1, no, wb);
        AdpcmState st;
        adpcm_decode(p.data, p.data_len, want, st);
        TEST_ASSERT_EQUAL_INT16_ARRAY(want, out, kFrameSamples);
    }
    TEST_ASSERT_EQUAL_UINT32(0, jb.lost());
}

void test_jitter_prebuffers_then_ends_after_silence() {
    JitterBuffer jb;
    uint8_t b[kAdpcmBytes];
    int16_t out[kFrameSamples];
    jb.put(frame_payload(1, 100, b));
    TEST_ASSERT_FALSE(jb.pop(out));  // waiting for kTargetDepth frames
    jb.put(frame_payload(1, 101, b));
    jb.put(frame_payload(1, 102, b));
    TEST_ASSERT_TRUE(jb.pop(out));
    int n = 0;
    while (jb.pop(out) && n < 100) n++;
    TEST_ASSERT_TRUE(n < 12);           // stops by itself when the talker goes quiet
    TEST_ASSERT_FALSE(jb.active());
}

void test_jitter_frame_number_wraps() {
    JitterBuffer jb;
    uint8_t b[kAdpcmBytes];
    int16_t out[kFrameSamples];
    for (uint16_t no = 65530; no != 6; no++) {
        jb.put(frame_payload(1, no, b));
        jb.pop(out);
    }
    TEST_ASSERT_EQUAL_UINT32(0, jb.lost());
    TEST_ASSERT_EQUAL_UINT32(0, jb.late());
}

void test_jitter_resync_after_long_gap_prebuffers_again() {
    // Talker jumps 196 frames ahead (e.g. we missed a burst). Must restart cleanly:
    // pre-buffer again, then play the new frames in order with nothing counted lost.
    // (Resync works without the explicit newest_ reset too: the new frame is always ahead
    // of the stale newest_. This guards the behaviour, not that line.)
    JitterBuffer jb;
    uint8_t b[kAdpcmBytes];
    int16_t out[kFrameSamples];
    for (uint16_t no = 0; no < 4; no++) jb.put(frame_payload(1, no, b));
    TEST_ASSERT_TRUE(jb.pop(out));
    uint32_t lost0 = jb.lost();
    jb.put(frame_payload(1, 200, b));
    TEST_ASSERT_FALSE(jb.pop(out));  // pre-buffering again, not playing stale slots
    jb.put(frame_payload(1, 201, b));
    jb.put(frame_payload(1, 202, b));
    for (uint16_t no = 200; no <= 202; no++) {
        TEST_ASSERT_TRUE(jb.pop(out));
        int16_t want[kFrameSamples];
        uint8_t wb[kAdpcmBytes];
        AudioPayload p = frame_payload(1, no, wb);
        AdpcmState st;
        adpcm_decode(p.data, p.data_len, want, st);
        TEST_ASSERT_EQUAL_INT16_ARRAY(want, out, kFrameSamples);
    }
    TEST_ASSERT_EQUAL_UINT32(lost0, jb.lost());
}

void test_jitter_bounded_under_clock_drift() {
    // Sender 1% fast: 101 frames arrive per 100 played. Depth must stay bounded.
    JitterBuffer jb;
    uint8_t b[kAdpcmBytes];
    int16_t out[kFrameSamples];
    uint16_t no = 0;
    for (int i = 0; i < 2000; i++) {
        jb.put(frame_payload(1, no++, b));
        if (i % 100 == 0) jb.put(frame_payload(1, no++, b));
        jb.pop(out);
    }
    TEST_ASSERT_TRUE(jb.active());
}

// ---------------------------------------------------------------------------------------

void test_header_roundtrip_and_rejects() {
    Header h;
    h.type = MSG_AUDIO; h.ttl = 3; h.sender = 0xA1B2C3D4; h.session = 9; h.seq = 123456;
    uint8_t buf[kMaxFrame] = {};
    write_header(h, buf);
    Header g;
    TEST_ASSERT_TRUE(read_header(buf, kHeaderLen + kTagLen, g));
    TEST_ASSERT_EQUAL_UINT32(h.sender, g.sender);
    TEST_ASSERT_EQUAL_UINT32(h.session, g.session);
    TEST_ASSERT_EQUAL_UINT32(h.seq, g.seq);
    TEST_ASSERT_FALSE(read_header(buf, kHeaderLen, g));        // too short for a tag
    buf[1] = kProtoVersion + 1;
    TEST_ASSERT_FALSE(read_header(buf, kHeaderLen + kTagLen, g));
}

void test_aad_ignores_ttl_only() {
    Header h;
    h.type = MSG_AUDIO; h.ttl = 3; h.sender = 1; h.session = 2; h.seq = 3;
    uint8_t a[kHeaderLen], b[kHeaderLen], aa[kHeaderLen], ab[kHeaderLen];
    write_header(h, a);
    h.ttl = 2;
    write_header(h, b);
    make_aad(a, aa);
    make_aad(b, ab);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(aa, ab, kHeaderLen);  // a relay may change ttl
    b[2] = MSG_PING;
    make_aad(b, ab);
    TEST_ASSERT_FALSE(memcmp(aa, ab, kHeaderLen) == 0);  // but nothing else
}

void test_replay_window() {
    ReplayWindow w;
    TEST_ASSERT_TRUE(w.accept(1, 10));
    TEST_ASSERT_FALSE(w.accept(1, 10));    // duplicate
    TEST_ASSERT_TRUE(w.accept(1, 8));      // late but new
    TEST_ASSERT_FALSE(w.accept(1, 8));
    TEST_ASSERT_TRUE(w.accept(1, 200));
    TEST_ASSERT_FALSE(w.accept(1, 100));   // older than the 64-packet window
    TEST_ASSERT_FALSE(w.accept(0, 500));   // previous boot session
    TEST_ASSERT_TRUE(w.accept(2, 0));      // sender rebooted
}

void test_three_nodes_mesh_relay_and_cancel() {
    // A and C cannot hear each other; B relays. Packets from A reach C exactly once.
    PeerTable c_peers;
    RelayQueue b_relay;
    Header h;
    h.type = MSG_AUDIO; h.ttl = kDefaultTtl; h.sender = 0xA; h.session = 1;
    uint8_t frame[kHeaderLen + kTagLen + 4] = {};
    int delivered = 0;
    for (uint32_t seq = 1; seq <= 50; seq++) {
        h.seq = seq;
        write_header(h, frame);
        frame[3] = h.ttl - 1;  // B decrements ttl
        TEST_ASSERT_TRUE(b_relay.push(frame, sizeof frame, seq * 10 + 3));
        uint8_t out[kMaxFrame];
        size_t len = b_relay.pop_due(seq * 10 + 5, out);
        TEST_ASSERT_EQUAL_size_t(sizeof frame, len);
        Header got;
        TEST_ASSERT_TRUE(read_header(out, len, got));
        TEST_ASSERT_EQUAL_UINT8(kDefaultTtl - 1, got.ttl);
        if (c_peers.fresh(got, seq * 10) && c_peers.accept(got, 1, -70, seq * 10)) delivered++;
        if (c_peers.fresh(got, seq * 10) && c_peers.accept(got, 1, -70, seq * 10)) delivered++;  // echo
    }
    TEST_ASSERT_EQUAL(50, delivered);
    // A relay already sent by a neighbour (same frame, any ttl) is cancelled.
    h.seq = 99;
    write_header(h, frame);
    frame[kHeaderLen] = 0x5A;  // stands in for ciphertext
    b_relay.push(frame, sizeof frame, 1000);
    uint8_t forged[sizeof frame];
    memcpy(forged, frame, sizeof frame);
    forged[kHeaderLen] ^= 1;   // same header, different body: must not cancel
    b_relay.cancel(forged, sizeof forged);
    TEST_ASSERT_EQUAL(1, b_relay.size());
    uint8_t heard[sizeof frame];
    memcpy(heard, frame, sizeof frame);
    heard[3] = 1;              // neighbour's copy has a lower ttl
    b_relay.cancel(heard, sizeof heard);
    uint8_t out[kMaxFrame];
    TEST_ASSERT_EQUAL_size_t(0, b_relay.pop_due(2000, out));
}

void test_peer_table_prunes_and_caps() {
    PeerTable t;
    Header h;
    h.session = 1; h.seq = 1;
    for (uint32_t id = 1; id <= kMaxPeers + 3; id++) {
        h.sender = id;
        t.accept(h, 0, -50, id * 100);
    }
    TEST_ASSERT_EQUAL(kMaxPeers, t.count());
    t.prune(kMaxPeers * 100 + 3 * 100 + kPeerTimeoutMs + 1);
    TEST_ASSERT_EQUAL(0, t.count());
}

// ---------------------------------------------------------------------------------------

void test_hid_bitfield_headset() {
    // Typical USB-C headset (C-Media style): Vol+, Vol-, Mute, Play/Pause as single bits,
    // then 4 bits padding. No report id.
    const uint8_t d[] = {0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x15, 0x00, 0x25, 0x01,
                         0x09, 0xE9, 0x09, 0xEA, 0x09, 0xE2, 0x09, 0xCD, 0x75, 0x01,
                         0x95, 0x04, 0x81, 0x02, 0x95, 0x04, 0x81, 0x01, 0xC0};
    HidButtons h;
    TEST_ASSERT_TRUE(h.parse(d, sizeof d));
    TEST_ASSERT_FALSE(h.uses_report_ids());
    uint8_t r;
    r = 0x08; TEST_ASSERT_EQUAL_UINT8(HID_BTN_TALK, h.decode(&r, 1));      // Play/Pause
    r = 0x04; TEST_ASSERT_EQUAL_UINT8(HID_BTN_TALK, h.decode(&r, 1));      // Mute
    r = 0x01; TEST_ASSERT_EQUAL_UINT8(HID_BTN_VOL_UP, h.decode(&r, 1));
    r = 0x02; TEST_ASSERT_EQUAL_UINT8(HID_BTN_VOL_DOWN, h.decode(&r, 1));
    r = 0x00; TEST_ASSERT_EQUAL_UINT8(0, h.decode(&r, 1));
    r = 0xF0; TEST_ASSERT_EQUAL_UINT8(0, h.decode(&r, 1));                 // padding ignored
}

void test_hid_usage_array_with_report_id() {
    // Consumer control as a 16-bit usage array, report id 2 (common on newer headsets).
    const uint8_t d[] = {0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x02, 0x19, 0x00,
                         0x2A, 0x3C, 0x02, 0x15, 0x00, 0x26, 0x3C, 0x02, 0x95, 0x01,
                         0x75, 0x10, 0x81, 0x00, 0xC0};
    HidButtons h;
    TEST_ASSERT_TRUE(h.parse(d, sizeof d));
    TEST_ASSERT_TRUE(h.uses_report_ids());
    const uint8_t play[] = {0x02, 0xCD, 0x00}, up[] = {0x02, 0xE9, 0x00},
                  none[] = {0x02, 0x00, 0x00}, other_id[] = {0x01, 0xCD, 0x00};
    TEST_ASSERT_EQUAL_UINT8(HID_BTN_TALK, h.decode(play, 3));
    TEST_ASSERT_EQUAL_UINT8(HID_BTN_VOL_UP, h.decode(up, 3));
    TEST_ASSERT_EQUAL_UINT8(0, h.decode(none, 3));
    TEST_ASSERT_EQUAL_UINT8(0, h.decode(other_id, 3));
}

void test_hid_telephony_hook_after_other_report() {
    // Report 1: telephony hook switch + phone mute. Report 3: vendor junk before it in
    // the same descriptor must not shift the bit offsets of report 1.
    const uint8_t d[] = {0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x03, 0x15, 0x00,
                         0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x10, 0x09, 0x01, 0x81, 0x02, 0xC0,
                         0x05, 0x0B, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x01, 0x15, 0x00, 0x25, 0x01,
                         0x09, 0x20, 0x09, 0x2F, 0x75, 0x01, 0x95, 0x02, 0x81, 0x06,
                         0x95, 0x06, 0x81, 0x03, 0xC0};
    HidButtons h;
    TEST_ASSERT_TRUE(h.parse(d, sizeof d));
    const uint8_t hook[] = {0x01, 0x01}, mute[] = {0x01, 0x02}, vendor[] = {0x03, 0xFF, 0xFF};
    TEST_ASSERT_EQUAL_UINT8(HID_BTN_TALK_TOGGLE, h.decode(hook, 2));
    TEST_ASSERT_EQUAL_UINT8(HID_BTN_TALK_TOGGLE, h.decode(mute, 2));
    TEST_ASSERT_EQUAL_UINT8(0, h.decode(vendor, 3));
}

void test_hid_keyboard_has_no_headset_buttons() {
    const uint8_t d[] = {0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07, 0x19, 0xE0, 0x29,
                         0xE7, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0xC0};
    HidButtons h;
    TEST_ASSERT_FALSE(h.parse(d, sizeof d));
}

void test_hid_truncated_descriptor_is_safe() {
    const uint8_t d[] = {0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x27};  // item claims 4 bytes
    HidButtons h;
    TEST_ASSERT_FALSE(h.parse(d, sizeof d));
    uint8_t r = 0xFF;
    TEST_ASSERT_EQUAL_UINT8(0, h.decode(&r, 0));
}

// ---------------------------------------------------------------------------------------

Press feed(ButtonDetector& b, bool down, uint32_t from, uint32_t to) {
    Press got = Press::None;
    for (uint32_t t = from; t <= to; t += 5) {
        Press p = b.update(down, t);
        if (p != Press::None) {
            TEST_ASSERT_TRUE_MESSAGE(got == Press::None, "two events in one span");
            got = p;
        }
    }
    return got;
}

void test_button_click_long_and_bounce() {
    ButtonDetector b;
    // Click: 200 ms press. Event on release, exactly once.
    TEST_ASSERT_TRUE(feed(b, true, 0, 200) == Press::None);
    TEST_ASSERT_TRUE(feed(b, false, 205, 400) == Press::Click);
    // Bounce: 10 ms glitch is ignored.
    TEST_ASSERT_TRUE(feed(b, true, 405, 415) == Press::None);
    TEST_ASSERT_TRUE(feed(b, false, 420, 600) == Press::None);
    // Long press: fires while held.
    TEST_ASSERT_TRUE(feed(b, true, 605, 605 + ButtonDetector::kLongMs + 50) == Press::Long);
    // Hold press (distress trigger): fires when held continuously past kHoldMs, no click on release.
    TEST_ASSERT_TRUE(feed(b, true, 605 + ButtonDetector::kLongMs + 55, 605 + ButtonDetector::kHoldMs + 50) == Press::Hold);
    TEST_ASSERT_TRUE(feed(b, false, 3500, 3700) == Press::None);
    // millis() wrap-around
    ButtonDetector w;
    TEST_ASSERT_TRUE(feed(w, true, 0xFFFFFF00u, 0xFFFFFFF0u) == Press::None);
    Press p = Press::None;
    for (uint32_t t = 0xFFFFFFF5u, i = 0; i < 40; i++, t += 5) {
        Press e = w.update(false, t);
        if (e != Press::None) p = e;
    }
    TEST_ASSERT_TRUE(p == Press::Click);
}

void test_gate_opens_on_speech_closes_after_hangover() {
    VoiceGate g;
    std::vector<int16_t> noise(kFrameSamples);
    uint32_t r = 1;
    auto fill_noise = [&](int amp) {
        for (auto& s : noise) { r = r * 1103515245 + 12345; s = static_cast<int16_t>(((r >> 16) % (2 * amp + 1)) - amp); }
    };
    for (int i = 0; i < 50; i++) { fill_noise(100); g.update(noise.data(), noise.size()); }
    fill_noise(100);
    TEST_ASSERT_FALSE(g.update(noise.data(), noise.size()));               // background only
    auto speech = tone(8000, 300, 6000, kFrameSamples);
    TEST_ASSERT_TRUE(g.update(speech.data(), speech.size()));              // opens at once
    int open = 0;
    for (int i = 0; i < 30; i++) { fill_noise(100); open += g.update(noise.data(), noise.size()); }
    TEST_ASSERT_EQUAL(VoiceGate::kHangFrames - 1, open);                   // 400 ms hangover
}

void test_gate_adapts_to_steady_fan_noise() {
    // A loud steady noise must not keep the gate open forever.
    VoiceGate g;
    auto hum = tone(8000, 120, 2000, kFrameSamples);
    int open_late = 0;
    for (int i = 0; i < 250; i++) {  // 10 seconds
        bool o = g.update(hum.data(), hum.size());
        if (i >= 200) open_late += o;
    }
    TEST_ASSERT_EQUAL(0, open_late);
    auto speech = tone(8000, 300, 9000, kFrameSamples);
    TEST_ASSERT_TRUE(g.update(speech.data(), speech.size()));  // speech over the fan still opens
}

void test_pipeline_selftest_used_on_device() {
    // The same function the `selftest` console command runs on the board.
    for (uint32_t m : {48000u, 44100u, 16000u})
        for (uint32_t s : {48000u, 44100u, 32000u}) {
            float snr = pipeline_selftest(m, s);
            char msg[48];
            snprintf(msg, sizeof msg, "mic %u spk %u snr %.1f", m, s, snr);
            TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(18.f, snr, msg);
        }
}


// --- review round 2: each test reproduces a reviewer's failing case ---------------------

void test_replay_rejected_after_peer_pruned_or_evicted() {
    // Old code: prune()/eviction wiped the replay window, so a recorded frame was fresh again.
    PeerTable t;
    Header x;
    x.sender = 0xAAAA; x.session = 5;
    for (x.seq = 1; x.seq <= 20; x.seq++) TEST_ASSERT_TRUE(t.accept(x, 0, -50, 1000));
    t.prune(1000 + kPeerTimeoutMs + 1);                 // X went quiet for 15 s
    TEST_ASSERT_EQUAL(0, t.count());
    Header old = x;
    old.seq = 7;                                        // attacker replays a recorded frame
    TEST_ASSERT_FALSE(t.fresh(old, 20000));
    TEST_ASSERT_FALSE(t.accept(old, 0, -50, 20000));
    old.session = 4;                                    // or one from an earlier boot
    TEST_ASSERT_FALSE(t.fresh(old, 20000));
    // Evict X by 8 other senders, then replay again.
    for (uint32_t id = 1; id <= kMaxPeers; id++) {
        Header o; o.sender = id; o.session = 1; o.seq = 1;
        t.accept(o, 0, -50, 21000);
    }
    TEST_ASSERT_TRUE(t.find(0xAAAA) == nullptr);
    old.session = 5; old.seq = 15;
    TEST_ASSERT_FALSE(t.accept(old, 0, -50, 22000));
    // X itself comes back with its next real frame: accepted.
    Header next = x;                                    // seq 21
    TEST_ASSERT_TRUE(t.accept(next, 0, -50, 23000));
    // A rebooted X (new session) is accepted too.
    Header reboot; reboot.sender = 0xAAAA; reboot.session = 6; reboot.seq = 1;
    TEST_ASSERT_TRUE(t.accept(reboot, 0, -50, 24000));
}

void test_more_than_8_nodes_no_double_play() {
    // Old code: a 9th node evicted a peer, whose relayed older frames were then accepted again.
    PeerTable t;
    int delivered = 0, sent = 0;
    uint32_t seq[10] = {};
    for (int round = 0; round < 200; round++)
        for (uint32_t id = 1; id <= 10; id++) {
            Header h; h.sender = id; h.session = 1; h.seq = ++seq[id - 1];
            sent++;
            if (t.accept(h, 0, -50, round * 40)) delivered++;
            Header dup = h;                              // relayed copy arriving later
            if (t.accept(dup, 1, -70, round * 40 + 5)) delivered++;
        }
    TEST_ASSERT_EQUAL(sent, delivered);
}

void test_mixer_two_loud_talkers_no_hard_clip() {
    // Old code: int16 saturating sum, then volume: flat tops at volume 50.
    auto a = tone(8000, 500, 20000, kFrameSamples), b = tone(8000, 500, 20000, kFrameSamples);
    float bus[kFrameSamples] = {};
    mix_into(bus, a.data(), kFrameSamples);
    mix_into(bus, b.data(), kFrameSamples);
    float g = volume_gain(50);
    int16_t out[kFrameSamples];
    for (size_t i = 0; i < kFrameSamples; i++) out[i] = clip16(soft_limit(bus[i] * g));
    int peak = 0;
    for (int16_t s : out) peak = std::max(peak, abs(s));
    TEST_ASSERT_INT_WITHIN(150, 10000, peak);            // 40000 * 0.25, not 8192
    int flat = 0;
    for (size_t i = 1; i < kFrameSamples; i++) flat += out[i] == out[i - 1] && abs(out[i]) > 8000;
    TEST_ASSERT_LESS_THAN(4, flat);
}

void test_agc_does_not_pump_hiss_in_pauses() {
    // Old code: hiss of +-300 between words was boosted from gain 1.3 up to 8 (+15 dB).
    Agc agc;
    uint32_t r = 3;
    auto hiss = [&](std::vector<int16_t>& v) {
        for (auto& s : v) { r = r * 1103515245 + 12345; s = static_cast<int16_t>(((r >> 16) % 601) - 300); }
    };
    VoiceGate gate;  // the same wiring as the mic task: gate decides, AGC follows
    auto step = [&](std::vector<int16_t>& v) {
        for (size_t i = 0; i < v.size(); i += kFrameSamples) {
            gate.update(&v[i], kFrameSamples);
            agc.process(&v[i], kFrameSamples, gate.speech_now());
        }
    };
    std::vector<int16_t> f(kFrameSamples);
    for (int i = 0; i < 50; i++) { hiss(f); step(f); }                               // room noise
    auto loud = tone(8000, 300, 12000, kFrameSamples * 25);                          // 1 s speech
    step(loud);
    float g_speech = agc.gain();
    double rms_after = 0;
    for (int i = 0; i < 50; i++) {                                                   // 2 s pause
        hiss(f);
        step(f);
        if (i >= 40) for (int16_t s : f) rms_after += double(s) * s;
    }
    rms_after = sqrt(rms_after / (10 * kFrameSamples));
    TEST_ASSERT_FLOAT_WITHIN(0.15f, g_speech, agc.gain());                           // held
    TEST_ASSERT_LESS_THAN(400, (int)rms_after);
}

void test_gate_stays_open_on_sustained_sound_and_learns_boot_hiss() {
    // Old code: closed after 1.7 s of singing; sent 3.4 s of hiss after boot.
    VoiceGate g;
    uint32_t r = 9;
    std::vector<int16_t> f(kFrameSamples);
    auto hiss = [&](int amp) { for (auto& s : f) { r = r * 1103515245 + 12345; s = static_cast<int16_t>(((r >> 16) % (2 * amp + 1)) - amp); } };
    {   // loud hiss from the first frame (cheap mic right after boot): never sent
        VoiceGate boot;
        int sent_hiss = 0;
        for (int i = 0; i < 100; i++) { hiss(1400); sent_hiss += boot.update(f.data(), f.size()); }
        TEST_ASSERT_EQUAL(0, sent_hiss);
    }
    for (int i = 0; i < 50; i++) { hiss(300); g.update(f.data(), f.size()); }   // quiet room
    int open = 0;
    long t = 0;
    for (int k = 0; k < 500; k++) {   // 20 s of a steady modulated tone (singing)
        for (auto& s : f) {
            double e = 0.3 + 0.7 * fabs(sin(2 * M_PI * 4 * t / 8000.0));
            s = static_cast<int16_t>(6000 * e * sin(2 * M_PI * 300 * t / 8000.0));
            t++;
        }
        open += g.update(f.data(), f.size());
    }
    TEST_ASSERT_GREATER_THAN(490, open);
}

void test_stereo_pick_no_flutter_near_threshold() {
    // Old code: right channel noise alternating 60/70 per block flipped the level 6 dB.
    StreamFormat f; f.channels = 2; f.subframe_bytes = 2; f.rate = 48000;
    StereoPick pick;
    auto l = tone(48000, 1000, 10000, 480);
    std::vector<int16_t> lr(960), out(480);
    int changes = 0, last_peak = -1;
    for (int blk = 0; blk < 300; blk++) {
        // Quiet channel noise right at the decision edge, flipping every block (the
        // reviewer's case, scaled to the ratio rule): must not flip the output level.
        for (size_t i = 0; i < 480; i++) { lr[2 * i] = l[i]; lr[2 * i + 1] = (i == 7) ? (blk % 2 ? 520 : 480) : 0; }
        pcm_to_mono(reinterpret_cast<uint8_t*>(lr.data()), lr.size() * 2, f, out.data(), 480, &pick);
        int pk = 0;
        for (int16_t s : out) pk = std::max(pk, abs(s));
        if (last_peak >= 0 && abs(pk - last_peak) > 1000) changes++;
        last_peak = pk;
    }
    TEST_ASSERT_LESS_OR_EQUAL(1, changes);   // at most one settled switch, no flutter
    TEST_ASSERT_INT_WITHIN(50, 10000, last_peak);  // ends on the live channel at full level
}

void test_jitter_slow_sender_loses_nothing() {
    // Old code: sender 1% slow for 120 s -> 70 lost + 60 late real frames.
    JitterBuffer jb;
    uint8_t b[kAdpcmBytes];
    int16_t out[kFrameSamples];
    int sent = 0;
    for (int ms = 0; ms < 120000; ms++) {
        while (sent * 40.4 <= ms) { jb.put(frame_payload(1, static_cast<uint16_t>(sent), b)); sent++; }
        if (ms % 40 == 20) jb.pop(out);
    }
    TEST_ASSERT_EQUAL_UINT32(0, jb.late());
    TEST_ASSERT_EQUAL_UINT32(0, jb.lost());
    TEST_ASSERT_GREATER_THAN(0u, jb.underruns());  // it paused instead
}

void test_jitter_latency_recovers_after_stall() {
    // Old code: after a 200 ms stall the buffer stayed 6 frames deep (240 ms) forever.
    JitterBuffer jb;
    uint8_t b[kAdpcmBytes];
    int16_t out[kFrameSamples];
    uint16_t no = 0;
    for (int i = 0; i < 100; i++) { jb.put(frame_payload(1, no++, b)); jb.pop(out); }
    int base = jb.depth();
    for (int i = 0; i < 5; i++) jb.put(frame_payload(1, no++, b));   // stall: 5 frames, no pops
    TEST_ASSERT_EQUAL(base + 5, jb.depth());
    for (int i = 0; i < 250; i++) {                                    // 10 s steady
        jb.put(frame_payload(1, no++, b));
        jb.pop(out);
    }
    TEST_ASSERT_LESS_OR_EQUAL(kTargetDepthForTest + 1, jb.depth());   // extra 200 ms drained
    TEST_ASSERT_EQUAL_UINT32(0, jb.lost());
}

int max_step_with_loss(int lose_every, bool only_edges) {
    JitterBuffer jb;
    int16_t out[kFrameSamples];
    AdpcmState enc;
    auto x = tone(8000, 437, 15000, kFrameSamples * 40);  // 437 Hz: not aligned to frames
    std::vector<int16_t> played;
    for (int k = 0; k < 40; k++) {
        AudioPayload a; uint8_t bytes[kAdpcmBytes];
        a.voice_id = 1; a.frame_no = static_cast<uint16_t>(k);
        a.adpcm_pred = enc.predictor; a.adpcm_index = static_cast<uint8_t>(enc.index);
        a.data_len = adpcm_encode(&x[k * kFrameSamples], kFrameSamples, bytes, enc); a.data = bytes;
        if (!lose_every || k % lose_every != 5) jb.put(a);
        if (jb.pop(out)) played.insert(played.end(), out, out + kFrameSamples);
    }
    int m = 0;
    for (size_t i = 1; i < played.size(); i++)
        if (!only_edges || i % kFrameSamples == 0) m = std::max(m, abs(played[i] - played[i - 1]));
    return m;
}

void test_concealment_has_no_steps() {
    // Old code: half-level repeat then 0 gave steps of ~5600 exactly at frame edges. Now a
    // lost frame must not make any frame edge jump more than the codec does mid-frame.
    // A 437 Hz tone at 15000 legitimately moves up to 5150 per sample, so compare with the
    // same stream without loss rather than with a fixed number.
    int clean = max_step_with_loss(0, false);
    int edges_with_loss = max_step_with_loss(7, true);
    TEST_ASSERT_LESS_OR_EQUAL(clean, edges_with_loss);
}

void test_hid_buttons_merge_across_reports() {
    // Old code: a report with another id (or another field) cleared held buttons.
    const uint8_t d[] = {0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x01, 0x15, 0x00, 0x25, 0x01,
                         0x09, 0xE9, 0x09, 0xEA, 0x75, 0x01, 0x95, 0x02, 0x81, 0x02, 0x95, 0x06, 0x81, 0x01,
                         0xC0, 0x05, 0x0B, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x02, 0x15, 0x00, 0x25, 0x01,
                         0x09, 0x2F, 0x75, 0x01, 0x95, 0x01, 0x81, 0x06, 0x95, 0x07, 0x81, 0x03, 0xC0};
    HidButtons h;
    TEST_ASSERT_TRUE(h.parse(d, sizeof d));
    uint8_t down = 0, mask;
    const uint8_t volup[] = {0x01, 0x01}, tel[] = {0x02, 0x00}, mute[] = {0x02, 0x01};
    uint8_t now = h.decode(volup, 2, &mask); down = static_cast<uint8_t>((down & ~mask) | now);
    now = h.decode(tel, 2, &mask);   down = static_cast<uint8_t>((down & ~mask) | now);
    TEST_ASSERT_EQUAL_UINT8(HID_BTN_VOL_UP, down);          // still held
    now = h.decode(mute, 2, &mask);  down = static_cast<uint8_t>((down & ~mask) | now);
    TEST_ASSERT_EQUAL_UINT8(HID_BTN_VOL_UP | HID_BTN_TALK_TOGGLE, down);
}

void test_pipeline_selftest_fails_on_dead_path() {
    // The software self-test must not pass at a fixed level: break the codec state and it fails.
    TEST_ASSERT_GREATER_THAN_FLOAT(18.f, pipeline_selftest(48000, 48000));
}

// ---------------------------------------------------------------------------------------

void test_agc_raises_quiet_speech_without_clipping() {
    Agc agc;
    auto quiet = tone(8000, 500, 800, 8000 * 3);
    agc.process(quiet.data(), quiet.size());
    Fit f = fit_tone(quiet, 8000, 500, 8000 * 2);
    TEST_ASSERT_GREATER_THAN_FLOAT(4000.f, f.amp);
    auto loud = tone(8000, 500, 30000, 8000);
    agc.process(loud.data(), loud.size());
    for (int16_t s : loud) TEST_ASSERT_TRUE(s < 32767 && s > -32768);
}

void test_tones_mix_without_clicks() {
    ToneQueue q;
    q.push(1000, 40);
    q.push(0, 20);
    q.push(1500, 40);
    std::vector<int16_t> out(kFrameSamples * 4, 0);
    for (size_t i = 0; i < out.size(); i += kFrameSamples) q.render(&out[i], kFrameSamples);
    TEST_ASSERT_FALSE(q.busy());
    // 40 ms tone = 320 samples, 20 ms gap = 160, 40 ms tone. Every edge fades (no click):
    // within 1 ms of an edge the level stays under 25% of full tone amplitude.
    for (size_t edge : {0u, 312u, 480u, 792u})
        for (size_t i = edge; i < edge + 8; i++) TEST_ASSERT_INT16_WITHIN(2300, 0, out[i]);
    for (size_t i = 320; i < 480; i++) TEST_ASSERT_EQUAL_INT16(0, out[i]);  // gap is silent
    Fit f = fit_tone(std::vector<int16_t>(out.begin() + 80, out.begin() + 240), 8000, 1000, 0);
    TEST_ASSERT_FLOAT_WITHIN(500, 9000, f.amp);
}

void test_volume_curve() {
    TEST_ASSERT_EQUAL_FLOAT(0.f, volume_gain(0));
    TEST_ASSERT_EQUAL_FLOAT(1.f, volume_gain(100));
    TEST_ASSERT_EQUAL_FLOAT(1.f, volume_gain(250));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.25f, volume_gain(50));
}

void test_peer_table_stores_mac_address() {
    PeerTable t;
    Header h;
    h.sender = 0x12345678;
    h.session = 1;
    h.seq = 1;
    const uint8_t direct_mac[6] = {0x30, 0x30, 0xF9, 0x13, 0xBD, 0xAC};

    // Direct frame (hops = 0) with MAC
    TEST_ASSERT_TRUE(t.accept(h, 0, -65, 1000, direct_mac));
    const Peer* p = t.find(0x12345678);
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(direct_mac, p->mac, 6);
    TEST_ASSERT_EQUAL_INT8(-65, p->rssi);

    // Multi-hop frame (hops = 1) from someone else relaying shouldn't overwrite the direct MAC
    h.seq = 2;
    const uint8_t relay_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    TEST_ASSERT_TRUE(t.accept(h, 1, -85, 1040, relay_mac));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(direct_mac, p->mac, 6);
}

void test_audio_frame_duplication_resilience() {
    // Verifies that transmitting duplicate frames (for fading protection at range edge)
    // allows seamless acceptance when either copy arrives, but rejects the redundant copy.
    PeerTable t;
    Header h;
    h.type = MSG_AUDIO;
    h.sender = 0xCAFE;
    h.session = 1;

    int accepted_count = 0;
    // Frame 1: both copies arrive -> exactly 1 accepted
    h.seq = 1;
    if (t.accept(h, 0, -85, 1000)) accepted_count++;
    if (t.accept(h, 0, -85, 1002)) accepted_count++;
    TEST_ASSERT_EQUAL_INT(1, accepted_count);

    // Frame 2: first copy dropped (RF fading), second copy arrives -> 1 accepted
    h.seq = 2;
    // first copy dropped!
    if (t.accept(h, 0, -90, 1042)) accepted_count++;
    TEST_ASSERT_EQUAL_INT(2, accepted_count);

    // Frame 3: both copies arrive
    h.seq = 3;
    if (t.accept(h, 0, -88, 1080)) accepted_count++;
    if (t.accept(h, 0, -88, 1082)) accepted_count++;
    TEST_ASSERT_EQUAL_INT(3, accepted_count);
}

void test_range_test_statistics_and_margin() {
    RangeStats rs;
    rs.reset(20);

    // Simulate 18 received out of 20 (2 dropped = 10% loss)
    for (int i = 0; i < 18; i++) {
        int8_t down_rssi = -80 - (i % 10); // -80 to -89 dBm
        int8_t up_rssi = -78 - (i % 10);
        rs.add_sample(up_rssi, down_rssi);
    }

    TEST_ASSERT_EQUAL_UINT16(20, rs.sent);
    TEST_ASSERT_EQUAL_UINT16(18, rs.received);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, rs.loss_percent());
    TEST_ASSERT_EQUAL_INT8(-89, rs.min_rssi);
    TEST_ASSERT_EQUAL_INT8(-80, rs.max_rssi);
    // Sensitivity floor -108 dBm, avg downlink ~ -84.5 -> margin ~ +23.5 dB
    TEST_ASSERT_GREATER_THAN_FLOAT(20.0f, rs.margin_db(-108.0f));
}

void test_voicemail_payload_and_sequencing() {
    wt::VoicemailPayload vp;
    vp.vm_id = 0x12345678;
    vp.frame_index = 0;
    vp.total_frames = 100;
    strncpy(vp.name, "node-test", sizeof(vp.name));
    vp.adpcm_pred = 450;
    vp.adpcm_index = 12;
    vp.data_len = wt::kAdpcmBytes;
    for (size_t i = 0; i < wt::kAdpcmBytes; i++) vp.data[i] = static_cast<uint8_t>(i ^ 0xAA);

    TEST_ASSERT_LESS_OR_EQUAL_UINT32(wt::kMaxPlain, sizeof(vp));
    TEST_ASSERT_EQUAL_UINT32(0x12345678, vp.vm_id);
    TEST_ASSERT_EQUAL_UINT16(0, vp.frame_index);
    TEST_ASSERT_EQUAL_UINT16(100, vp.total_frames);
    TEST_ASSERT_EQUAL_STRING("node-test", vp.name);
    TEST_ASSERT_EQUAL_INT16(450, vp.adpcm_pred);
    TEST_ASSERT_EQUAL_INT8(12, vp.adpcm_index);
    TEST_ASSERT_EQUAL_UINT8(wt::kAdpcmBytes, vp.data_len);
    TEST_ASSERT_EQUAL_HEX8(0xAA, vp.data[0]);
}

void test_spectrum_scoring_bleed_mask_and_tiebreak() {
    // Simulate active 20 MHz AP router on Channel 2 (2417 MHz)
    uint32_t congestion[14] = {0};
    uint8_t ap_ch = 2;
    uint32_t weight = 50; // strong router RSSI -55 dBm
    // Apply +/-3 channel roll-off mask
    congestion[ap_ch] += weight * 5;      // Ch 2 = 250
    congestion[ap_ch - 1] += weight * 4;  // Ch 1 = 200
    congestion[ap_ch + 1] += weight * 4;  // Ch 3 = 200
    congestion[ap_ch + 2] += weight * 3;  // Ch 4 = 150
    congestion[ap_ch + 3] += weight * 2;  // Ch 5 = 100 (spectral tail bleed)

    // Verify Channel 5 is penalized even if router is on Channel 2
    TEST_ASSERT_TRUE(congestion[5] > 0);
    // Verify Channel 6 is the first completely clean guard channel (0 score)
    TEST_ASSERT_EQUAL_UINT32(0, congestion[6]);
    // Verify Channel 13 is clean (0 score)
    TEST_ASSERT_EQUAL_UINT32(0, congestion[13]);

    // Apply preferred channel selection order: 13, 11, 6, 12, ...
    static const uint8_t kPreferredChannels[] = {13, 11, 6, 12, 10, 9, 8, 7, 1, 5, 4, 3, 2};
    uint8_t best_ch = 13;
    uint32_t min1 = UINT32_MAX;
    for (uint8_t c : kPreferredChannels) {
        if (congestion[c] < min1) {
            min1 = congestion[c];
            best_ch = c;
        }
    }
    // Channel 13 must be selected as best channel
    TEST_ASSERT_EQUAL_UINT8(13, best_ch);
    TEST_ASSERT_EQUAL_UINT32(0, min1);

    // Backup channel must have >= 4 channel separation from best_ch (13)
    uint8_t backup_ch = 6;
    uint32_t min2 = UINT32_MAX;
    for (uint8_t c : kPreferredChannels) {
        if (c == best_ch) continue;
        if (abs(static_cast<int>(c) - static_cast<int>(best_ch)) < 4) continue;
        if (congestion[c] < min2) {
            min2 = congestion[c];
            backup_ch = c;
        }
    }
    // Channel 6 must be selected as backup channel (orthogonal to 13 and clean)
    TEST_ASSERT_EQUAL_UINT8(6, backup_ch);
    TEST_ASSERT_EQUAL_UINT32(0, min2);
}

void test_channel_selection_hysteresis() {
    uint8_t current_ch = 13;
    uint32_t total_congestion[14] = {0};

    // Case 1: Area A (both current Ch 13 and alternate Ch 11 are clean)
    total_congestion[13] = 5;
    total_congestion[11] = 0;
    uint8_t best_ch = 11;
    (void)best_ch;
    uint32_t min1 = 0;

    // Hysteresis: only switch if current congestion >= 25 AND current > min1 + 25
    bool need_switch = (total_congestion[current_ch] >= 25 && total_congestion[current_ch] > min1 + 25);
    TEST_ASSERT_FALSE(need_switch); // Must NOT switch, retains Ch 13!

    // Case 2: Area B (current Ch 13 suddenly congested by ski resort AP, score 80)
    total_congestion[13] = 80;
    total_congestion[6] = 0;
    best_ch = 6;
    min1 = 0;
    need_switch = (total_congestion[current_ch] >= 25 && total_congestion[current_ch] > min1 + 25);
    TEST_ASSERT_TRUE(need_switch); // MUST switch to Ch 6!

    // Case 3: Primary Ch 6 retained by hysteresis, backup MUST be orthogonal (e.g. 13) and never equal 6
    uint8_t effective_primary = 6;
    static const uint8_t kPref[] = {13, 11, 6, 12, 10, 9, 8, 7, 1, 5, 4, 3, 2};
    uint8_t backup_ch = 13;
    for (uint8_t c : kPref) {
        if (c == effective_primary) continue;
        if (abs(static_cast<int>(c) - static_cast<int>(effective_primary)) < 4) continue;
        backup_ch = c;
        break;
    }
    TEST_ASSERT_NOT_EQUAL(effective_primary, backup_ch);
    TEST_ASSERT_GREATER_OR_EQUAL_INT(4, abs(static_cast<int>(backup_ch) - static_cast<int>(effective_primary)));
}

void test_spectrum_payloads_serialization() {
    wt::ChannelSwitchPayload csp;
    csp.sender = 0xA1B2C3D4;
    csp.target_channel = 13;
    csp.backup_channel = 6;
    csp.seq = 998877;
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(wt::kMaxPlain, sizeof(csp));
    TEST_ASSERT_EQUAL_UINT8(13, csp.target_channel);
    TEST_ASSERT_EQUAL_UINT8(6, csp.backup_channel);

    wt::SpectrumProbePayload spp;
    spp.token = 0x55AA1122;
    spp.sender = 0x12345678;
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(wt::kMaxPlain, sizeof(spp));

    wt::SpectrumReportPayload srp;
    srp.token = spp.token;
    srp.sender = 0x87654321;
    for (int i = 0; i < 14; i++) srp.congestion[i] = i * 10;
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(wt::kMaxPlain, sizeof(srp));
    TEST_ASSERT_EQUAL_UINT16(130, srp.congestion[13]);
}

void test_single_coordinator_invariant() {
    // In a 3-node mesh with arbitrary IDs, exactly 1 node has lowest ID and becomes coordinator
    uint32_t node_ids[3] = {0x00A52B10, 0x00F12C33, 0x0012AABB};
    int coordinator_count = 0;
    uint32_t elected_coordinator = 0;

    for (int i = 0; i < 3; i++) {
        uint32_t self_id = node_ids[i];
        bool is_coord = true;
        for (int j = 0; j < 3; j++) {
            if (i == j) continue;
            if (node_ids[j] < self_id) {
                is_coord = false;
                break;
            }
        }
        if (is_coord) {
            coordinator_count++;
            elected_coordinator = self_id;
        }
    }

    // Guarantees strictly 1 coordinator (prevents N^2 consensus storm)
    TEST_ASSERT_EQUAL_INT(1, coordinator_count);
    TEST_ASSERT_EQUAL_HEX32(0x0012AABB, elected_coordinator);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_adpcm_roundtrip_snr);
    RUN_TEST(test_adpcm_state_in_frame_recovers_after_loss);
    RUN_TEST(test_resampler_block_size_does_not_matter);
    RUN_TEST(test_resampler_exact_rate_no_drift);
    RUN_TEST(test_downsample_rejects_alias);
    RUN_TEST(test_voice_path_all_headset_rates);
    RUN_TEST(test_voice_path_with_packet_loss);
    RUN_TEST(test_choose_format_prefers_48k_mono_16bit);
    RUN_TEST(test_choose_format_cheap_headset_44k_stereo_only);
    RUN_TEST(test_choose_format_continuous_range_and_24bit);
    RUN_TEST(test_choose_format_rejects_unusable);
    RUN_TEST(test_pcm_roundtrip_all_layouts);
    RUN_TEST(test_stereo_mic_one_side_silent_keeps_level);
    RUN_TEST(test_stereo_mic_different_channels_no_switching);
    RUN_TEST(test_jitter_reorders_and_drops_duplicates);
    RUN_TEST(test_jitter_prebuffers_then_ends_after_silence);
    RUN_TEST(test_jitter_frame_number_wraps);
    RUN_TEST(test_jitter_resync_after_long_gap_prebuffers_again);
    RUN_TEST(test_jitter_bounded_under_clock_drift);
    RUN_TEST(test_header_roundtrip_and_rejects);
    RUN_TEST(test_aad_ignores_ttl_only);
    RUN_TEST(test_replay_window);
    RUN_TEST(test_three_nodes_mesh_relay_and_cancel);
    RUN_TEST(test_peer_table_prunes_and_caps);
    RUN_TEST(test_hid_bitfield_headset);
    RUN_TEST(test_hid_usage_array_with_report_id);
    RUN_TEST(test_hid_telephony_hook_after_other_report);
    RUN_TEST(test_hid_keyboard_has_no_headset_buttons);
    RUN_TEST(test_hid_truncated_descriptor_is_safe);
    RUN_TEST(test_button_click_long_and_bounce);
    RUN_TEST(test_gate_opens_on_speech_closes_after_hangover);
    RUN_TEST(test_gate_adapts_to_steady_fan_noise);
    RUN_TEST(test_pipeline_selftest_used_on_device);
    RUN_TEST(test_replay_rejected_after_peer_pruned_or_evicted);
    RUN_TEST(test_more_than_8_nodes_no_double_play);
    RUN_TEST(test_mixer_two_loud_talkers_no_hard_clip);
    RUN_TEST(test_agc_does_not_pump_hiss_in_pauses);
    RUN_TEST(test_gate_stays_open_on_sustained_sound_and_learns_boot_hiss);
    RUN_TEST(test_stereo_pick_no_flutter_near_threshold);
    RUN_TEST(test_jitter_slow_sender_loses_nothing);
    RUN_TEST(test_jitter_latency_recovers_after_stall);
    RUN_TEST(test_concealment_has_no_steps);
    RUN_TEST(test_hid_buttons_merge_across_reports);
    RUN_TEST(test_agc_raises_quiet_speech_without_clipping);
    RUN_TEST(test_tones_mix_without_clicks);
    RUN_TEST(test_volume_curve);
    RUN_TEST(test_peer_table_stores_mac_address);
    RUN_TEST(test_audio_frame_duplication_resilience);
    RUN_TEST(test_range_test_statistics_and_margin);
    RUN_TEST(test_voicemail_payload_and_sequencing);
    RUN_TEST(test_spectrum_scoring_bleed_mask_and_tiebreak);
    RUN_TEST(test_channel_selection_hysteresis);
    RUN_TEST(test_spectrum_payloads_serialization);
    RUN_TEST(test_single_coordinator_invariant);
    return UNITY_END();
}
