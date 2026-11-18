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
#include "voice_gate.hpp"

using namespace wt;

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
    pcm_to_mono(reinterpret_cast<uint8_t*>(lr.data()), lr.size() * 2, f, out, 480);
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
        if (c_peers.fresh(got) && c_peers.accept(got, 1, -70, seq * 10)) delivered++;
        if (c_peers.fresh(got) && c_peers.accept(got, 1, -70, seq * 10)) delivered++;  // echo
    }
    TEST_ASSERT_EQUAL(50, delivered);
    // A relay already sent by a neighbour is cancelled.
    h.seq = 99;
    write_header(h, frame);
    b_relay.push(frame, sizeof frame, 1000);
    b_relay.cancel(0xA, 1, 99);
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
    TEST_ASSERT_EQUAL_UINT8(HID_BTN_TALK, h.decode(hook, 2));
    TEST_ASSERT_EQUAL_UINT8(HID_BTN_TALK, h.decode(mute, 2));
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
    // Long press: fires while held, no click on release.
    TEST_ASSERT_TRUE(feed(b, true, 605, 605 + ButtonDetector::kLongMs + 50) == Press::Long);
    TEST_ASSERT_TRUE(feed(b, false, 2300, 2500) == Press::None);
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
    RUN_TEST(test_agc_raises_quiet_speech_without_clipping);
    RUN_TEST(test_tones_mix_without_clicks);
    RUN_TEST(test_volume_curve);
    return UNITY_END();
}
