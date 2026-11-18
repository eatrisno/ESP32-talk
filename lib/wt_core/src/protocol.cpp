#include "protocol.hpp"

#include <cstring>

namespace wt {
namespace {

void put32(uint8_t* p, uint32_t v) {
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}
uint32_t get32(const uint8_t* p) {
    return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

}  // namespace

void write_header(const Header& h, uint8_t* out) {
    out[0] = kMagic;
    out[1] = kProtoVersion;
    out[2] = h.type;
    out[3] = h.ttl;
    put32(out + 4, h.sender);
    put32(out + 8, h.session);
    put32(out + 12, h.seq);
}

bool read_header(const uint8_t* in, size_t len, Header& h) {
    if (len < kHeaderLen + kTagLen || len > kMaxFrame) return false;
    if (in[0] != kMagic || in[1] != kProtoVersion) return false;
    h.type = in[2];
    h.ttl = in[3];
    h.sender = get32(in + 4);
    h.session = get32(in + 8);
    h.seq = get32(in + 12);
    return true;
}

void make_nonce(const Header& h, uint8_t nonce[kNonceLen]) {
    put32(nonce, h.sender);
    put32(nonce + 4, h.session);
    put32(nonce + 8, h.seq);
}

void make_aad(const uint8_t* header, uint8_t aad[kHeaderLen]) {
    memcpy(aad, header, kHeaderLen);
    aad[3] = 0;  // ttl is the only field a relay changes
}

size_t write_audio(const AudioPayload& a, uint8_t* out, size_t max) {
    if (kAudioHdr + a.data_len > max) return 0;
    out[0] = a.voice_id;
    out[1] = a.frame_no;
    out[2] = a.frame_no >> 8;
    out[3] = static_cast<uint16_t>(a.adpcm_pred);
    out[4] = static_cast<uint16_t>(a.adpcm_pred) >> 8;
    out[5] = a.adpcm_index;
    memcpy(out + kAudioHdr, a.data, a.data_len);
    return kAudioHdr + a.data_len;
}

bool read_audio(const uint8_t* in, size_t len, AudioPayload& a) {
    if (len < kAudioHdr + 1) return false;
    a.voice_id = in[0];
    a.frame_no = static_cast<uint16_t>(in[1] | (in[2] << 8));
    a.adpcm_pred = static_cast<int16_t>(in[3] | (in[4] << 8));
    a.adpcm_index = in[5];
    if (a.adpcm_index > 88) return false;
    a.data = in + kAudioHdr;
    a.data_len = len - kAudioHdr;
    return true;
}

bool ReplayWindow::fresh(uint32_t session, uint32_t seq) const {
    if (!init_ || session > session_) return true;
    if (session < session_) return false;
    if (seq > top_) return true;
    uint32_t back = top_ - seq;
    return back < 64 && !(bits_ & (1ULL << back));
}

bool ReplayWindow::accept(uint32_t session, uint32_t seq) {
    if (!fresh(session, seq)) return false;
    if (!init_ || session > session_) {
        init_ = true;
        session_ = session;
        top_ = seq;
        bits_ = 1;
        return true;
    }
    if (seq > top_) {
        uint32_t shift = seq - top_;
        bits_ = shift >= 64 ? 1 : (bits_ << shift) | 1;
        top_ = seq;
        return true;
    }
    uint32_t back = top_ - seq;
    if (back >= 64) return false;
    uint64_t m = 1ULL << back;
    if (bits_ & m) return false;
    bits_ |= m;
    return true;
}

}  // namespace wt
