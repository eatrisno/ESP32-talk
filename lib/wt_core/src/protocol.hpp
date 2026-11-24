#pragma once
// Over-the-air packet format. Every frame is authenticated with AES-128-CCM. The header
// (with ttl zeroed, so relays may decrement it) is the associated data, the nonce is
// sender|session|seq. session is a boot counter in NVS that starts at a random value
// after a flash erase, so a nonce is not reused (see next_session() in main.cpp).
//
//   off  size  field
//   0    1     magic 'W'
//   1    1     version (kProtoVersion)
//   2    1     type (MsgType)
//   3    1     ttl (hops left)
//   4    4     sender id (low 4 bytes of the sender MAC)
//   8    4     session (sender boot counter, random start)
//   12   4     seq (per session, strictly increasing)
//   16   n     ciphertext
//   16+n 8     CCM tag
//
// Audio payload (plaintext): voice_id(1) frame_no(2) adpcm_pred(2) adpcm_index(1) data(N)
#include <cstddef>
#include <cstdint>

namespace wt {

constexpr uint8_t kMagic = 'W';
constexpr uint8_t kProtoVersion = 3;
constexpr size_t  kHeaderLen = 16;
constexpr size_t  kTagLen = 8;
constexpr size_t  kNonceLen = 12;
constexpr size_t  kMaxFrame = 250;  // ESP-NOW v1 payload limit
constexpr size_t  kMaxPlain = kMaxFrame - kHeaderLen - kTagLen;  // 226

constexpr uint32_t kFrameMs = 40;                              // voice frame length
constexpr size_t   kFrameSamples = 8000 * kFrameMs / 1000;     // 320 @ 8 kHz
constexpr size_t   kAdpcmBytes = kFrameSamples / 2;            // 160
constexpr size_t   kAudioHdr = 6;
static_assert(kAudioHdr + kAdpcmBytes <= kMaxPlain, "audio frame does not fit");

enum MsgType : uint8_t {
    MSG_BEACON = 1,   // presence + name, every few seconds
    MSG_JOIN   = 2,   // started talking
    MSG_LEAVE  = 3,   // stopped talking
    MSG_AUDIO      = 4,
    MSG_PING       = 5,
    MSG_PONG       = 6,
    MSG_RANGE_REQ  = 7,
    MSG_RANGE_RESP = 8,
    MSG_DISTRESS   = 9,
    MSG_VOICEMAIL  = 10,
    MSG_CHANNEL_SWITCH  = 11,
    MSG_SPECTRUM_PROBE  = 12,
    MSG_SPECTRUM_REPORT = 13,
    MSG_PAIR_BEACON     = 14,
    MSG_PAIR_ACK        = 15,
};

#pragma pack(push, 1)
struct PairPayload {
    uint32_t sender;
    uint8_t  mac[6];
    char     name[16];
    uint8_t  expected_group;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct ChannelSwitchPayload {
    uint32_t sender;
    uint8_t  target_channel;
    uint8_t  backup_channel; // 0 if none, or 1..13
    uint32_t seq;
};

struct SpectrumProbePayload {
    uint32_t token;
    uint32_t sender;
};

struct SpectrumReportPayload {
    uint32_t token;
    uint32_t sender;
    uint16_t congestion[14]; // congestion scores for channels 1..13
};

struct DistressPayload {
    uint32_t sender;
    uint8_t  active; // 1 = SOS active, 0 = all-clear / cancelled
    char     name[12];
};

struct VoicemailPayload {
    uint32_t vm_id;        // unique token for this voice message
    uint16_t frame_index;  // 0 .. total_frames - 1
    uint16_t total_frames; // total count of frames in this message
    char     name[12];     // sender node name
    int16_t  adpcm_pred;   // IMA ADPCM state
    int8_t   adpcm_index;
    uint8_t  data_len;     // bytes of ADPCM data (usually 160)
    uint8_t  data[kAdpcmBytes]; // 160 bytes
};

struct PiggybackBeacon {
    uint8_t  flags;          // Bit 0: talking/PTT, Bit 1: SOS, Bit 2: Voicemail inbox
    uint8_t  primary_ch;     // Consensus primary channel
    uint8_t  backup_ch;      // Consensus backup channel
    uint16_t ack_seq;        // Application ACK: sequence number of last frame from peer
    int8_t   uplink_rssi;    // Piggybacked Uplink RSSI of peer
    uint32_t relay_id;       // Piggybacked Relay ID (0 if direct, or ID of relay hop)
    char     name[12];       // Node name
};
#pragma pack(pop)
static_assert(sizeof(VoicemailPayload) <= kMaxPlain, "VoicemailPayload exceeds max plain size");
static_assert(sizeof(PiggybackBeacon) <= kMaxPlain, "PiggybackBeacon exceeds max plain size");

struct Header {
    uint8_t  type = 0;
    uint8_t  ttl = 0;
    uint32_t sender = 0;
    uint32_t session = 0;
    uint32_t seq = 0;
};

void   write_header(const Header& h, uint8_t* out);
// Returns false on bad magic/version/length.
bool   read_header(const uint8_t* in, size_t len, Header& h);
// CCM nonce and associated data for a header.
void   make_nonce(const Header& h, uint8_t nonce[kNonceLen]);
void   make_aad(const uint8_t* header, uint8_t aad[kHeaderLen]);

struct AudioPayload {
    uint8_t  voice_id = 0;
    uint16_t frame_no = 0;
    int16_t  adpcm_pred = 0;
    uint8_t  adpcm_index = 0;
    const uint8_t* data = nullptr;
    size_t   data_len = 0;
};

size_t write_audio(const AudioPayload& a, uint8_t* out, size_t max);
bool   read_audio(const uint8_t* in, size_t len, AudioPayload& a);

// Replay filter per sender: rejects older sessions and seq numbers already seen in the
// last 64 of the current session. Call only after the packet authenticated.
class ReplayWindow {
public:
    bool accept(uint32_t session, uint32_t seq);
    // Same test without recording (to decide whether a packet is worth decrypting).
    bool fresh(uint32_t session, uint32_t seq) const;
    void reset() { session_ = 0; top_ = 0; bits_ = 0; init_ = false; }

private:
    uint32_t session_ = 0;
    uint32_t top_ = 0;
    uint64_t bits_ = 0;
    bool init_ = false;
};

}  // namespace wt
