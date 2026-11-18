#ifndef MESSAGES_H
#define MESSAGES_H

#include <cstdint>
#include <cstring>

// Protocol Identifiers
#define ESPNOW_FRAME_MAGIC_0    0xAA
#define ESPNOW_FRAME_MAGIC_1    0x55
#define ESPNOW_PAYLOAD_MAGIC_0  0x57 // 'W'
#define ESPNOW_PAYLOAD_MAGIC_1  0x4B // 'K'

// ============================================================
// Message Types (transmitted encrypted over ESP-NOW)
// ============================================================
enum MsgType : uint8_t {
    MSG_PING           = 0x01,   // Ping discovery request
    MSG_PONG           = 0x02,   // Pong discovery reply
    MSG_PTT_START      = 0x03,   // Voice stream start (User Joined)
    MSG_PTT_STOP       = 0x04,   // Voice stream stop (User Left)
    MSG_AUDIO          = 0x05,   // AES-128 Encrypted Voice Audio Chunk (50ms ADPCM)
    MSG_CHANNEL_BEACON = 0x06    // Discovery & channel sync beacon
};

// ============================================================
// ESP-NOW Wire Message Structure (Exactly 250 bytes)
// Wire Layout:
//   magic:        2 bytes  (0xAA, 0x55 frame marker)
//   type:         1 byte   (MsgType enum)
//   seq:          1 byte   (Sequence counter 0-255)
//   channel:      1 byte   (Operating RF channel 1-13)
//   ttl:          1 byte   (Time-To-Live hop limit)
//   hops:         1 byte   (Traversed hop counter)
//   sender_mac:   6 bytes  (Origin device MAC)
//   target_mac:   6 bytes  (Target peer MAC or FF:FF:FF:FF:FF:FF)
//   sender_name: 10 bytes  (Human readable node callsign)
//   iv:           8 bytes  (AES-128 random Initialization Vector)
//   payload_len:  2 bytes  (Encrypted payload byte count)
//   payload:    211 bytes  (AES-128 Encrypted payload)
//   ─────────────────────────────────────────────────────────
//   TOTAL:      250 bytes  (Exact ESP-NOW hardware packet limit)
// ============================================================

#define ESPNOW_MSG_MAX_PAYLOAD  211
#define ESPNOW_MSG_HEADER_SIZE  (250 - ESPNOW_MSG_MAX_PAYLOAD) // 39 bytes

#pragma pack(push, 1)
struct EspNowMessage {
    uint8_t   magic[2];                         // 0xAA, 0x55
    uint8_t   type;                             // MsgType
    uint8_t   seq;                              // Sequence number
    uint8_t   channel;                          // WiFi channel (1-13)
    uint8_t   ttl;                              // Time-To-Live (Hop limit: 1-3)
    uint8_t   hops;                             // Hop count traversed (0 = direct)
    uint8_t   sender_mac[6];                    // Origin MAC
    uint8_t   target_mac[6];                    // Target MAC
    char      sender_name[10];                  // Callsign (e.g. Peto)
    uint8_t   iv[8];                            // AES-128 Nonce / IV
    uint16_t  payload_len;                      // Encrypted payload size
    uint8_t   payload[ESPNOW_MSG_MAX_PAYLOAD];  // Encrypted ciphertext
};
#pragma pack(pop)

static_assert(sizeof(EspNowMessage) == 250, "EspNowMessage must be exactly 250 bytes");

// ============================================================
// Payload Internal Layouts (Decrypted)
// ============================================================

#define AUDIO_CHUNK_HEADER_SIZE 6 // 2 bytes magic + 4 bytes header
#define AUDIO_CHUNK_DATA_MAX    (ESPNOW_MSG_MAX_PAYLOAD - AUDIO_CHUNK_HEADER_SIZE) // 205 bytes

enum AudioMime : uint8_t {
    AUDIO_MIME_RAW_PCM = 1,
    AUDIO_MIME_ADPCM   = 3
};

#pragma pack(push, 1)
struct PingPayload {
    uint8_t  magic[2];       // 'W', 'K'
    uint32_t pingToken;      // millis() timestamp / random token
};

struct PongPayload {
    uint8_t  magic[2];       // 'W', 'K'
    uint32_t pingToken;      // Echoes sender's ping token
    int8_t   rssi;           // Receiver observed RSSI
};

struct BeaconPayload {
    uint8_t  magic[2];       // 'W', 'K'
    uint8_t  channel;        // Operating channel
    uint32_t uptimeSec;      // Device uptime
};
#pragma pack(pop)

#endif // MESSAGES_H
