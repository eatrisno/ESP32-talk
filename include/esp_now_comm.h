#ifndef ESP_NOW_COMM_H
#define ESP_NOW_COMM_H

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "config.h"
#include "messages.h"
#include "security.h"

// Callback type invoked when an authentic, decrypted message is received
typedef void (*EspNowDecryptedReceiveCb)(const EspNowMessage& header,
                                        const uint8_t* decryptedPayload,
                                        uint16_t decryptedLen,
                                        int rssi);

struct PeerNode {
    uint8_t  mac[6];
    char     name[12];
    int      rssi;
    uint32_t lastSeenMs;
    bool     active;
};

class EspNowComm {
public:
    static EspNowComm& getInstance();

    /// Initialize ESP-NOW in pure STA mode with Long-Range protocol
    bool init(uint8_t channel, bool enableLR = true);

    /// Dynamic channel switching (for auto-search and user channels)
    bool setChannel(uint8_t channel);
    uint8_t getChannel() const { return _channel; }

    /// High-Level Encrypted Transmissions
    bool sendPing(uint32_t pingToken);
    bool sendPong(const uint8_t* targetMac, uint32_t pingToken, int8_t rssi);
    bool sendPttStart();
    bool sendPttStop(uint16_t totalAudioBytes = 0);
    bool sendAudioChunk(uint8_t msgId, uint8_t chunkIdx, uint8_t totalChunks,
                        uint8_t mimeType, const uint8_t* data, uint16_t len);
    bool sendChannelBeacon(uint8_t channel, uint32_t uptimeSec);

    /// Raw encrypted message broadcast
    bool sendMessage(const EspNowMessage& msg);

    /// Callback configuration
    void setReceiveCallback(EspNowDecryptedReceiveCb cb);

    /// Peer discovery and distance tracking
    void updatePeer(const uint8_t* mac, const char* name, int rssi);
    void pruneStalePeers();
    int  getActivePeers(PeerNode* outList, int maxList);
    uint32_t getLastPeerSeenMs() const { return _lastPeerSeenMs; }

    /// Accessors
    const uint8_t* getMac()        const { return _mac; }
    String         getMacString()  const;
    const char*    getDeviceName() const { return _deviceName; }
    void           setDeviceName(const char* name);
    uint32_t       getTxCount()    const { return _txCount; }
    uint32_t       getRxCount()    const { return _rxCount; }
    uint32_t       getRelayCount() const { return _relayCount; }
    int8_t         getLastRssi()   const;

    /// Automatic Mesh deduplication
    bool isDuplicate(const uint8_t* mac, uint8_t seq, uint8_t type);
    void markSeen(const uint8_t* mac, uint8_t seq, uint8_t type);

private:
    EspNowComm() = default;

    static void onReceive(const uint8_t* mac, const uint8_t* data, int len);
    static void onSend(const uint8_t* mac, esp_now_send_status_t status);

    uint8_t  _mac[6]         = {0};
    char     _deviceName[10] = {0};
    uint8_t  _channel        = 1;
    uint8_t  _seq            = 0;
    uint32_t _txCount        = 0;
    uint32_t _rxCount        = 0;
    uint32_t _relayCount     = 0;
    uint32_t _lastPeerSeenMs = 0;

    struct DedupEntry {
        uint8_t  mac[6];
        uint8_t  seq;
        uint8_t  type;
        uint32_t timestampMs;
    };
    static const int DEDUP_CACHE_SIZE = 64;
    DedupEntry _dedupCache[DEDUP_CACHE_SIZE] = {};
    uint8_t    _dedupIdx = 0;

    PeerNode _peers[MAX_PEERS] = {};
    EspNowDecryptedReceiveCb _receiveCb = nullptr;
};

#endif // ESP_NOW_COMM_H
