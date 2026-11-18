#include "esp_now_comm.h"

// Singleton
EspNowComm& EspNowComm::getInstance() {
    static EspNowComm instance;
    return instance;
}

int8_t EspNowComm::getLastRssi() const {
    return -60;
}

bool EspNowComm::init(uint8_t channel, bool enableLR) {
    _channel = channel;
    WiFi.macAddress(_mac);

    if (_deviceName[0] == '\0') {
        snprintf(_deviceName, sizeof(_deviceName), "WT-%02X%02X", _mac[4], _mac[5]);
    }

    Serial.printf("[ESP-NOW] Node MAC: %s\n", getMacString().c_str());
    Serial.printf("[ESP-NOW] Device Name: %s\n", _deviceName);

    // 1. Strictly Pure Station Mode
    WiFi.mode(WIFI_STA);

    // 2. Disable WiFi Modem Sleep for 0% packet loss
    esp_wifi_set_ps(WIFI_PS_NONE);

    // 3. Configure RF Protocols
    if (enableLR) {
        esp_wifi_set_protocol(WIFI_IF_STA,
            WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
            WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
        Serial.println("[ESP-NOW] Long Range Mode Active (WIFI_PROTOCOL_LR Enabled)");
    } else {
        esp_wifi_set_protocol(WIFI_IF_STA,
            WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    }

    // 4. Maximum Legal Output Power (20 dBm = 100mW)
    esp_wifi_set_max_tx_power(ESPNOW_TX_POWER);
    int8_t curPower = 0;
    esp_wifi_get_max_tx_power(&curPower);
    Serial.printf("[ESP-NOW] RF Max TX Power: %d (%.2f dBm)\n", curPower, curPower * 0.25);

    // 5. Lock interface to initial channel
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    // 6. Initialize ESP-NOW subsystem
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] ERROR: esp_now_init() failed!");
        return false;
    }

    // 7. Lock RF PHY Rate to 6 Mbps OFDM (fast 340 µs airtime per chunk)
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_6M);
    Serial.println("[ESP-NOW] PHY Rate locked to 6 Mbps OFDM (Airtime: 340 µs/pkt)");

    // 8. Register callbacks
    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSend);

    // 9. Register broadcast peer (FF:FF:FF:FF:FF:FF)
    esp_now_peer_info_t peerInfo = {};
    memset(peerInfo.peer_addr, 0xFF, 6);
    peerInfo.channel = channel;
    peerInfo.encrypt = false;
    peerInfo.ifidx   = WIFI_IF_STA;

    esp_err_t err = esp_now_add_peer(&peerInfo);
    if (err != ESP_OK) {
        Serial.printf("[ESP-NOW] ERROR: add broadcast peer failed: %s\n", esp_err_to_name(err));
        return false;
    }

    // 10. Initialize AES-128 Hardware Security
    Security::getInstance().init();

    Serial.printf("[ESP-NOW] Initialized on channel %d (broadcast + AES-128)\n", channel);
    return true;
}

bool EspNowComm::setChannel(uint8_t channel) {
    if (channel < CHANNEL_MIN || channel > CHANNEL_MAX) return false;
    _channel = channel;

    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_del_peer(bcast);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, bcast, 6);
    peerInfo.channel = channel;
    peerInfo.encrypt = false;
    peerInfo.ifidx   = WIFI_IF_STA;

    esp_now_add_peer(&peerInfo);
    return true;
}

void EspNowComm::setDeviceName(const char* name) {
    if (!name || strlen(name) == 0) return;
    strncpy(_deviceName, name, sizeof(_deviceName) - 1);
    _deviceName[sizeof(_deviceName) - 1] = '\0';
}

String EspNowComm::getMacString() const {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             _mac[0], _mac[1], _mac[2], _mac[3], _mac[4], _mac[5]);
    return String(buf);
}

// Automatic Mesh Deduplication Implementation
bool EspNowComm::isDuplicate(const uint8_t* mac, uint8_t seq, uint8_t type) {
    if (!mac) return false;
    uint32_t now = millis();
    uint32_t maxAge = (type == MSG_AUDIO) ? 80 : 3500; // Audio window 80ms; mesh control 3.5s
    for (int i = 0; i < DEDUP_CACHE_SIZE; i++) {
        if (_dedupCache[i].seq == seq &&
            _dedupCache[i].type == type &&
            memcmp(_dedupCache[i].mac, mac, 6) == 0) {
            if (now - _dedupCache[i].timestampMs < maxAge) {
                return true;
            }
        }
    }
    return false;
}

void EspNowComm::markSeen(const uint8_t* mac, uint8_t seq, uint8_t type) {
    if (!mac) return;
    memcpy(_dedupCache[_dedupIdx].mac, mac, 6);
    _dedupCache[_dedupIdx].seq = seq;
    _dedupCache[_dedupIdx].type = type;
    _dedupCache[_dedupIdx].timestampMs = millis();
    _dedupIdx = (_dedupIdx + 1) % DEDUP_CACHE_SIZE;
}

// Peer Node Tracking
void EspNowComm::updatePeer(const uint8_t* mac, const char* name, int rssi) {
    if (!mac || !name) return;
    uint32_t now = millis();
    _lastPeerSeenMs = now;

    // Search existing
    for (int i = 0; i < MAX_PEERS; i++) {
        if (_peers[i].active && memcmp(_peers[i].mac, mac, 6) == 0) {
            strncpy(_peers[i].name, name, sizeof(_peers[i].name) - 1);
            _peers[i].rssi = rssi;
            _peers[i].lastSeenMs = now;
            return;
        }
    }

    // Find empty slot
    for (int i = 0; i < MAX_PEERS; i++) {
        if (!_peers[i].active) {
            memcpy(_peers[i].mac, mac, 6);
            strncpy(_peers[i].name, name, sizeof(_peers[i].name) - 1);
            _peers[i].name[sizeof(_peers[i].name) - 1] = '\0';
            _peers[i].rssi = rssi;
            _peers[i].lastSeenMs = now;
            _peers[i].active = true;
            Serial.printf("[PEER] Discovered: %s [%02X:%02X:%02X:%02X:%02X:%02X] (RSSI: %d)\n",
                          name, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], rssi);
            return;
        }
    }
}

void EspNowComm::pruneStalePeers() {
    uint32_t now = millis();
    for (int i = 0; i < MAX_PEERS; i++) {
        if (_peers[i].active && (now - _peers[i].lastSeenMs > PEER_TIMEOUT_MS)) {
            Serial.printf("[PEER] Stale node timed out: %s\n", _peers[i].name);
            _peers[i].active = false;
        }
    }
}

int EspNowComm::getActivePeers(PeerNode* outList, int maxList) {
    pruneStalePeers();
    int count = 0;
    for (int i = 0; i < MAX_PEERS && count < maxList; i++) {
        if (_peers[i].active) {
            outList[count++] = _peers[i];
        }
    }
    return count;
}

// Encrypted Transmission Implementation
bool EspNowComm::sendMessage(const EspNowMessage& msg) {
    uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint16_t sendLen = ESPNOW_MSG_HEADER_SIZE + msg.payload_len;

    esp_err_t result = esp_now_send(broadcastAddr, (const uint8_t*)&msg, sendLen);
    for (int retry = 0; retry < 3 && (result != ESP_OK); retry++) {
        delayMicroseconds(350 + (retry * 200));
        result = esp_now_send(broadcastAddr, (const uint8_t*)&msg, sendLen);
    }

    if (result == ESP_OK) {
        _txCount++;
        return true;
    }
    return false;
}

bool EspNowComm::sendPing(uint32_t pingToken) {
    EspNowMessage msg = {};
    msg.magic[0] = ESPNOW_FRAME_MAGIC_0;
    msg.magic[1] = ESPNOW_FRAME_MAGIC_1;
    msg.type = MSG_PING;
    msg.seq = ++_seq;
    msg.channel = _channel;
    msg.ttl = DEFAULT_TTL;
    msg.hops = 0;
    memcpy(msg.sender_mac, _mac, 6);
    memset(msg.target_mac, 0xFF, 6);
    strncpy(msg.sender_name, _deviceName, sizeof(msg.sender_name) - 1);

    Security& sec = Security::getInstance();
    sec.generateIv(msg.iv);

    PingPayload plain = {};
    plain.magic[0] = ESPNOW_PAYLOAD_MAGIC_0;
    plain.magic[1] = ESPNOW_PAYLOAD_MAGIC_1;
    plain.pingToken = pingToken;

    msg.payload_len = sizeof(PingPayload);
    sec.encrypt((const uint8_t*)&plain, msg.payload, sizeof(PingPayload), msg.iv, msg.seq, msg.type);

    Serial.printf("[ESP-NOW] TX Encrypted PING [token=%u ch=%d seq=%d]\n", pingToken, _channel, msg.seq);
    bool ok = sendMessage(msg);
    delayMicroseconds(600);
    sendMessage(msg); // 2nd burst for RF reliability
    return ok;
}

bool EspNowComm::sendPong(const uint8_t* targetMac, uint32_t pingToken, int8_t rssi) {
    EspNowMessage msg = {};
    msg.magic[0] = ESPNOW_FRAME_MAGIC_0;
    msg.magic[1] = ESPNOW_FRAME_MAGIC_1;
    msg.type = MSG_PONG;
    msg.seq = ++_seq;
    msg.channel = _channel;
    msg.ttl = DEFAULT_TTL;
    msg.hops = 0;
    memcpy(msg.sender_mac, _mac, 6);
    if (targetMac) {
        memcpy(msg.target_mac, targetMac, 6);
    } else {
        memset(msg.target_mac, 0xFF, 6);
    }
    strncpy(msg.sender_name, _deviceName, sizeof(msg.sender_name) - 1);

    Security& sec = Security::getInstance();
    sec.generateIv(msg.iv);

    PongPayload plain = {};
    plain.magic[0] = ESPNOW_PAYLOAD_MAGIC_0;
    plain.magic[1] = ESPNOW_PAYLOAD_MAGIC_1;
    plain.pingToken = pingToken;
    plain.rssi = rssi;

    msg.payload_len = sizeof(PongPayload);
    sec.encrypt((const uint8_t*)&plain, msg.payload, sizeof(PongPayload), msg.iv, msg.seq, msg.type);

    Serial.printf("[ESP-NOW] TX Encrypted PONG [echo=%u rssi=%d seq=%d]\n", pingToken, rssi, msg.seq);
    bool ok = sendMessage(msg);
    delayMicroseconds(600);
    sendMessage(msg);
    return ok;
}

bool EspNowComm::sendPttStart() {
    EspNowMessage msg = {};
    msg.magic[0] = ESPNOW_FRAME_MAGIC_0;
    msg.magic[1] = ESPNOW_FRAME_MAGIC_1;
    msg.type = MSG_PTT_START;
    msg.seq = ++_seq;
    msg.channel = _channel;
    msg.ttl = DEFAULT_TTL;
    msg.hops = 0;
    memcpy(msg.sender_mac, _mac, 6);
    memset(msg.target_mac, 0xFF, 6);
    strncpy(msg.sender_name, _deviceName, sizeof(msg.sender_name) - 1);

    Security& sec = Security::getInstance();
    sec.generateIv(msg.iv);

    uint8_t plain[2] = { ESPNOW_PAYLOAD_MAGIC_0, ESPNOW_PAYLOAD_MAGIC_1 };
    msg.payload_len = 2;
    sec.encrypt(plain, msg.payload, 2, msg.iv, msg.seq, msg.type);

    Serial.printf("[ESP-NOW] TX Encrypted VOICE_JOIN [seq=%d ch=%d]\n", msg.seq, _channel);
    bool ok = sendMessage(msg);
    delayMicroseconds(600);
    sendMessage(msg);
    return ok;
}

bool EspNowComm::sendPttStop(uint16_t totalAudioBytes) {
    EspNowMessage msg = {};
    msg.magic[0] = ESPNOW_FRAME_MAGIC_0;
    msg.magic[1] = ESPNOW_FRAME_MAGIC_1;
    msg.type = MSG_PTT_STOP;
    msg.seq = ++_seq;
    msg.channel = _channel;
    msg.ttl = DEFAULT_TTL;
    msg.hops = 0;
    memcpy(msg.sender_mac, _mac, 6);
    memset(msg.target_mac, 0xFF, 6);
    strncpy(msg.sender_name, _deviceName, sizeof(msg.sender_name) - 1);

    Security& sec = Security::getInstance();
    sec.generateIv(msg.iv);

    uint8_t plain[4] = {
        ESPNOW_PAYLOAD_MAGIC_0, ESPNOW_PAYLOAD_MAGIC_1,
        (uint8_t)(totalAudioBytes & 0xFF), (uint8_t)(totalAudioBytes >> 8)
    };
    msg.payload_len = 4;
    sec.encrypt(plain, msg.payload, 4, msg.iv, msg.seq, msg.type);

    Serial.printf("[ESP-NOW] TX Encrypted VOICE_LEAVE [bytes=%u seq=%d ch=%d]\n", totalAudioBytes, msg.seq, _channel);
    bool ok = sendMessage(msg);
    delayMicroseconds(600);
    sendMessage(msg);
    return ok;
}

bool EspNowComm::sendAudioChunk(uint8_t msgId, uint8_t chunkIdx, uint8_t totalChunks,
                                uint8_t mimeType, const uint8_t* data, uint16_t len) {
    if (!data || len == 0 || len > AUDIO_CHUNK_DATA_MAX) return false;

    EspNowMessage msg = {};
    msg.magic[0] = ESPNOW_FRAME_MAGIC_0;
    msg.magic[1] = ESPNOW_FRAME_MAGIC_1;
    msg.type = MSG_AUDIO;
    msg.seq = ++_seq;
    msg.channel = _channel;
    msg.ttl = DEFAULT_TTL;
    msg.hops = 0;
    memcpy(msg.sender_mac, _mac, 6);
    memset(msg.target_mac, 0xFF, 6);
    strncpy(msg.sender_name, _deviceName, sizeof(msg.sender_name) - 1);

    Security& sec = Security::getInstance();
    sec.generateIv(msg.iv);

    uint8_t plain[ESPNOW_MSG_MAX_PAYLOAD];
    plain[0] = ESPNOW_PAYLOAD_MAGIC_0;
    plain[1] = ESPNOW_PAYLOAD_MAGIC_1;
    plain[2] = msgId;
    plain[3] = chunkIdx;
    plain[4] = totalChunks;
    plain[5] = mimeType;
    memcpy(&plain[AUDIO_CHUNK_HEADER_SIZE], data, len);

    uint16_t plainLen = AUDIO_CHUNK_HEADER_SIZE + len;
    msg.payload_len = plainLen;
    sec.encrypt(plain, msg.payload, plainLen, msg.iv, msg.seq, msg.type);

    return sendMessage(msg);
}

bool EspNowComm::sendChannelBeacon(uint8_t channel, uint32_t uptimeSec) {
    EspNowMessage msg = {};
    msg.magic[0] = ESPNOW_FRAME_MAGIC_0;
    msg.magic[1] = ESPNOW_FRAME_MAGIC_1;
    msg.type = MSG_CHANNEL_BEACON;
    msg.seq = ++_seq;
    msg.channel = _channel;
    msg.ttl = DEFAULT_TTL;
    msg.hops = 0;
    memcpy(msg.sender_mac, _mac, 6);
    memset(msg.target_mac, 0xFF, 6);
    strncpy(msg.sender_name, _deviceName, sizeof(msg.sender_name) - 1);

    Security& sec = Security::getInstance();
    sec.generateIv(msg.iv);

    BeaconPayload plain = {};
    plain.magic[0] = ESPNOW_PAYLOAD_MAGIC_0;
    plain.magic[1] = ESPNOW_PAYLOAD_MAGIC_1;
    plain.channel = channel;
    plain.uptimeSec = uptimeSec;

    msg.payload_len = sizeof(BeaconPayload);
    sec.encrypt((const uint8_t*)&plain, msg.payload, sizeof(BeaconPayload), msg.iv, msg.seq, msg.type);

    return sendMessage(msg);
}

void EspNowComm::setReceiveCallback(EspNowDecryptedReceiveCb cb) {
    _receiveCb = cb;
}

// Receive Callback
void EspNowComm::onReceive(const uint8_t* mac, const uint8_t* data, int len) {
    if (len < ESPNOW_MSG_HEADER_SIZE) return;

    EspNowComm& self = getInstance();
    self._rxCount++;

    EspNowMessage msg;
    int copyLen = len;
    if (copyLen > (int)sizeof(EspNowMessage)) copyLen = (int)sizeof(EspNowMessage);
    memset(&msg, 0, sizeof(msg));
    memcpy(&msg, data, copyLen);

    // 1. Verify Frame Sync Magic (0xAA, 0x55)
    if (msg.magic[0] != ESPNOW_FRAME_MAGIC_0 || msg.magic[1] != ESPNOW_FRAME_MAGIC_1) {
        return; // Drop non-protocol packet
    }

    // 2. Drop self loopback
    if (memcmp(msg.sender_mac, self._mac, 6) == 0) {
        self._rxCount--;
        return;
    }

    // 3. Deduplication Check (Loop & Broadcast-Storm Prevention)
    if (self.isDuplicate(msg.sender_mac, msg.seq, msg.type)) {
        return;
    }
    self.markSeen(msg.sender_mac, msg.seq, msg.type);

    // 4. Hardware AES-128 Decryption
    uint8_t decrypted[ESPNOW_MSG_MAX_PAYLOAD + 1];
    uint16_t decLen = msg.payload_len;
    if (decLen > ESPNOW_MSG_MAX_PAYLOAD) decLen = ESPNOW_MSG_MAX_PAYLOAD;

    bool decOk = Security::getInstance().decrypt(msg.payload, decrypted, decLen, msg.iv, msg.seq, msg.type);
    if (!decOk) {
        return;
    }

    // 5. Authenticate Payload Magic ('W', 'K')
    if (decLen < 2 || decrypted[0] != ESPNOW_PAYLOAD_MAGIC_0 || decrypted[1] != ESPNOW_PAYLOAD_MAGIC_1) {
        return;
    }

    int packetRssi = -60;

    // 6. Update active peer record
    self.updatePeer(msg.sender_mac, msg.sender_name, packetRssi);

    // 7. Invoke application callback with authentic decrypted payload
    if (self._receiveCb) {
        self._receiveCb(msg, decrypted, decLen, packetRssi);
    }

    // 8. Automatic Mesh Relay Forwarding
    #if MESH_RELAY_ENABLED
    if (msg.ttl > 1) {
        // Direct audio streaming and PTT signals are direct 1-hop broadcasts to avoid RF collisions
        if (msg.type == MSG_AUDIO || msg.type == MSG_PTT_START || msg.type == MSG_PTT_STOP) {
            return;
        }

        EspNowMessage relayMsg = msg;
        relayMsg.ttl--;
        relayMsg.hops++;

        // Micro-jitter backoff (2.5 to 5.5ms) to prevent RF collision
        delayMicroseconds(2500 + (esp_random() % 3000));
        self.sendMessage(relayMsg);
        self._relayCount++;

        Serial.printf("[MESH] 🔁 Relayed %s from %s [seq=%u hops=%u->%u ttl=%u]\n",
                      (msg.type == MSG_PING ? "PING" :
                       msg.type == MSG_PONG ? "PONG" : "BEACON"),
                      msg.sender_name, msg.seq, msg.hops, relayMsg.hops, relayMsg.ttl);
    }
    #endif
}

void EspNowComm::onSend(const uint8_t* mac, esp_now_send_status_t status) {
    // Hardware send status callback
}
