#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include "config.h"
#include "messages.h"
#include "security.h"
#include "esp_now_comm.h"
#include "usb_headset.h"

// Preferences for non-volatile storage (NVS)
static Preferences prefs;

// Node Identity & Network State
static char     g_deviceName[16]    = "";
static uint8_t  g_channel           = CHANNEL_DEFAULT;
static uint32_t g_lastBeaconMs      = 0;
static uint32_t g_ledOffMs          = 0;
static uint32_t g_buzzerOffMs       = 0;

// Fleet Discovery Ping State
#define PING_COLLECTION_WINDOW_MS 450
static bool     g_pingCollecting    = false;
static uint32_t g_pendingPingToken  = 0;
static uint32_t g_pingSentMs        = 0;
static uint8_t  g_pongDirectCount   = 0;
static uint8_t  g_pongRelayCount    = 0;

struct PingResponder {
    uint8_t mac[6];
    char    name[16];
    uint8_t hops;
    int8_t  remoteRssi;
    int8_t  localRssi;
    uint32_t rtt;
};
static PingResponder g_responders[MAX_PEERS];
static uint8_t       g_numResponders = 0;

// Auto-Channel Search State
static bool     g_isScanning        = false;
static uint8_t  g_scanChannel       = CHANNEL_MIN;
static uint32_t g_scanDwellStartMs  = 0;
static uint8_t  g_origChannel       = CHANNEL_DEFAULT;

// Remote Voice State
static bool     g_remoteVoiceActive = false;
static uint32_t g_remoteVoiceExpiry = 0;
static char     g_remoteSpeakerName[16] = "Remote";

// WS2812 RGB LED Driver (Gentle ~6% Dimming to prevent RF / 3.3V rail noise)
static uint8_t s_lastR = 255, s_lastG = 255, s_lastB = 255;

static void rawRgbWrite(uint8_t r, uint8_t g, uint8_t b) {
    r = (r * 16) / 255;
    g = (g * 16) / 255;
    b = (b * 16) / 255;

    if (r == s_lastR && g == s_lastG && b == s_lastB) {
        return;
    }
    s_lastR = r;
    s_lastG = g;
    s_lastB = b;

    #ifdef RGB_BUILTIN
    neopixelWrite(RGB_BUILTIN, r, g, b);
    #else
    neopixelWrite(48, r, g, b);
    #endif
}

void setStatusLed(LedColor color, uint32_t autoOffMs) {
    g_ledOffMs = (autoOffMs > 0) ? (millis() + autoOffMs) : 0;

    switch (color) {
        case LED_COLOR_OFF:
            rawRgbWrite(0, 0, 0);
            break;
        case LED_COLOR_RED:    // SENDER: Voice Transmit Active
            rawRgbWrite(255, 0, 0);
            break;
        case LED_COLOR_BLUE:   // Transmit complete blip
            rawRgbWrite(0, 80, 255);
            break;
        case LED_COLOR_GREEN:  // TARGET: Incoming Voice Playing / Peer Found
            rawRgbWrite(0, 255, 0);
            break;
        case LED_COLOR_AMBER:  // AUTO-SEARCH: Scanning channels
            rawRgbWrite(255, 120, 0);
            break;
        case LED_COLOR_CYAN:   // PING: Ping in flight / Channel locked
            rawRgbWrite(0, 255, 200);
            break;
    }
}

static void blinkPattern(LedColor color, int count, uint16_t onMs = 120, uint16_t offMs = 80) {
    for (int i = 0; i < count; i++) {
        setStatusLed(color);
        delay(onMs);
        setStatusLed(LED_COLOR_OFF);
        if (i < count - 1) delay(offMs);
    }
}

static void triggerAlertAsync(uint16_t durationMs = 60) {
    digitalWrite(BUZZER_PIN, HIGH);
    g_buzzerOffMs = millis() + durationMs;
}

static void triggerAlert(uint16_t durationMs = 60, uint16_t count = 1) {
    for (uint16_t i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(durationMs);
        digitalWrite(BUZZER_PIN, LOW);
        if (i < count - 1) delay(50);
    }
}

// Fleet Discovery Ping Trigger
static void triggerFleetPing(uint32_t now) {
    if (g_pingCollecting) return;
    g_pendingPingToken = now;
    g_pingSentMs = now;
    g_pingCollecting = true;
    g_numResponders = 0;
    g_pongDirectCount = 0;
    g_pongRelayCount = 0;

    Serial.println("\n[MESH-PING] ========================================");
    Serial.printf("[MESH-PING] Probing fleet on channel %d [token=%u]...\n", g_channel, g_pendingPingToken);
    Serial.println("[MESH-PING] Listening for direct and relayed responses (450ms)...");
    Serial.println("[MESH-PING] ========================================");

    setStatusLed(LED_COLOR_CYAN, 80);
    EspNowComm::getInstance().sendPing(g_pendingPingToken);
}

// Auto-Channel Search
static void startAutoChannelSearch() {
    if (g_isScanning) return;
    Serial.println("\n[AUTO-SEARCH] ========================================");
    Serial.println("[AUTO-SEARCH] Starting 13-channel peer search...");
    Serial.println("[AUTO-SEARCH] ========================================");

    g_isScanning = true;
    g_origChannel = g_channel;
    g_scanChannel = CHANNEL_MIN;
    g_scanDwellStartMs = millis();

    setStatusLed(LED_COLOR_AMBER);
    EspNowComm::getInstance().setChannel(g_scanChannel);
    EspNowComm::getInstance().sendChannelBeacon(g_scanChannel, millis() / 1000);
}

static void processAutoChannelSearch() {
    if (!g_isScanning) return;

    uint32_t now = millis();
    if (now - g_scanDwellStartMs >= CHANNEL_SCAN_DWELL_MS) {
        g_scanChannel++;
        if (g_scanChannel > CHANNEL_MAX) {
            Serial.printf("[AUTO-SEARCH] Scan cycle complete without discovery. Reverting to ch%d\n", g_origChannel);
            g_isScanning = false;
            g_channel = g_origChannel;
            EspNowComm::getInstance().setChannel(g_channel);
            setStatusLed(LED_COLOR_OFF);
            return;
        }

        g_scanDwellStartMs = now;
        setStatusLed(LED_COLOR_AMBER);
        EspNowComm::getInstance().setChannel(g_scanChannel);
        EspNowComm::getInstance().sendChannelBeacon(g_scanChannel, millis() / 1000);
        Serial.printf("[AUTO-SEARCH] Probing channel %d...\n", g_scanChannel);
    }
}

static void onPeerDiscoveredOnChannel(uint8_t channel) {
    if (g_isScanning) {
        g_isScanning = false;
        g_channel = channel;
        prefs.putUChar(NVS_KEY_CHANNEL, g_channel);
        Serial.println("\n[AUTO-SEARCH] ****************************************");
        Serial.printf("[AUTO-SEARCH] PEER FOUND! Locked to Channel %d\n", g_channel);
        Serial.println("[AUTO-SEARCH] ****************************************\n");
        blinkPattern(LED_COLOR_CYAN, 2, 150, 100);
        triggerAlert(100, 2);
    }
}

// Dynamic Callsign Pool (Auto-assigned without hardcoding)
static const char* CALLSIGN_POOL[] = { "Peto", "Bubu", "Wawa" };
static const int POOL_SIZE = sizeof(CALLSIGN_POOL) / sizeof(CALLSIGN_POOL[0]);

static void saveDeviceName(const char* name) {
    if (!name || strlen(name) == 0) return;
    strncpy(g_deviceName, name, sizeof(g_deviceName) - 1);
    g_deviceName[sizeof(g_deviceName) - 1] = '\0';
    prefs.putString(NVS_KEY_DEVICE_NAME, g_deviceName);
    EspNowComm::getInstance().setDeviceName(g_deviceName);
    Serial.printf("\n[NVS] Callsign saved to flash: %s\n\n", g_deviceName);
}

// ESP-NOW Receive Callback (Encrypted & Authenticated)
static void onEspNowDecryptedReceived(const EspNowMessage& header,
                                     const uint8_t* decryptedPayload,
                                     uint16_t decryptedLen,
                                     int rssi) {
    if (g_isScanning) {
        onPeerDiscoveredOnChannel(header.channel);
    }

    switch (header.type) {
        case MSG_PING: {
            if (decryptedLen >= sizeof(PingPayload)) {
                const PingPayload* ping = (const PingPayload*)decryptedPayload;
                // Zero-turnaround: send Pong reply immediately
                EspNowComm::getInstance().sendPong(header.sender_mac, ping->pingToken, rssi);
                setStatusLed(LED_COLOR_CYAN, 300);
                triggerAlertAsync(40);
                Serial.printf("[PING] Received Ping from %s [token=%u]\n", header.sender_name, ping->pingToken);
            }
            break;
        }

        case MSG_PONG: {
            if (decryptedLen >= sizeof(PongPayload)) {
                const PongPayload* pong = (const PongPayload*)decryptedPayload;
                if (g_pingCollecting && (pong->pingToken == g_pendingPingToken)) {
                    uint32_t rtt = millis() - g_pingSentMs;
                    bool exists = false;
                    for (int i = 0; i < g_numResponders; i++) {
                        if (memcmp(g_responders[i].mac, header.sender_mac, 6) == 0) {
                            exists = true;
                            break;
                        }
                    }
                    if (!exists && g_numResponders < MAX_PEERS) {
                        PingResponder& r = g_responders[g_numResponders++];
                        memcpy(r.mac, header.sender_mac, 6);
                        strncpy(r.name, header.sender_name, sizeof(r.name) - 1);
                        r.name[sizeof(r.name) - 1] = '\0';
                        r.hops = header.hops;
                        r.remoteRssi = pong->rssi;
                        r.localRssi = rssi;
                        r.rtt = rtt;

                        if (header.hops == 0) g_pongDirectCount++;
                        else g_pongRelayCount++;

                        Serial.printf("[PING-FLEET] RX Pong from %s (%s, hops=%u, RTT=%ums)\n",
                                      header.sender_name, (header.hops == 0 ? "🟢 Direct" : "🩵 Relayed"),
                                      header.hops, (unsigned int)rtt);
                    }
                }
            }
            break;
        }

        case MSG_PTT_START: {
            g_remoteVoiceActive = true;
            strncpy(g_remoteSpeakerName, header.sender_name, sizeof(g_remoteSpeakerName) - 1);
            g_remoteSpeakerName[sizeof(g_remoteSpeakerName) - 1] = '\0';
            g_remoteVoiceExpiry = millis() + 1000;
            setStatusLed(LED_COLOR_GREEN);
            Serial.printf("[HT-VOICE] 👥 Peer JOINED voice channel: %s\n", header.sender_name);
            break;
        }

        case MSG_PTT_STOP: {
            g_remoteVoiceActive = false;
            g_remoteVoiceExpiry = 0;
            UsbHeadset::getInstance().onRemotePttStop(header.sender_mac);
            if (!UsbHeadset::getInstance().isTransmitting()) {
                setStatusLed(LED_COLOR_OFF);
            }
            Serial.printf("[HT-VOICE] 👥 Peer LEFT voice channel: %s\n", header.sender_name);
            break;
        }

        case MSG_AUDIO: {
            if (decryptedLen > AUDIO_CHUNK_HEADER_SIZE) {
                g_remoteVoiceActive = true;
                g_remoteVoiceExpiry = millis() + 800;

                uint8_t msgId       = decryptedPayload[2];
                uint8_t chunkIdx    = decryptedPayload[3];
                uint8_t totalChunks = decryptedPayload[4];
                uint8_t mimeType    = decryptedPayload[5];
                const uint8_t* audioData = &decryptedPayload[AUDIO_CHUNK_HEADER_SIZE];
                uint16_t dataLen = decryptedLen - AUDIO_CHUNK_HEADER_SIZE;

                // Deliver to Headset Full Duplex Jitter Queue & Mixer
                UsbHeadset::getInstance().receiveAudioChunk(
                    header.sender_mac, msgId, chunkIdx, totalChunks, mimeType, audioData, dataLen
                );

                if (!UsbHeadset::getInstance().isTransmitting()) {
                    setStatusLed(LED_COLOR_GREEN, 500);
                }
            }
            break;
        }

        case MSG_CHANNEL_BEACON:
            break;

        default:
            break;
    }
}

// Setup
void setup() {
    // Lock CPU to 240 MHz for real-time AES crypto and DSP
    setCpuFrequencyMhz(240);

    Serial.begin(115200);
    delay(400);

    Serial.println("\n=======================================================");
    Serial.println("   ESP32-S3 HT Full Duplex Mesh Walkie-Talkie");
    Serial.printf("   Firmware: %s\n", FIRMWARE_VERSION);
    Serial.println("   Engine: ESP-NOW LR Mesh + AES-128 Crypto + Type-C Headset");
    Serial.println("=======================================================");

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    pinMode(PTT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(PTT_EXT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(VOL_UP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(VOL_DOWN_BUTTON_PIN, INPUT_PULLUP);

    // Visual Power-On Self Test: Red -> Green -> Blue -> Off
    setStatusLed(LED_COLOR_RED);
    delay(100);
    setStatusLed(LED_COLOR_GREEN);
    delay(100);
    setStatusLed(LED_COLOR_BLUE);
    delay(100);
    setStatusLed(LED_COLOR_OFF);

    // Boot chirp
    triggerAlert(60, 2);

    // Initialize Preferences
    prefs.begin(NVS_NAMESPACE, false);
    String savedName = prefs.getString(NVS_KEY_DEVICE_NAME, "");

    if (savedName.length() > 0) {
        strncpy(g_deviceName, savedName.c_str(), sizeof(g_deviceName) - 1);
        Serial.printf("[NVS] Callsign: %s\n", g_deviceName);
    } else {
        uint8_t mac[6];
        WiFi.macAddress(mac);
        uint32_t seed = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        const char* candidate = CALLSIGN_POOL[seed % POOL_SIZE];
        saveDeviceName(candidate);
        Serial.printf("[NVS] Auto-assigned callsign: %s\n", g_deviceName);
    }

    g_channel = prefs.getUChar(NVS_KEY_CHANNEL, CHANNEL_DEFAULT);
    if (g_channel < CHANNEL_MIN || g_channel > CHANNEL_MAX) {
        g_channel = CHANNEL_DEFAULT;
    }

    // Load saved volume (default 80%)
    uint8_t savedVol = prefs.getUChar(NVS_KEY_VOLUME, 80);
    if (savedVol > 100) savedVol = 80;
    UsbHeadset::getInstance().setVolume(savedVol);

    // 1. Initialize ESP-NOW Mesh Subsystem
    EspNowComm& comm = EspNowComm::getInstance();
    if (g_deviceName[0] != '\0') {
        comm.setDeviceName(g_deviceName);
    }
    comm.setReceiveCallback(onEspNowDecryptedReceived);
    comm.init(g_channel, ESPNOW_ENABLE_LR);

    // 2. Initialize USB Host Type-C Headset Audio Driver
    UsbHeadset::getInstance().begin();

    Serial.println("\n[SETUP] Node Ready!");
    Serial.printf("[SETUP] Callsign: %s | Channel: %d | MAC: %s\n", g_deviceName, g_channel, comm.getMacString().c_str());
    Serial.printf("[SETUP] Audio Volume: %u%%\n", UsbHeadset::getInstance().getVolume());
    Serial.println("[SETUP] Headset Button (GPIO 4) / BOOT (GPIO 0): Press to Join / Leave Voice (HT Full Duplex)");
    Serial.println("[SETUP] Volume UP Button (GPIO 1) / Volume DOWN Button (GPIO 2)");
    Serial.println("[SETUP] BOOT Click: Fleet Discovery Ping | BOOT Double Click: Auto-Search Channels 1..13\n");
}

// Main Loop
void loop() {
    uint32_t now = millis();
    EspNowComm& comm = EspNowComm::getInstance();

    // ============================================================
    // Non-blocking Interactive Serial Terminal
    // ============================================================
    static char   s_serialBuf[128];
    static size_t s_serialIdx = 0;

    while (Serial.available()) {
        int c = Serial.read();
        if (c < 0) break;
        if (c == '\r') continue;
        if (c == '\n') {
            s_serialBuf[s_serialIdx] = '\0';
            if (s_serialIdx > 0) {
                char* line = s_serialBuf;
                size_t len = s_serialIdx;

                if (len == 1 && (line[0] == 't' || line[0] == 'T' || line[0] == ' ')) {
                    UsbHeadset::getInstance().toggleVoice();
                } else if (len == 1 && (line[0] == '+' || line[0] == '=' || line[0] == 'u' || line[0] == 'U')) {
                    UsbHeadset::getInstance().volumeUp(10);
                    prefs.putUChar(NVS_KEY_VOLUME, UsbHeadset::getInstance().getVolume());
                } else if (len == 1 && (line[0] == '-' || line[0] == '_' || line[0] == 'd' || line[0] == 'D')) {
                    UsbHeadset::getInstance().volumeDown(10);
                    prefs.putUChar(NVS_KEY_VOLUME, UsbHeadset::getInstance().getVolume());
                } else if (len == 1 && (line[0] == 'p' || line[0] == 'P')) {
                    triggerFleetPing(now);
                } else if (len == 1 && (line[0] == 'c' || line[0] == 'C')) {
                    startAutoChannelSearch();
                } else if (strncasecmp(line, "ch ", 3) == 0 || strncasecmp(line, "channel ", 8) == 0) {
                    const char* p = strchr(line, ' ');
                    if (p) {
                        int ch = atoi(p + 1);
                        if (ch >= CHANNEL_MIN && ch <= CHANNEL_MAX) {
                            g_channel = ch;
                            prefs.putUChar(NVS_KEY_CHANNEL, g_channel);
                            comm.setChannel(g_channel);
                            Serial.printf("[CHANNEL] 📻 Switched to Channel %d\n", g_channel);
                        } else {
                            Serial.println("[CHANNEL] Invalid channel (1..13)");
                        }
                    }
                } else if (strncasecmp(line, "name ", 5) == 0) {
                    const char* name = line + 5;
                    while (*name == ' ') name++;
                    if (strlen(name) > 0) saveDeviceName(name);
                } else if (len == 1 && (line[0] == '?' || line[0] == 'h' || line[0] == 'H')) {
                    Serial.println("\n============== HT WALKIE-TALKIE COMMANDS ==============");
                    Serial.println("  t / Space -> 🎙️ Toggle Join / Leave Voice (HT Full Duplex)");
                    Serial.println("  + / u     -> 🔊 Volume UP (+10%)");
                    Serial.println("  - / d     -> 🔉 Volume DOWN (-10%)");
                    Serial.println("  p         -> 📡 Fleet Discovery Ping");
                    Serial.println("  c         -> 🔍 Manual Channel Search (1..13)");
                    Serial.println("  ch <1-13> -> 📻 Change Operating Channel");
                    Serial.println("  name <str>-> 🏷️ Set Callsign Name");
                    Serial.printf( "  Status    -> Transmitting: %s | Volume: %u%% | Ch: %d | MAC: %s\n",
                                  UsbHeadset::getInstance().isTransmitting() ? "YES (Joined)" : "NO (Standby)",
                                  UsbHeadset::getInstance().getVolume(), g_channel, comm.getMacString().c_str());
                    Serial.printf( "  Headset   -> Connected: %s | DAC: %u Hz, %u-bit, %u ch | Mic: %u Hz, %u-bit, %u ch\n",
                                  UsbHeadset::getInstance().isConnected() ? "YES" : "NO",
                                  (unsigned int)UsbHeadset::getInstance().getSpkSampleRate(),
                                  UsbHeadset::getInstance().getSpkBitRes(),
                                  UsbHeadset::getInstance().getSpkChannels(),
                                  (unsigned int)UsbHeadset::getInstance().getMicSampleRate(),
                                  UsbHeadset::getInstance().getMicBitRes(),
                                  UsbHeadset::getInstance().getMicChannels());
                    Serial.println("========================================================\n");
                }
            }
            s_serialIdx = 0;
        } else if (s_serialIdx < sizeof(s_serialBuf) - 1) {
            s_serialBuf[s_serialIdx++] = (char)c;
        } else {
            s_serialIdx = 0;
        }
    }

    // USB Headset Background Loop
    UsbHeadset::getInstance().loop();

    // Channel Scanning (active only when manually triggered)
    if (g_isScanning) {
        processAutoChannelSearch();
    }

    // Periodic Mesh Channel Beacon (15s energy-saving, 4s startup)
    PeerNode peers[1];
    bool hasActivePeers = (comm.getActivePeers(peers, 1) > 0);
    uint32_t beaconInterval = hasActivePeers ? CHANNEL_BEACON_INTERVAL_MS : CHANNEL_BEACON_FAST_MS;

    if (!g_isScanning && (now - g_lastBeaconMs >= beaconInterval)) {
        g_lastBeaconMs = now;
        comm.pruneStalePeers();
        comm.sendChannelBeacon(g_channel, now / 1000);
    }

    // Fleet Discovery Ping Window Processing (450ms collection window)
    if (g_pingCollecting && (now - g_pingSentMs >= PING_COLLECTION_WINDOW_MS)) {
        g_pingCollecting = false;

        Serial.println("\n[PING-FLEET] ===================================================");
        Serial.printf("[PING-FLEET] Discovery Complete in %u ms!\n", (unsigned int)(now - g_pingSentMs));
        Serial.printf("[PING-FLEET] Total Nodes Reached: %u (Direct: %u, Relayed: %u)\n",
                      g_numResponders, g_pongDirectCount, g_pongRelayCount);

        for (int i = 0; i < g_numResponders; i++) {
            const PingResponder& r = g_responders[i];
            const char* routeStr = (r.hops == 0) ? "🟢 Direct Line-of-Sight" :
                                   (r.hops == 1) ? "🩵 1-Hop Mesh Relay 🔁" : "🩵 2-Hop Mesh Relay 🔁🔁";
            Serial.printf("[PING-FLEET] [%u] %-6s | RTT: %2u ms | Local RSSI: %3d dBm | Remote RSSI: %3d dBm | %s\n",
                          i + 1, r.name, (unsigned int)r.rtt, r.localRssi, r.remoteRssi, routeStr);
        }
        Serial.println("[PING-FLEET] ===================================================\n");

        if (g_numResponders == 0) {
            blinkPattern(LED_COLOR_RED, 2, 120, 80);
            triggerAlert(150, 1);
        } else {
            blinkPattern(LED_COLOR_GREEN, g_numResponders, 100, 60);
        }
    }

    // ============================================================
    // HEADSET / EXTERNAL BUTTON (GPIO 4) & ON-BOARD BOOT (GPIO 0):
    // Press button to JOIN voice channel, press again to LEAVE!
    // Double click BOOT -> Auto-channel search
    // ============================================================
    bool pttBtnRaw = (digitalRead(PTT_BUTTON_PIN) == LOW) || (digitalRead(PTT_EXT_BUTTON_PIN) == LOW);
    static bool     s_pttPressed     = false;
    static uint32_t s_pttDebounceMs  = 0;
    static uint32_t s_lastPttClickMs = 0;

    if (pttBtnRaw != s_pttPressed && (now - s_pttDebounceMs >= 40)) {
        s_pttDebounceMs = now;
        s_pttPressed = pttBtnRaw;

        if (s_pttPressed) {
            // Check double-click for channel search (within 350ms)
            if (now - s_lastPttClickMs < 350) {
                s_lastPttClickMs = 0;
                startAutoChannelSearch();
            } else {
                s_lastPttClickMs = now;
                // Single Click: Toggle Voice Join / Leave (Full Duplex HT)
                UsbHeadset::getInstance().toggleVoice();
            }
        }
    }

    // ============================================================
    // Volume UP Button (GPIO 1 to GND)
    // ============================================================
    bool volUpRaw = (digitalRead(VOL_UP_BUTTON_PIN) == LOW);
    static bool     s_volUpPressed    = false;
    static uint32_t s_volUpDebounceMs = 0;

    if (volUpRaw != s_volUpPressed && (now - s_volUpDebounceMs >= 40)) {
        s_volUpDebounceMs = now;
        s_volUpPressed = volUpRaw;
        if (s_volUpPressed) {
            UsbHeadset::getInstance().volumeUp(10);
            prefs.putUChar(NVS_KEY_VOLUME, UsbHeadset::getInstance().getVolume());
        }
    }

    // ============================================================
    // Volume DOWN Button (GPIO 2 to GND)
    // ============================================================
    bool volDownRaw = (digitalRead(VOL_DOWN_BUTTON_PIN) == LOW);
    static bool     s_volDownPressed    = false;
    static uint32_t s_volDownDebounceMs = 0;

    if (volDownRaw != s_volDownPressed && (now - s_volDownDebounceMs >= 40)) {
        s_volDownDebounceMs = now;
        s_volDownPressed = volDownRaw;
        if (s_volDownPressed) {
            UsbHeadset::getInstance().volumeDown(10);
            prefs.putUChar(NVS_KEY_VOLUME, UsbHeadset::getInstance().getVolume());
        }
    }

    // Auto-turn off status LED after timeout (if not actively transmitting or receiving)
    if (g_ledOffMs > 0 && now >= g_ledOffMs) {
        g_ledOffMs = 0;
        if (!UsbHeadset::getInstance().isTransmitting() && !UsbHeadset::getInstance().isPlaying()) {
            setStatusLed(LED_COLOR_OFF);
        }
    }

    // Auto-turn off buzzer
    if (g_buzzerOffMs > 0 && now >= g_buzzerOffMs) {
        g_buzzerOffMs = 0;
        digitalWrite(BUZZER_PIN, LOW);
    }

    vTaskDelay(pdMS_TO_TICKS(15));
}
