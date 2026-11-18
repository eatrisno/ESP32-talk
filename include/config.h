#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// Firmware Version
// ============================================================
#define FIRMWARE_VERSION          "2.1.0-HT-MESH"

// ============================================================
// ESP-NOW Long-Range RF & Mesh Configuration
// ============================================================
#define ESPNOW_ENABLE_LR          true        // Long Range mode (WIFI_PROTOCOL_LR)
#define ESPNOW_TX_POWER           80          // 80 = 20 dBm (100mW, maximum legal TX power)
#define DEFAULT_TTL               3           // Automatic Mesh: up to 2 intermediate relays (reaches 1.5 - 2+ km)
#define MESH_RELAY_ENABLED        true        // Every node acts as an autonomous repeater

// ============================================================
// Hardware AES-128 Encryption Security Key
// 128-bit secret key shared between all paired nodes
// ============================================================
#define AES_KEY_BYTES { \
    0x57, 0x61, 0x6C, 0x6B, 0x69, 0x65, 0x54, 0x61, \
    0x6C, 0x6B, 0x69, 0x65, 0x5F, 0x53, 0x33, 0x21  \
} // "WalkieTalkie_S3!" (16 bytes)

// ============================================================
// Auto-Channel Search & Peer Discovery
// ============================================================
#define CHANNEL_MIN               1
#define CHANNEL_MAX               13
#define CHANNEL_DEFAULT           1
#define CHANNEL_SCAN_DWELL_MS     250         // Dwell time per channel while scanning
#define CHANNEL_BEACON_INTERVAL_MS 15000      // Broadcast beacon every 15s (energy saving)
#define CHANNEL_BEACON_FAST_MS    4000        // Fast 4s beacon during initial startup
#define PEER_AUTO_SEARCH_TIMEOUT_MS 50000     // Auto-scan if no peer heard for 50s
#define PEER_TIMEOUT_MS           40000       // Peer considered offline after 40s
#define MAX_PEERS                 8

// ============================================================
// USB-OTG Audio Configuration
// ============================================================
#define AUDIO_SAMPLE_RATE         8000        // 8kHz mono voice
#define AUDIO_BITS                16          // 16-bit linear PCM
#define AUDIO_CHANNELS            1           // Mono

// ============================================================
// Hardware Pin Definitions
// ============================================================
#define PTT_BUTTON_PIN            0           // On-board BOOT button on ESP32-S3 DevKit (GPIO 0)
#define PTT_EXT_BUTTON_PIN        4           // Headset / External Button for Voice Join (INPUT_PULLUP to GND)
#define VOL_UP_BUTTON_PIN         1           // Volume UP Button (INPUT_PULLUP to GND)
#define VOL_DOWN_BUTTON_PIN       2           // Volume DOWN Button (INPUT_PULLUP to GND)
#define LED_PIN                   48          // Built-in WS2812 RGB LED (GPIO 48)
#define BUZZER_PIN                21          // Alert tone buzzer (GPIO 21)

// ============================================================
// Visual Light Indicator (WS2812 RGB)
// ============================================================
enum LedColor {
    LED_COLOR_OFF,
    LED_COLOR_RED,      // SENDER: Voice Active / Transmitting
    LED_COLOR_BLUE,     // Transmit complete / blue confirmation blip
    LED_COLOR_GREEN,    // TARGET: Remote Audio Arrived & Playing / Peer Found
    LED_COLOR_AMBER,    // AUTO-SEARCH: Scanning channels
    LED_COLOR_CYAN      // PING: Ping sent, awaiting pong / Channel locked
};

void setStatusLed(LedColor color, uint32_t autoOffMs = 0);

// ============================================================
// NVS Storage Keys
// ============================================================
#define NVS_NAMESPACE             "walkie"
#define NVS_KEY_DEVICE_NAME       "dev_name"
#define NVS_KEY_CHANNEL           "channel"
#define NVS_KEY_VOLUME            "vol"

#endif // CONFIG_H
