#pragma once
// Board pins and tunables. Everything a builder may want to change is here.
#include "driver/gpio.h"

namespace cfg {

// Buttons (active low, internal pull-up). Any of them can be left unconnected.
constexpr gpio_num_t kPinBoot = GPIO_NUM_0;     // on-board BOOT: talk / long = ping
constexpr gpio_num_t kPinTalk = GPIO_NUM_4;     // optional external talk button
constexpr gpio_num_t kPinVolUp = GPIO_NUM_1;
constexpr gpio_num_t kPinVolDown = GPIO_NUM_2;

constexpr gpio_num_t kPinLed = GPIO_NUM_48;     // WS2812 on ESP32-S3-DevKitC-1 (v1.0: 48, v1.1: 38)
constexpr uint8_t kLedBrightness = 1;           // of 255; dim to absolute minimum (1 LSB) to eliminate 3V3 rail noise and save power

// Radio: Hard-locked to Channel 13 (2472 MHz), Pure LR 250k, 20 dBm TX power, -108 dBm sensitivity.
constexpr uint8_t kDefaultChannel = 13;          // Static Lock: Ch 13 (2472 MHz)
constexpr uint8_t kDefaultBackupChannel = 13;   // Static Lock: Zero Channel Hopping
constexpr int8_t  kTxPowerQdBm = 84;            // 21 dBm (clamped to 20 dBm / 100 mW max hardware power)
// Send at Espressif LR 250 kbit/s instead of 6 Mbit/s OFDM. Gives up to ~108 dBm sensitivity
// (vs -90 dBm on OFDM) for dramatically improved range through obstacles and distance.
constexpr bool    kLongRange = true;
constexpr bool    kDuplicateAudioFrames = true; // send voice frames with duplicate redundancy for resilience against packet loss at range edge
constexpr uint32_t kBeaconMs = 3000;

// Talk
constexpr uint32_t kMaxTalkMs = 0;              // 0 = disabled (never auto-mute, stay ON until user turns it OFF)
// Voice-gate power saving:
// When false: stops transmitting RF packets during silence, saving ~150mA current and dramatically
// reducing heat. Resumes transmitting instantly when speech is detected.
// When true: stream audio continuously while ON even during complete silence.
constexpr bool     kContinuousStream = false;
constexpr uint8_t  kDefaultVolume = 70;

// Network key: 16 bytes, same on every node. Set WT_NETWORK_KEY_HEX (32 hex chars) as a
// build flag, see README. The fallback key is public: anyone can join a group using it.
#ifdef WT_NETWORK_KEY_HEX
constexpr const char* kNetworkKeyHex = WT_NETWORK_KEY_HEX;
constexpr bool kDefaultKey = false;
#else
#warning "WT_NETWORK_KEY_HEX not set: building with the public default key (see README)"
constexpr bool kDefaultKey = true;
constexpr const char* kNetworkKeyHex = "57616c6b6965546f6b65725f53332121";
#endif

}  // namespace cfg
