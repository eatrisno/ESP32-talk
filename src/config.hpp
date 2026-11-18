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
constexpr uint8_t kLedBrightness = 2;           // of 255; dim to eliminate 3V3 power rail noise and save power

// Radio
constexpr uint8_t kDefaultChannel = 13;          // Ch 13 (2472 MHz) is quiet, far from crowded Wi-Fi channels 1, 6, 11
constexpr int8_t  kTxPowerQdBm = 84;            // 21 dBm (maximum hardware power).
// Send at Espressif LR 250 kbit/s instead of 6 Mbit/s OFDM. Gives up to ~108 dBm sensitivity
// (vs -90 dBm on OFDM) for dramatically improved range through obstacles and distance.
constexpr bool    kLongRange = true;
constexpr bool    kDuplicateAudioFrames = true; // send voice frames with duplicate redundancy for resilience against packet loss at range edge
constexpr uint32_t kBeaconMs = 3000;

// Talk
constexpr uint32_t kMaxTalkMs = 0;              // 0 = disabled (never auto-mute, stay ON until user turns it OFF)
constexpr bool     kContinuousStream = true;    // stream audio continuously while ON without voice-gate auto-muting
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
