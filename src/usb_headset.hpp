#pragma once
// USB-C headset: UAC 1.0 microphone and speaker plus HID buttons, on the ESP-IDF USB host
// stack. Any rate/channels/bit depth the headset offers is accepted.
#include <cstddef>
#include <cstdint>

#include "format.hpp"

namespace usb_headset {

using ButtonCb = void (*)(uint8_t hid_buttons_down);  // wt::HidButton bits, on every change
using StateCb = void (*)(bool connected);

void start(ButtonCb on_buttons, StateCb on_state);

bool mic_ready();
bool spk_ready();
wt::StreamFormat mic_format();
wt::StreamFormat spk_format();
bool has_buttons();

// Blocking with timeout. Return bytes read / false on error or no stream.
size_t mic_read(uint8_t* buf, size_t max, uint32_t timeout_ms);
bool spk_write(const uint8_t* buf, size_t len, uint32_t timeout_ms);

// One line per stream for the console.
void print_info();

}  // namespace usb_headset
