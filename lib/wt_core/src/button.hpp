#pragma once
// Press detector shared by the BOOT button, the external button and the headset's own
// button (HID), so all of them behave the same. Click fires on release (< long press),
// long press fires once while still held. No double-click: it would delay every click.
#include <cstdint>

namespace wt {

enum class Press : uint8_t { None, Click, Long, Hold };

class ButtonDetector {
public:
    static constexpr uint32_t kDebounceMs = 30;
    static constexpr uint32_t kLongMs = 1200;
    static constexpr uint32_t kHoldMs = 2500;
    // Feed the raw level whenever it is sampled (or changes). Returns an event or None.
    Press update(bool down, uint32_t now_ms);

private:
    bool stable_ = false;
    bool raw_ = false;
    bool long_sent_ = false;
    bool hold_sent_ = false;
    uint32_t changed_ms_ = 0;
    uint32_t pressed_ms_ = 0;
};

}  // namespace wt
