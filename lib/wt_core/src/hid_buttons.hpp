#pragma once
// Headset buttons over USB HID. Parses the device's report descriptor once, then turns
// each input report into a set of buttons. Handles both layouts headsets use: one bit per
// button (variable fields) and "usage code" arrays.
#include <cstddef>
#include <cstdint>

namespace wt {

enum HidButton : uint8_t {
    HID_BTN_TALK = 1 << 0,      // Play/Pause, Play, Hook switch, Mic mute
    HID_BTN_VOL_UP = 1 << 1,
    HID_BTN_VOL_DOWN = 1 << 2,
};

class HidButtons {
public:
    // Returns false if the descriptor has no button this device can use.
    bool parse(const uint8_t* desc, size_t len);
    // Buttons currently held according to this report (report id byte included when
    // the descriptor uses report ids).
    uint8_t decode(const uint8_t* report, size_t len) const;
    bool uses_report_ids() const { return uses_ids_; }
    int field_count() const { return n_; }

private:
    struct Field {
        uint8_t  report_id;
        uint16_t bit;         // offset after the report id byte
        uint8_t  size;
        uint8_t  count;
        bool     array;
        uint8_t  button;      // variable field: the button for this single element
        uint16_t page;
        uint16_t usage_min;   // array field: element value v -> usage usage_min + v - lmin
        int32_t  lmin;
    };
    static constexpr int kMax = 16;
    Field f_[kMax] = {};
    int n_ = 0;
    bool uses_ids_ = false;
};

// Maps an HID usage to a headset button (0 if not one we use).
uint8_t hid_usage_to_button(uint16_t page, uint16_t usage);

}  // namespace wt
