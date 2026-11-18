#include "hid_buttons.hpp"

namespace wt {
namespace {

constexpr uint16_t kPageTelephony = 0x0B;
constexpr uint16_t kPageConsumer = 0x0C;

uint32_t uval(const uint8_t* p, int n) {
    uint32_t v = 0;
    for (int i = 0; i < n; i++) v |= static_cast<uint32_t>(p[i]) << (8 * i);
    return v;
}

int32_t sval(const uint8_t* p, int n) {
    uint32_t v = uval(p, n);
    if (n == 1) return static_cast<int8_t>(v);
    if (n == 2) return static_cast<int16_t>(v);
    return static_cast<int32_t>(v);
}

uint32_t bits_at(const uint8_t* data, size_t len, uint32_t bit, uint8_t size) {
    uint32_t v = 0;
    for (uint8_t i = 0; i < size && i < 32; i++) {
        uint32_t b = bit + i;
        if (b / 8 >= len) break;
        if (data[b / 8] & (1u << (b % 8))) v |= 1u << i;
    }
    return v;
}

}  // namespace

uint8_t hid_usage_to_button(uint16_t page, uint16_t usage) {
    if (page == kPageConsumer) {
        switch (usage) {
            case 0xCD:  // Play/Pause
            case 0xB0:  // Play
            case 0xE2:  // Mute
                return HID_BTN_TALK;
            case 0xE9: return HID_BTN_VOL_UP;
            case 0xEA: return HID_BTN_VOL_DOWN;
        }
    } else if (page == kPageTelephony) {
        if (usage == 0x20 || usage == 0x2F) return HID_BTN_TALK_TOGGLE;  // Hook switch, Phone mute
    }
    return 0;
}

bool HidButtons::parse(const uint8_t* d, size_t len) {
    n_ = 0;
    uses_ids_ = false;
    // Global state
    uint16_t page = 0;
    int32_t lmin = 0;
    uint8_t rsize = 0, rcount = 0, rid = 0;
    // Bit position per report id. ponytail: static scratch, parse() is not reentrant
    // (called once per connect from the USB task).
    static uint16_t pos[256];
    for (auto& p : pos) p = 0;
    // Local state
    uint32_t usages[16];
    int nus = 0;
    uint32_t umin = 0, umax = 0;
    bool range = false;
    auto clear_local = [&] { nus = 0; range = false; umin = umax = 0; };
    // Usage with an explicit page in the high 16 bits overrides the global page.
    auto page_of = [&](uint32_t u) { return (u >> 16) ? static_cast<uint16_t>(u >> 16) : page; };

    size_t i = 0;
    while (i < len) {
        uint8_t b = d[i++];
        if (b == 0xFE) {  // long item
            if (i + 1 >= len) break;
            i += 2 + d[i];
            continue;
        }
        int sz = b & 3;
        if (sz == 3) sz = 4;
        if (i + sz > len) break;
        const uint8_t* v = d + i;
        i += sz;
        uint8_t type = (b >> 2) & 3, tag = b >> 4;
        if (type == 1) {  // global
            switch (tag) {
                case 0x0: page = static_cast<uint16_t>(uval(v, sz)); break;
                case 0x1: lmin = sval(v, sz); break;
                case 0x7: rsize = static_cast<uint8_t>(uval(v, sz)); break;
                case 0x8: rid = static_cast<uint8_t>(uval(v, sz)); uses_ids_ = true; break;
                case 0x9: rcount = static_cast<uint8_t>(uval(v, sz)); break;
            }
        } else if (type == 2) {  // local
            uint32_t u = uval(v, sz);
            if (sz == 4) {
                // full 32-bit usage keeps its page
            } else {
                u &= 0xFFFF;
            }
            switch (tag) {
                case 0x0: if (nus < 16) usages[nus++] = u; break;
                case 0x1: umin = u; range = true; break;
                case 0x2: umax = u; range = true; break;
            }
        } else if (type == 0) {  // main
            if (tag == 0x8) {  // Input
                uint32_t flags = uval(v, sz);
                bool constant = flags & 1, variable = flags & 2;
                uint16_t start = pos[rid];
                if (!constant) {
                    if (variable) {
                        for (uint8_t k = 0; k < rcount && n_ < kMax; k++) {
                            uint32_t u = range ? umin + k
                                               : (nus ? usages[k < nus ? k : nus - 1] : 0);
                            uint8_t btn = hid_usage_to_button(page_of(u), u & 0xFFFF);
                            if (btn)
                                f_[n_++] = {rid, static_cast<uint16_t>(start + k * rsize), rsize, 1,
                                            false, btn, 0, 0, 0};
                        }
                    } else {
                        uint32_t first = range ? umin : (nus ? usages[0] : 0);
                        uint16_t pg = page_of(first);
                        bool useful = false;
                        uint32_t last = range ? umax : first;
                        for (uint32_t u = first & 0xFFFF; u <= (last & 0xFFFF) && u - (first & 0xFFFF) < 1024; u++)
                            if (hid_usage_to_button(pg, static_cast<uint16_t>(u))) { useful = true; break; }
                        // ponytail: arrays with a usage list (not a range) are skipped; no
                        // headset seen uses them. Add a list lookup if one does.
                        if (!range && nus > 1) useful = false;
                        if (useful && n_ < kMax)
                            f_[n_++] = {rid, start, rsize, rcount, true, 0, pg,
                                        static_cast<uint16_t>(first & 0xFFFF), lmin};
                    }
                }
                pos[rid] = static_cast<uint16_t>(start + rsize * rcount);
            }
            clear_local();
        }
    }
    return n_ > 0;
}

uint8_t HidButtons::decode(const uint8_t* r, size_t len, uint8_t* mask) const {
    uint8_t id = 0;
    if (mask) *mask = 0;
    if (uses_ids_) {
        if (len < 1) return 0;
        id = r[0];
        r++;
        len--;
    }
    uint8_t out = 0;
    for (int i = 0; i < n_; i++) {
        const Field& f = f_[i];
        if (f.report_id != id) continue;
        if (mask) *mask |= f.array ? static_cast<uint8_t>(HID_BTN_TALK | HID_BTN_VOL_UP | HID_BTN_VOL_DOWN |
                                                          HID_BTN_TALK_TOGGLE)
                                   : f.button;
        if (!f.array) {
            if (bits_at(r, len, f.bit, f.size)) out |= f.button;
            continue;
        }
        for (uint8_t k = 0; k < f.count; k++) {
            int32_t val = static_cast<int32_t>(bits_at(r, len, f.bit + k * f.size, f.size));
            if (val == 0 && f.lmin == 0) continue;  // empty slot
            int32_t u = f.usage_min + (val - f.lmin);
            if (u < 0 || u > 0xFFFF) continue;
            out |= hid_usage_to_button(f.page, static_cast<uint16_t>(u));
        }
    }
    return out;
}

}  // namespace wt
