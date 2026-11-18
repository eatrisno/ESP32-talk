#include "button.hpp"

namespace wt {

Press ButtonDetector::update(bool down, uint32_t now_ms) {
    if (down != raw_) {
        raw_ = down;
        changed_ms_ = now_ms;
    }
    if (raw_ != stable_ && now_ms - changed_ms_ >= kDebounceMs) {
        stable_ = raw_;
        if (stable_) {
            pressed_ms_ = now_ms;
            long_sent_ = false;
        } else if (!long_sent_) {
            return Press::Click;
        }
    }
    if (stable_ && !long_sent_ && now_ms - pressed_ms_ >= kLongMs) {
        long_sent_ = true;
        return Press::Long;
    }
    return Press::None;
}

}  // namespace wt
