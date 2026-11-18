#include "jitter.hpp"

#include <cstring>

namespace wt {
namespace {

// Signed distance a - b for wrapping 16-bit frame numbers.
int16_t dist(uint16_t a, uint16_t b) { return static_cast<int16_t>(a - b); }

}  // namespace

void JitterBuffer::reset() {
    for (auto& s : slots_) s.full = false;
    started_ = false;
    have_any_ = false;
    concealed_ = 0;
    memset(last_, 0, sizeof(last_));
}

void JitterBuffer::put(const AudioPayload& a) {
    if (a.data_len != kAdpcmBytes) return;
    if (have_any_ && a.voice_id != voice_id_) reset();  // talker started a new over
    if (!have_any_) {
        have_any_ = true;
        voice_id_ = a.voice_id;
        next_ = a.frame_no;
        newest_ = a.frame_no;
    }
    if (started_ && dist(a.frame_no, next_) < 0) {
        late_++;
        return;
    }
    if (dist(a.frame_no, next_) >= kSlots) {
        // Far ahead (long gap or clock drift): drop the backlog and resync.
        reset();
        have_any_ = true;
        voice_id_ = a.voice_id;
        next_ = a.frame_no;
        newest_ = a.frame_no;
    }
    Slot& s = slots_[a.frame_no % kSlots];
    if (s.full && s.frame_no == a.frame_no) return;  // duplicate
    AdpcmState st;
    st.predictor = a.adpcm_pred;
    st.index = static_cast<int8_t>(a.adpcm_index);
    adpcm_decode(a.data, a.data_len, s.pcm, st);
    s.frame_no = a.frame_no;
    s.full = true;
    if (dist(a.frame_no, newest_) > 0) newest_ = a.frame_no;
}

bool JitterBuffer::pop(int16_t* out) {
    if (!have_any_) return false;
    int depth = dist(newest_, next_) + 1;
    if (!started_) {
        if (depth < kTargetDepth) return false;
        started_ = true;
    }
    // Clock drift: if the buffer keeps growing, skip one frame to catch up.
    if (depth > kTargetDepth + 3) {
        Slot& skip = slots_[next_ % kSlots];
        if (skip.full && skip.frame_no == next_) skip.full = false;
        next_++;
    }
    Slot& s = slots_[next_ % kSlots];
    if (s.full && s.frame_no == next_) {
        memcpy(out, s.pcm, sizeof(s.pcm));
        memcpy(last_, s.pcm, sizeof(s.pcm));
        s.full = false;
        concealed_ = 0;
    } else {
        // Lost frame: repeat the last one at half level, then silence.
        lost_++;
        concealed_++;
        float g = concealed_ == 1 ? 0.5f : 0.f;
        for (size_t i = 0; i < kFrameSamples; i++) {
            out[i] = static_cast<int16_t>(last_[i] * g);
            last_[i] = out[i];
        }
        if (concealed_ > 6) {  // 240 ms of nothing: the talker has gone
            reset();
            return false;
        }
    }
    next_++;
    return true;
}

}  // namespace wt
