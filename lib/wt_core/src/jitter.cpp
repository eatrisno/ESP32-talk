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
    over_ = 0;
    tail_ = 0;
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

int JitterBuffer::depth() const {
    if (!have_any_) return 0;
    int d = dist(newest_, next_) + 1;
    return d < 0 ? 0 : d;
}

bool JitterBuffer::pop(int16_t* out) {
    if (!have_any_) return false;
    int depth = dist(newest_, next_) + 1;
    if (!started_) {
        if (depth < kTargetDepth) return false;
        started_ = true;
    }
    Slot& s = slots_[next_ % kSlots];
    bool have = s.full && s.frame_no == next_;
    if (!have && depth <= 0) {
        // Underrun: the next frame is not overdue yet (sender slightly slower than us, or
        // late network). Play concealment but keep waiting for that same frame instead of
        // skipping it, so a slow sender costs a pause, not a lost frame.
        conceal(out);
        underruns_++;
        if (concealed_ > 6) {  // 240 ms of nothing: the talker has gone
            reset();
            return false;
        }
        return true;
    }
    // Sender faster than us, or a stall that left extra delay behind: once the buffer has
    // been above target for a whole second (25 frames), drop one frame, then measure again.
    // A short burst (frames bunched up by the radio) drains by itself and is never cut.
    if (depth > kTargetDepth + 1) over_++;
    else over_ = 0;
    if (over_ >= 25 && have) {
        over_ = 0;
        s.full = false;
        next_++;
        drift_skips_++;
        Slot& n = slots_[next_ % kSlots];
        if (!(n.full && n.frame_no == next_)) {
            conceal(out);
            lost_++;
            next_++;
            return true;
        }
        return pop_slot(n, out);
    }
    if (have) return pop_slot(s, out);
    conceal(out);  // frame really lost: newer ones are already here
    lost_++;
    next_++;
    if (concealed_ > 6) {
        reset();
        return false;
    }
    return true;
}

bool JitterBuffer::pop_slot(Slot& s, int16_t* out) {
    if (concealed_ > 0) {
        // Coming back from concealment: cross-fade over the first 5 ms so there is no step.
        const size_t kFade = kFrameSamples / 8;
        for (size_t i = 0; i < kFade; i++) {
            float a = static_cast<float>(i + 1) / (kFade + 1);
            s.pcm[i] = static_cast<int16_t>(s.pcm[i] * a + tail_ * (1.f - a));
        }
    }
    memcpy(out, s.pcm, sizeof(s.pcm));
    memcpy(last_, s.pcm, sizeof(s.pcm));
    tail_ = s.pcm[kFrameSamples - 1];
    s.full = false;
    concealed_ = 0;
    next_++;
    return true;
}

void JitterBuffer::conceal(int16_t* out) {
    // First lost frame: repeat the last one, fading from its end level to 0 so neither edge
    // steps. Further lost frames: silence.
    concealed_++;
    if (concealed_ == 1) {
        for (size_t i = 0; i < kFrameSamples; i++) {
            float g = 1.f - static_cast<float>(i + 1) / kFrameSamples;
            // Start from where playback stopped (tail_), then follow the repeated frame.
            float x = i < kFrameSamples / 8
                          ? tail_ + (last_[i] - tail_) * (static_cast<float>(i + 1) / (kFrameSamples / 8))
                          : last_[i];
            out[i] = static_cast<int16_t>(x * g);
        }
    } else {
        memset(out, 0, kFrameSamples * sizeof(int16_t));
    }
    tail_ = out[kFrameSamples - 1];
}

}  // namespace wt
