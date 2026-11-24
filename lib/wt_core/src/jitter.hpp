#pragma once
// Per-talker jitter buffer. Frames are placed by frame number, so out-of-order and
// duplicate packets are handled. A missing frame is concealed (last frame faded out),
// and the play-out depth adapts slowly to keep the two nodes' clocks from drifting apart.
#include <cstddef>
#include <cstdint>

#include "adpcm.hpp"
#include "protocol.hpp"

namespace wt {

class JitterBuffer {
public:
    static constexpr int kSlots = 16;          // 640 ms of capacity
    static constexpr int kTargetDepth = 2;     // 80 ms start-up delay (low latency)

    void reset();
    // Stores a received audio frame (decoded immediately with the state it carries).
    void put(const AudioPayload& a);
    // Pops one kFrameSamples frame. Returns false while idle/pre-buffering
    // (out is not touched then).
    bool pop(int16_t* out);
    bool active() const { return started_; }
    uint32_t lost() const { return lost_; }          // frames that never arrived
    uint32_t late() const { return late_; }          // frames that arrived after their turn
    uint32_t underruns() const { return underruns_; }  // waited for a not-yet-due frame
    uint32_t drift_skips() const { return drift_skips_; }
    // Frames buffered ahead of the play position (latency = depth * 40 ms).
    int depth() const;

private:
    struct Slot {
        bool     full = false;
        uint16_t frame_no = 0;
        int16_t  pcm[kFrameSamples];
    };
    bool pop_slot(Slot& s, int16_t* out);
    void conceal(int16_t* out);
    Slot     slots_[kSlots];
    bool     started_ = false;
    bool     have_any_ = false;
    uint16_t next_ = 0;        // next frame number to play
    uint16_t newest_ = 0;
    int      concealed_ = 0;   // consecutive concealed frames
    int      over_ = 0;        // consecutive pops with the buffer above target
    int16_t  last_[kFrameSamples] = {};
    int16_t  tail_ = 0;        // last sample sent to the output
    uint32_t lost_ = 0, late_ = 0, underruns_ = 0, drift_skips_ = 0;
    uint8_t  voice_id_ = 0;
};

}  // namespace wt
