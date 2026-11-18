#pragma once

#include <cstddef>
#include <cstdint>

namespace wt {

enum PromptId : uint8_t {
    PROMPT_NONE = 0,
    PROMPT_ZERO,
    PROMPT_ONE,
    PROMPT_TWO,
    PROMPT_THREE,
    PROMPT_FOUR,
    PROMPT_FIVE,
    PROMPT_DEVICE,
    PROMPT_DEVICES,
    PROMPT_NEARBY,
    PROMPT_MIDDLE,
    PROMPT_FAR,
    PROMPT_MIC_ON,
    PROMPT_MIC_OFF,
    PROMPT_VM_RECORDING,
    PROMPT_VM_SENT,
    PROMPT_VM_OUTBOX,
    PROMPT_VM_INCOMING,
    PROMPT_EMERGENCY_ON,
    PROMPT_EMERGENCY_OFF,
    PROMPT_EMERGENCY_RX,
    PROMPT_CONNECTED,
    PROMPT_DISCONNECTED,
    PROMPT_READY,
    PROMPT_COUNT
};

struct PromptData {
    const int16_t* samples;
    size_t count;
};

const PromptData& get_prompt(PromptId id);

class PromptQueue {
public:
    PromptQueue() = default;
    void push(PromptId id);
    void clear();
    bool busy() const;
    bool render(int16_t* out, size_t n, float vol_scale = 1.0f);
private:
    static constexpr int kMaxQueue = 8;
    PromptId queue_[kMaxQueue] = {};
    int head_ = 0, tail_ = 0;
    size_t sample_pos_ = 0;
};

}  // namespace wt
