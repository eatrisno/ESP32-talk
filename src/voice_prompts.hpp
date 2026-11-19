#pragma once

#include <cstddef>
#include <cstdint>

namespace wt {

enum PromptId : uint8_t {
    PROMPT_NONE = 0,
    PROMPT_READY,
    PROMPT_POWER_OFF,
    PROMPT_CONNECTED,
    PROMPT_DISCONNECTED,
    PROMPT_MIC_ON,
    PROMPT_MIC_OFF,
    PROMPT_MENU,
    PROMPT_MENU_VOICE_NOTE,
    PROMPT_MENU_RADAR,
    PROMPT_MENU_WALK,
    PROMPT_MENU_PTT,
    PROMPT_MENU_HANDS_FREE,
    PROMPT_MENU_CHANNEL,
    PROMPT_WALK_ON,
    PROMPT_WALK_OFF,
    PROMPT_PTT_ON,
    PROMPT_HANDS_FREE_ON,
    PROMPT_VM_RECORDING,
    PROMPT_VM_SENT,
    PROMPT_VM_OUTBOX,
    PROMPT_VM_INCOMING,
    PROMPT_EMERGENCY_ON,
    PROMPT_EMERGENCY_OFF,
    PROMPT_EMERGENCY_RX,
    PROMPT_RADAR_NONE,
    PROMPT_RADAR_1_NEAR,
    PROMPT_RADAR_1_MID,
    PROMPT_RADAR_1_FAR,
    PROMPT_RADAR_2_NEAR,
    PROMPT_RADAR_2_MID,
    PROMPT_RADAR_2_FAR,
    PROMPT_RADAR_MANY_NEAR,
    PROMPT_RADAR_MANY_MID,
    PROMPT_RADAR_MANY_FAR,
    PROMPT_NUM_1,
    PROMPT_NUM_2,
    PROMPT_NUM_3,
    PROMPT_NUM_4,
    PROMPT_NUM_5,
    PROMPT_NUM_6,
    PROMPT_NUM_7,
    PROMPT_NUM_8,
    PROMPT_NUM_9,
    PROMPT_NUM_10,
    PROMPT_NUM_11,
    PROMPT_NUM_12,
    PROMPT_NUM_13,
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
    int16_t fade_last_ = 0;
    int fade_remain_ = 0;
};

}  // namespace wt
