#pragma once
#include <cstddef>
#include <cstdint>

namespace wt {

#pragma pack(push, 1)
struct RangeReq {
    uint32_t token;
    uint16_t seq;
    uint16_t total;
};

struct RangeResp {
    uint32_t token;
    uint16_t seq;
    int8_t   req_rssi;
};
#pragma pack(pop)

struct RangeStats {
    uint16_t sent = 0;
    uint16_t received = 0;
    int32_t  sum_uplink_rssi = 0;
    int32_t  sum_downlink_rssi = 0;
    int8_t   min_rssi = 0;
    int8_t   max_rssi = -128;

    void reset(uint16_t total_sent = 0) {
        sent = total_sent;
        received = 0;
        sum_uplink_rssi = 0;
        sum_downlink_rssi = 0;
        min_rssi = 0;
        max_rssi = -128;
    }

    void add_sample(int8_t uplink_rssi, int8_t downlink_rssi) {
        received++;
        sum_uplink_rssi += uplink_rssi;
        sum_downlink_rssi += downlink_rssi;
        if (downlink_rssi < min_rssi || received == 1) min_rssi = downlink_rssi;
        if (downlink_rssi > max_rssi || received == 1) max_rssi = downlink_rssi;
    }

    float loss_percent() const {
        if (sent == 0) return 0.0f;
        if (received >= sent) return 0.0f;
        return (1.0f - static_cast<float>(received) / static_cast<float>(sent)) * 100.0f;
    }

    float avg_downlink_rssi() const {
        if (received == 0) return 0.0f;
        return static_cast<float>(sum_downlink_rssi) / static_cast<float>(received);
    }

    float avg_uplink_rssi() const {
        if (received == 0) return 0.0f;
        return static_cast<float>(sum_uplink_rssi) / static_cast<float>(received);
    }

    float margin_db(float sensitivity_floor = -108.0f) const {
        if (received == 0) return 0.0f;
        return avg_downlink_rssi() - sensitivity_floor;
    }
};

}  // namespace wt
