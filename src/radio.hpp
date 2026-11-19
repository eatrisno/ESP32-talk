#pragma once
// ESP-NOW transport: authenticated broadcast frames plus flood relay. One task owns the
// radio; the Wi-Fi receive callback only copies frames into a queue.
#include <cstddef>
#include <cstdint>

#include "mesh.hpp"
#include "protocol.hpp"

namespace radio {

struct Rx {
    wt::Header hdr;
    uint8_t    hops;          // kDefaultTtl - ttl
    int8_t     rssi;
    uint32_t   relay_id;      // 0 if direct, or ID of intermediate relay node
    uint8_t    plain[wt::kMaxPlain];
    size_t     len;
};

// Called from the radio task for every new authenticated frame (never for duplicates).
using RxHandler = void (*)(const Rx& rx);

bool start(uint8_t channel, uint32_t session, RxHandler on_rx);
bool set_channel(uint8_t channel);
uint8_t channel();
uint32_t self_id();

// Encrypts and broadcasts. Safe from any task. Returns false if the queue is full.
bool send(wt::MsgType type, const uint8_t* plain, size_t len);

wt::PeerTable& peers();   // read from the radio task or under peers_lock()
void peers_lock();
void peers_unlock();

struct Stats {
    uint32_t tx, rx, relayed, auth_fail, dup, rx_drop, tx_fail;
};
Stats stats();

}  // namespace radio
