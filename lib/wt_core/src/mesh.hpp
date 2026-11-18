#pragma once
// Peer table and flood-relay rules. Every node rebroadcasts each new packet once while
// ttl > 1, after a short random delay; if it hears another node relay the same packet
// during that delay it cancels its own copy (counter-based flooding), so a fully
// connected group does not pay for relays it does not need.
#include <cstddef>
#include <cstdint>

#include "protocol.hpp"

namespace wt {

constexpr uint8_t kDefaultTtl = 3;   // origin + up to 2 relays
constexpr int     kMaxPeers = 8;
constexpr uint32_t kPeerTimeoutMs = 15000;

struct Peer {
    bool     used = false;
    uint32_t id = 0;
    char     name[12] = {};
    int8_t   rssi = 0;          // of the last packet heard directly from this peer
    uint8_t  hops = 0;          // 0 = heard directly
    uint32_t last_ms = 0;
    bool     talking = false;
    ReplayWindow replay;
};

class PeerTable {
public:
    // Before decrypting: is this (sender, session, seq) new? Cheap filter for floods.
    bool fresh(const Header& h) const;
    // After the packet authenticated: records it. Returns false if it was a duplicate.
    bool accept(const Header& h, uint8_t hops, int8_t rssi, uint32_t now_ms);
    void set_name(uint32_t id, const char* name);
    void set_talking(uint32_t id, bool on);
    void prune(uint32_t now_ms);
    Peer* find(uint32_t id);
    const Peer* begin() const { return peers_; }
    const Peer* end() const { return peers_ + kMaxPeers; }
    int count() const;

private:
    Peer* slot_for(uint32_t id, uint32_t now_ms);
    Peer peers_[kMaxPeers];
};

// Pending relays with cancellation. Owned by the radio task.
class RelayQueue {
public:
    static constexpr int kCap = 16;
    // Schedules frame for relay at due_ms. Frame must already have ttl decremented.
    bool push(const uint8_t* frame, size_t len, uint32_t due_ms);
    // Someone else already relayed this frame: drop our copy. Only a byte-identical frame
    // (ttl aside) counts, so a forged header cannot cancel relays.
    void cancel(const uint8_t* frame, size_t len);
    // Pops the next due frame. Returns its length or 0.
    size_t pop_due(uint32_t now_ms, uint8_t* out);
    int size() const;

private:
    struct Item {
        bool used = false;
        uint32_t due = 0, sender = 0, session = 0, seq = 0;
        uint8_t len = 0;
        uint8_t data[kMaxFrame];
    };
    Item items_[kCap];
};

}  // namespace wt
