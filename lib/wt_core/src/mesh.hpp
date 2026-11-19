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
    uint8_t  mac[6] = {};       // 6-byte MAC for unicast delivery with hardware retries
    char     name[12] = {};
    int8_t   rssi = 0;          // of the last packet heard directly from this peer
    uint8_t  hops = 0;          // 0 = heard directly
    uint32_t last_ms = 0;
    bool     talking = false;
    uint32_t relay_id = 0;      // ID of intermediate relay node (if hops > 0)
    int8_t   uplink_rssi = 0;   // Piggybacked Uplink RSSI reported by this peer
    ReplayWindow replay;
};

// Highest session seen per sender, kept much longer than the peer table so a frame from an
// old session can never be replayed after its sender was pruned or evicted. Entries are
// only written after a frame authenticated, so an attacker without the key cannot fill it.
// Forgotten after kForgetMs of silence. Trade-off: after that, one old frame from that sender
// could be replayed once; but a node whose flash was erased (its session restarts at a new
// random value, possibly lower) is heard again after at most this long.
class SessionMemory {
public:
    static constexpr int kCap = 64;
    static constexpr uint32_t kForgetMs = 10u * 60u * 1000u;
    // False if (session, seq) is not newer than the newest frame remembered for sender.
    bool newer(uint32_t sender, uint32_t session, uint32_t seq, uint32_t now_ms) const;
    void put(uint32_t sender, uint32_t session, uint32_t seq, uint32_t now_ms);

private:
    struct E { uint32_t id = 0, session = 0, seq = 0, last_ms = 0; bool used = false; };
    E e_[kCap];
};

class PeerTable {
public:
    // Before decrypting: is this (sender, session, seq) new? Cheap filter for floods.
    bool fresh(const Header& h, uint32_t now_ms) const;
    // After the packet authenticated: records it. Returns false if it was a duplicate.
    bool accept(const Header& h, uint8_t hops, int8_t rssi, uint32_t now_ms, const uint8_t* mac = nullptr, uint32_t relay_id = 0);
    void set_name(uint32_t id, const char* name);
    void set_talking(uint32_t id, bool on);
    void prune(uint32_t now_ms);
    Peer* find(uint32_t id);
    const Peer* begin() const { return peers_; }
    const Peer* end() const { return peers_ + kMaxPeers; }
    int count() const;
    bool should_relay(uint32_t sender_id, int8_t rx_rssi, uint32_t now_ms) const;
    bool has_weak_or_remote_peer(uint32_t now_ms) const;

private:
    Peer* slot_for(uint32_t id, uint32_t now_ms);
    Peer peers_[kMaxPeers];
    SessionMemory sessions_;
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
