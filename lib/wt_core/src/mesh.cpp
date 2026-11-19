#include "mesh.hpp"

#include <cstring>

namespace wt {

Peer* PeerTable::find(uint32_t id) {
    for (auto& p : peers_)
        if (p.used && p.id == id) return &p;
    return nullptr;
}

Peer* PeerTable::slot_for(uint32_t id, uint32_t now_ms) {
    if (Peer* p = find(id)) return p;
    Peer* oldest = nullptr;
    for (auto& p : peers_) {
        if (!p.used) {
            oldest = &p;
            break;
        }
        if (!oldest || now_ms - p.last_ms > now_ms - oldest->last_ms) oldest = &p;
    }
    *oldest = Peer();
    oldest->used = true;
    oldest->id = id;
    return oldest;
}

bool SessionMemory::newer(uint32_t sender, uint32_t session, uint32_t seq, uint32_t now_ms) const {
    for (const auto& x : e_) {
        if (!x.used || x.id != sender) continue;
        if (now_ms - x.last_ms > kForgetMs) return true;
        return session > x.session || (session == x.session && seq > x.seq);
    }
    return true;
}

void SessionMemory::put(uint32_t sender, uint32_t session, uint32_t seq, uint32_t now_ms) {
    E* slot = nullptr;
    for (auto& x : e_)
        if (x.used && x.id == sender) { slot = &x; break; }
    if (!slot)
        for (auto& x : e_)
            if (!x.used) { slot = &x; break; }
    if (!slot) {  // full: forget the sender heard from longest ago
        slot = &e_[0];
        for (auto& x : e_)
            if (now_ms - x.last_ms > now_ms - slot->last_ms) slot = &x;
    }
    bool fresh_slot = !slot->used || slot->id != sender || now_ms - slot->last_ms > kForgetMs;
    if (fresh_slot || session > slot->session || (session == slot->session && seq > slot->seq)) {
        slot->session = session;
        slot->seq = seq;
    }
    slot->id = sender;
    slot->used = true;
    slot->last_ms = now_ms;
}

bool PeerTable::fresh(const Header& h, uint32_t now_ms) const {
    for (const auto& p : peers_)
        if (p.used && p.id == h.sender) return p.replay.fresh(h.session, h.seq);
    // Not in the table (pruned or evicted): without its 64-frame window, accept only frames
    // newer than the newest one remembered. Relayed copies of older frames are dropped.
    return sessions_.newer(h.sender, h.session, h.seq, now_ms);
}

bool PeerTable::accept(const Header& h, uint8_t hops, int8_t rssi, uint32_t now_ms, const uint8_t* mac, uint32_t relay_id) {
    if (!fresh(h, now_ms)) return false;
    Peer* p = find(h.sender);
    bool is_new = (p == nullptr);
    if (!p) {
        p = slot_for(h.sender, now_ms);
        // New slot: seed the window from memory so frames at or below the remembered seq
        // stay rejected.
        p->replay.accept(h.session, h.seq);
    } else if (!p->replay.accept(h.session, h.seq)) {
        return false;
    }
    sessions_.put(h.sender, h.session, h.seq, now_ms);
    if (hops == 0) {
        p->hops = 0;
        p->rssi = rssi;
        p->relay_id = 0;
        if (mac) memcpy(p->mac, mac, 6);
    } else if (is_new || p->hops != 0 || (now_ms - p->last_ms > 5000)) {
        p->hops = hops;
        p->relay_id = relay_id;
    }
    p->last_ms = now_ms;
    return true;
}

void PeerTable::set_name(uint32_t id, const char* name) {
    if (Peer* p = find(id)) {
        strncpy(p->name, name, sizeof(p->name) - 1);
        p->name[sizeof(p->name) - 1] = 0;
    }
}

void PeerTable::set_talking(uint32_t id, bool on) {
    if (Peer* p = find(id)) p->talking = on;
}

void PeerTable::prune(uint32_t now_ms) {
    for (auto& p : peers_)
        if (p.used && now_ms - p.last_ms > kPeerTimeoutMs) p = Peer();
}

int PeerTable::count() const {
    int n = 0;
    for (const auto& p : peers_) n += p.used;
    return n;
}

bool RelayQueue::push(const uint8_t* frame, size_t len, uint32_t due_ms) {
    Header h;
    if (!read_header(frame, len, h)) return false;
    for (auto& it : items_) {
        if (it.used) continue;
        it.used = true;
        it.due = due_ms;
        it.sender = h.sender;
        it.session = h.session;
        it.seq = h.seq;
        it.len = static_cast<uint8_t>(len);
        memcpy(it.data, frame, len);
        return true;
    }
    return false;  // full: drop the relay, never block the radio
}

void RelayQueue::cancel(const uint8_t* frame, size_t len) {
    Header h;
    if (!read_header(frame, len, h)) return;
    for (auto& it : items_) {
        if (!it.used || it.sender != h.sender || it.session != h.session || it.seq != h.seq) continue;
        if (it.len != len) continue;
        if (memcmp(it.data, frame, 3) == 0 && memcmp(it.data + 4, frame + 4, len - 4) == 0)
            it.used = false;
    }
}

size_t RelayQueue::pop_due(uint32_t now_ms, uint8_t* out) {
    Item* best = nullptr;
    for (auto& it : items_)
        if (it.used && static_cast<int32_t>(now_ms - it.due) >= 0 &&
            (!best || static_cast<int32_t>(it.due - best->due) < 0))
            best = &it;
    if (!best) return 0;
    best->used = false;
    memcpy(out, best->data, best->len);
    return best->len;
}

int RelayQueue::size() const {
    int n = 0;
    for (const auto& it : items_) n += it.used;
    return n;
}

}  // namespace wt
