#include "radio.hpp"

#include <cstring>

#include "config.hpp"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "psa/crypto.h"

namespace radio {
namespace {

const char* TAG = "radio";
const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// Returns true if the MAC contains at least one non-zero byte (i.e. is not the zero MAC).
static inline bool mac_valid(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) if (mac[i] != 0) return true;
    return false;
}
constexpr psa_algorithm_t kAlg = PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, wt::kTagLen);

struct Raw {
    uint8_t len;
    int8_t  rssi;
    uint8_t mac[6];
    uint8_t data[wt::kMaxFrame];
};
struct Tx {
    uint8_t type;
    uint8_t len;
    uint8_t dest_mac[6];
    uint8_t plain[wt::kMaxPlain];
};

static bool s_repeater_mode = false;
QueueHandle_t s_rxq, s_txq;
SemaphoreHandle_t s_peers_mtx;
TaskHandle_t s_task;
psa_key_id_t s_key;
wt::PeerTable s_peers;
wt::RelayQueue s_relay;
RxHandler s_on_rx;
uint32_t s_self, s_session, s_seq;
uint8_t s_channel;
Stats s_stats;
portMUX_TYPE s_stats_mux = portMUX_INITIALIZER_UNLOCKED;  // Wi-Fi task + radio task both count

uint32_t now_ms() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

// Wi-Fi task context: copy and return.
void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len < static_cast<int>(wt::kHeaderLen + wt::kTagLen) || len > static_cast<int>(wt::kMaxFrame)) return;
    if (data[0] != wt::kMagic) return;
    Raw r;
    r.len = static_cast<uint8_t>(len);
    r.rssi = info->rx_ctrl ? static_cast<int8_t>(info->rx_ctrl->rssi) : 0;
    if (info->src_addr) memcpy(r.mac, info->src_addr, 6);
    else memset(r.mac, 0, 6);
    memcpy(r.data, data, len);
    if (xQueueSend(s_rxq, &r, 0) != pdTRUE) {  // queue full: shed load
        portENTER_CRITICAL(&s_stats_mux);
        s_stats.rx_drop++;
        portEXIT_CRITICAL(&s_stats_mux);
    }
    xTaskNotifyGive(s_task);
}

void on_sent(const esp_now_send_info_t*, esp_now_send_status_t st) {
    if (st != ESP_NOW_SEND_SUCCESS) {
        portENTER_CRITICAL(&s_stats_mux);
        s_stats.tx_fail++;
        portEXIT_CRITICAL(&s_stats_mux);
    }
}

static RateMode s_rate_mode = cfg::kLongRange ? RATE_LR_250K : RATE_1M_L;

static void fill_rate_config(RateMode mode, esp_now_rate_config_t& rc) {
    memset(&rc, 0, sizeof(rc));
    switch (mode) {
        case RATE_1M_L:
            rc.phymode = WIFI_PHY_MODE_11B;
            rc.rate = WIFI_PHY_RATE_1M_L;
            break;
        case RATE_LR_500K:
            rc.phymode = WIFI_PHY_MODE_LR;
            rc.rate = WIFI_PHY_RATE_LORA_500K;
            break;
        case RATE_LR_250K:
        default:
            rc.phymode = WIFI_PHY_MODE_LR;
            rc.rate = WIFI_PHY_RATE_LORA_250K;
            break;
    }
}

// Cache of unicast MACs already registered with ESP-NOW (radio task only; esp_now_del_peer is never called).
constexpr int kRegMacCap = wt::kMaxPeers * 2;
uint8_t s_reg_macs[kRegMacCap][6];
int s_reg_mac_count = 0;

void cache_registered_mac(const uint8_t* mac) {
    if (s_reg_mac_count < kRegMacCap) {
        memcpy(s_reg_macs[s_reg_mac_count++], mac, 6);
    }
}

void register_unicast_peer(const uint8_t* mac) {
    if (!mac || memcmp(mac, kBroadcast, 6) == 0 || !mac_valid(mac)) return;
    for (int i = 0; i < s_reg_mac_count; i++) {
        if (memcmp(s_reg_macs[i], mac, 6) == 0) return;
    }
    if (esp_now_is_peer_exist(mac)) {
        cache_registered_mac(mac);
        return;
    }

    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.ifidx = WIFI_IF_STA;
    p.channel = 0;  // follow current channel
    esp_err_t err = esp_now_add_peer(&p);
    if (err == ESP_OK) {
        cache_registered_mac(mac);
        esp_now_rate_config_t rc;
        fill_rate_config(s_rate_mode, rc);
        esp_err_t rerr = esp_now_set_peer_rate_config(mac, &rc);
        if (rerr != ESP_OK) {
            ESP_LOGW(TAG, "failed to set rate config for peer: %s", esp_err_to_name(rerr));
        }
        ESP_LOGI(TAG, "registered unicast peer %02x:%02x:%02x:%02x:%02x:%02x (rate: %s)",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], rate_mode_str(s_rate_mode));
    } else if (err == ESP_ERR_ESPNOW_EXIST) {
        cache_registered_mac(mac);
    }
}

bool seal(const wt::Header& h, const uint8_t* plain, size_t len, uint8_t* out, size_t* out_len) {
    wt::write_header(h, out);
    uint8_t nonce[wt::kNonceLen], aad[wt::kHeaderLen];
    wt::make_nonce(h, nonce);
    wt::make_aad(out, aad);
    size_t n = 0;
    psa_status_t st = psa_aead_encrypt(s_key, kAlg, nonce, sizeof nonce, aad, sizeof aad, plain, len,
                                       out + wt::kHeaderLen, wt::kMaxFrame - wt::kHeaderLen, &n);
    if (st != PSA_SUCCESS) return false;
    *out_len = wt::kHeaderLen + n;
    return true;
}

bool open(const uint8_t* frame, size_t len, const wt::Header& h, uint8_t* plain, size_t* plain_len) {
    uint8_t nonce[wt::kNonceLen], aad[wt::kHeaderLen];
    wt::make_nonce(h, nonce);
    wt::make_aad(frame, aad);
    psa_status_t st = psa_aead_decrypt(s_key, kAlg, nonce, sizeof nonce, aad, sizeof aad,
                                       frame + wt::kHeaderLen, len - wt::kHeaderLen, plain,
                                       wt::kMaxPlain, plain_len);
    return st == PSA_SUCCESS;
}

bool send_espnow(const uint8_t* mac, const uint8_t* frame, size_t len, bool allow_retry) {
    esp_err_t e = esp_now_send(mac, frame, len);
    if (e == ESP_ERR_ESPNOW_NO_MEM && allow_retry) {
        vTaskDelay(1);
        e = esp_now_send(mac, frame, len);
    }
    portENTER_CRITICAL(&s_stats_mux);
    if (e == ESP_OK) s_stats.tx++;
    else s_stats.tx_fail++;
    portEXIT_CRITICAL(&s_stats_mux);
    return e == ESP_OK;
}

bool transmit(const uint8_t* frame, size_t len, bool force_broadcast = false) {
    constexpr uint32_t kUnicastFreshMs = 10000;  // avoid 7x hardware retries to offline peers
    uint8_t direct_macs[wt::kMaxPeers][6];
    int n_direct = 0;
    if (!force_broadcast) {
        xSemaphoreTake(s_peers_mtx, portMAX_DELAY);
        uint32_t now = now_ms();
        for (const wt::Peer* p = s_peers.begin(); p != s_peers.end(); ++p) {
            if (p->used && (now - p->last_ms <= kUnicastFreshMs)) {
                if (mac_valid(p->mac) && n_direct < wt::kMaxPeers) {
                    bool already = false;
                    for (int j = 0; j < n_direct; j++) {
                        if (memcmp(direct_macs[j], p->mac, 6) == 0) { already = true; break; }
                    }
                    if (!already) {
                        memcpy(direct_macs[n_direct++], p->mac, 6);
                    }
                }
            }
        }
        xSemaphoreGive(s_peers_mtx);
    }

    wt::Header h;
    bool has_hdr = wt::read_header(frame, len, h);
    bool is_audio = has_hdr && h.type == wt::MSG_AUDIO;
    bool is_bcast_type = has_hdr &&
        (h.type == wt::MSG_BEACON || h.type == wt::MSG_DISTRESS || h.type == wt::MSG_PAIR_BEACON);
    // Group audio (2+ direct peers) uses broadcast so 1 frame serves all peers without 2*N unicast retries.
    if (is_bcast_type || (is_audio && n_direct > 1)) force_broadcast = true;

    bool allow_retry = !is_audio;
    if (n_direct == 1 && !force_broadcast) {
        register_unicast_peer(direct_macs[0]);
        return send_espnow(direct_macs[0], frame, len, allow_retry);
    }
    if (n_direct > 1 && n_direct <= 3 && !force_broadcast) {
        bool any_ok = false;
        for (int i = 0; i < n_direct; i++) {
            register_unicast_peer(direct_macs[i]);
            any_ok |= send_espnow(direct_macs[i], frame, len, allow_retry);
        }
        return any_ok;
    }
    return send_espnow(kBroadcast, frame, len, allow_retry);
}

void handle_raw(Raw& r) {
    wt::Header h;
    if (!wt::read_header(r.data, r.len, h)) return;
    if (h.sender == s_self) return;                      // our own frame, relayed back
    // ttl is outside the authenticated data (relays decrement it). Clamp it so an attacker
    // rebroadcasting a real frame with ttl 255 cannot carry it past the designed 2 relays.
    if (h.ttl > wt::kDefaultTtl) {
        h.ttl = wt::kDefaultTtl;
        r.data[3] = wt::kDefaultTtl;
    }
    // A neighbour already relayed this one: our pending copy is redundant.
    s_relay.cancel(r.data, r.len);
    xSemaphoreTake(s_peers_mtx, portMAX_DELAY);
    bool fresh = s_peers.fresh(h, now_ms());
    xSemaphoreGive(s_peers_mtx);
    if (!fresh) {
        s_stats.dup++;
        return;
    }
    Rx rx;
    if (!open(r.data, r.len, h, rx.plain, &rx.len)) {
        s_stats.auth_fail++;
        return;
    }
    rx.hdr = h;
    rx.hops = static_cast<uint8_t>(wt::kDefaultTtl - h.ttl);
    rx.rssi = r.rssi;
    memcpy(rx.mac, r.mac, 6);
    rx.relay_id = (rx.hops > 0) ? (static_cast<uint32_t>(r.mac[2]) << 24 |
                                   static_cast<uint32_t>(r.mac[3]) << 16 |
                                   static_cast<uint32_t>(r.mac[4]) << 8  |
                                   static_cast<uint32_t>(r.mac[5])) : 0;
    xSemaphoreTake(s_peers_mtx, portMAX_DELAY);
    bool accepted = s_peers.accept(h, rx.hops, r.rssi, now_ms(), r.mac, rx.relay_id);
    int num_peers = s_peers.count();
    xSemaphoreGive(s_peers_mtx);
    if (!accepted) return;
    s_stats.rx++;
    if (rx.hops == 0) {
        register_unicast_peer(r.mac);
    }
    // Smart Selective Relay & Repeater Mode:
    // When repeater mode is enabled, it ALWAYS relays any frame with ttl > 1 immediately (1-2ms hold-off).
    // Emergency / control frames (SOS, channel switch) always relay if ttl > 1.
    // Routine frames (audio, beacon, ping/pong, range req/resp) relay if there is a weak or multi-hop peer.
    if (h.ttl > 1 && (num_peers > 1 || s_repeater_mode)) {
        bool do_relay = true;
        if (!s_repeater_mode && h.type != wt::MSG_DISTRESS && h.type != wt::MSG_CHANNEL_SWITCH) {
            xSemaphoreTake(s_peers_mtx, portMAX_DELAY);
            do_relay = s_peers.should_relay(h.sender, r.rssi, now_ms());
            xSemaphoreGive(s_peers_mtx);
        }
        if (do_relay) {
            r.data[3] = h.ttl - 1;
            // Expedite forwarding to 1..2 ms in repeater mode for high-speed seamless forwarding
            uint32_t base = s_repeater_mode ? 1 : (r.rssi < -80 ? 2 : (r.rssi < -65 ? 5 : 8));
            s_relay.push(r.data, r.len, now_ms() + base + (s_repeater_mode ? (esp_random() % 2) : (esp_random() % 5)));
        }
    }
    if (s_on_rx) s_on_rx(rx);
}

void task(void*) {
    Raw r;
    Tx t;
    uint8_t frame[wt::kMaxFrame];
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
        while (xQueueReceive(s_rxq, &r, 0) == pdTRUE) handle_raw(r);
        while (xQueueReceive(s_txq, &t, 0) == pdTRUE) {
            while (xQueueReceive(s_rxq, &r, 0) == pdTRUE) handle_raw(r);
            wt::Header h;
            h.type = t.type;
            h.ttl = wt::kDefaultTtl;
            h.sender = s_self;
            h.session = s_session;
            h.seq = ++s_seq;
            size_t n;
            if (seal(h, t.plain, t.len, frame, &n)) {
                bool is_explicit_dest = mac_valid(t.dest_mac);
                if (is_explicit_dest) {
                    register_unicast_peer(t.dest_mac);
                    send_espnow(t.dest_mac, frame, n, t.type != wt::MSG_AUDIO);
                } else {
                    bool force_bcast = (t.type == wt::MSG_BEACON || t.type == wt::MSG_DISTRESS);
                    bool ok = transmit(frame, n, force_bcast);
                    // Adaptive 2x Audio Redundancy: only duplicate when a peer link is weak/relayed,
                    // primary TX succeeded, and TX queue has no backlog.
                    if (cfg::kDuplicateAudioFrames && t.type == wt::MSG_AUDIO && ok &&
                        uxQueueMessagesWaiting(s_txq) <= 1) {
                        xSemaphoreTake(s_peers_mtx, portMAX_DELAY);
                        bool need_dup = s_peers.has_weak_or_remote_peer(now_ms(), -85);
                        xSemaphoreGive(s_peers_mtx);
                        if (need_dup) {
                            vTaskDelay(pdMS_TO_TICKS(1));
                            while (xQueueReceive(s_rxq, &r, 0) == pdTRUE) handle_raw(r);
                            transmit(frame, n, false);
                        }
                    }
                }
            }
        }
        uint32_t now = now_ms();
        size_t n;
        while ((n = s_relay.pop_due(now, frame)) > 0) {
            if (transmit(frame, n, true)) s_stats.relayed++;
        }
    }
}

bool import_key() {
    uint8_t key[16];
    const char* hex = cfg::kNetworkKeyHex;
    if (strlen(hex) != 32) return false;
    for (int i = 0; i < 16; i++) {
        auto nib = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        int hi = nib(hex[2 * i]), lo = nib(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return false;
        key[i] = static_cast<uint8_t>(hi << 4 | lo);
    }
    if (psa_crypto_init() != PSA_SUCCESS) return false;
    psa_key_attributes_t a = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_usage_flags(&a, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&a, kAlg);
    psa_set_key_type(&a, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&a, 128);
    psa_status_t st = psa_import_key(&a, key, sizeof key, &s_key);
    memset(key, 0, sizeof key);
    return st == PSA_SUCCESS;
}

}  // namespace

bool start(uint8_t ch, uint32_t session, RxHandler on_rx) {
    if (!import_key()) {
        ESP_LOGE(TAG, "network key invalid (need 32 hex chars)");
        return false;
    }
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    s_self = static_cast<uint32_t>(mac[2]) << 24 |
             static_cast<uint32_t>(mac[3]) << 16 |
             static_cast<uint32_t>(mac[4]) << 8  |
             static_cast<uint32_t>(mac[5]);
    s_session = session;
    s_on_rx = on_rx;
    s_rxq = xQueueCreate(24, sizeof(Raw));
    s_txq = xQueueCreate(8, sizeof(Tx));
    s_peers_mtx = xSemaphoreCreateMutex();
    // The task must exist before on_recv can be called: it notifies s_task.
    xTaskCreatePinnedToCore(task, "radio", 6144, nullptr, 7, &s_task, 0);

    wifi_init_config_t wc = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wc));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Configure manual country info to enable Channel 13 at full power.
    // Note: wifi_country_t.max_tx_power is informational (quarter-dBm, same unit as
    // esp_wifi_set_max_tx_power). The actual TX power cap is set via esp_wifi_set_max_tx_power.
    wifi_country_t country = {
        .cc = "JP",
        .schan = 1,
        .nchan = 14,
        .max_tx_power = cfg::kTxPowerQdBm, // quarter-dBm, informational field
        .policy = WIFI_COUNTRY_POLICY_MANUAL
    };
    esp_wifi_set_country(&country);

    ESP_ERROR_CHECK(esp_wifi_start());
    set_channel(ch);

    uint8_t proto = cfg::kLongRange ? (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR)
                                    : (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, proto));

    // Disable power saving to keep radio awake at full readiness
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(on_recv);
    esp_now_register_send_cb(on_sent);

    // Set ESP-NOW PHY rate: Pure LR 250k (-108 dBm max sensitivity, maximum range)
    esp_now_rate_config_t rc;
    fill_rate_config(s_rate_mode, rc);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, rc.rate);

    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, kBroadcast, 6);
    p.ifidx = WIFI_IF_STA;
    p.channel = 0;  // follow the current channel
    ESP_ERROR_CHECK(esp_now_add_peer(&p));
    esp_err_t rerr = esp_now_set_peer_rate_config(kBroadcast, &rc);
    if (rerr != ESP_OK) {
        ESP_LOGE(TAG, "failed to set broadcast rate config: %s", esp_err_to_name(rerr));
    }

    // Maximize TX power AFTER esp_now_init() and all rate config: ESP-NOW init can reset
    // the power level internally, so this must be the last radio setting applied.
    esp_wifi_set_max_tx_power(cfg::kTxPowerQdBm);

    int8_t pwr = 0;
    esp_wifi_get_max_tx_power(&pwr);
    ESP_LOGI(TAG, "node %08lx session %lu ch %u (phy %s, tx %d = %.2f dBm)",
             (unsigned long)s_self, (unsigned long)session, ch,
             rate_mode_str(s_rate_mode),
             pwr, pwr * 0.25f);
    if (cfg::kDefaultKey)
        ESP_LOGW(TAG, "using the PUBLIC default network key: anyone can listen. See README.");
    return true;
}

bool set_channel(uint8_t ch) {
    if (ch < 1 || ch > 13) return false;
    if (esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
    s_channel = ch;
    return true;
}

uint8_t channel() { return s_channel; }
uint32_t self_id() { return s_self; }

void set_repeater_mode(bool on) {
    s_repeater_mode = on;
    if (s_peers_mtx) {
        xSemaphoreTake(s_peers_mtx, portMAX_DELAY);
        s_peers.set_repeater_mode(on);
        xSemaphoreGive(s_peers_mtx);
    } else {
        s_peers.set_repeater_mode(on);
    }
}

bool is_repeater_mode() {
    return s_repeater_mode;
}

bool send_to(wt::MsgType type, const uint8_t* dest_mac, const uint8_t* plain, size_t len) {
    if (len > wt::kMaxPlain) return false;
    // Bound real-time audio backlog to 3 frames (120 ms) so TX stalls never accumulate lag.
    if (type == wt::MSG_AUDIO && uxQueueMessagesWaiting(s_txq) >= 3) return false;
    Tx t;
    t.type = type;
    t.len = static_cast<uint8_t>(len);
    if (dest_mac) memcpy(t.dest_mac, dest_mac, 6);
    else memset(t.dest_mac, 0, 6);
    memcpy(t.plain, plain, len);
    if (xQueueSend(s_txq, &t, 0) != pdTRUE) return false;
    xTaskNotifyGive(s_task);
    return true;
}

bool send(wt::MsgType type, const uint8_t* plain, size_t len) {
    return send_to(type, nullptr, plain, len);
}

wt::PeerTable& peers() { return s_peers; }
void peers_lock() { if (s_peers_mtx) xSemaphoreTake(s_peers_mtx, portMAX_DELAY); }
void peers_unlock() { if (s_peers_mtx) xSemaphoreGive(s_peers_mtx); }
Stats stats() {
    portENTER_CRITICAL(&s_stats_mux);
    Stats s = s_stats;
    portEXIT_CRITICAL(&s_stats_mux);
    return s;
}

const char* rate_mode_str(RateMode mode) {
    switch (mode) {
        case RATE_1M_L: return "1M_L (1 Mbps)";
        case RATE_LR_500K: return "LR 500k (-105 dBm)";
        case RATE_LR_250K: return "LR 250k (-108 dBm)";
        default: return "Unknown";
    }
}

RateMode rate_mode() {
    return s_rate_mode;
}

bool set_rate_mode(RateMode mode) {
    esp_now_rate_config_t rc;
    fill_rate_config(mode, rc);
    esp_err_t err = esp_now_set_peer_rate_config(kBroadcast, &rc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to set broadcast rate config: %s", esp_err_to_name(err));
        return false;
    }
    esp_wifi_config_espnow_rate(WIFI_IF_STA, rc.rate);

    xSemaphoreTake(s_peers_mtx, portMAX_DELAY);
    for (const wt::Peer* p = s_peers.begin(); p != s_peers.end(); ++p) {
        if (p->used && p->hops == 0) {
            if (mac_valid(p->mac)) {
                esp_now_set_peer_rate_config(p->mac, &rc);
            }
        }
    }
    xSemaphoreGive(s_peers_mtx);
    s_rate_mode = mode;
    ESP_LOGI(TAG, "PHY rate switched to: %s", rate_mode_str(mode));
    return true;
}

}  // namespace radio
