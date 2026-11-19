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
    uint8_t plain[wt::kMaxPlain];
};

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

void register_unicast_peer(const uint8_t* mac) {
    if (!mac || esp_now_is_peer_exist(mac)) return;
    if (memcmp(mac, kBroadcast, 6) == 0) return;
    bool non_zero = false;
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0) { non_zero = true; break; }
    }
    if (!non_zero) return;

    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.ifidx = WIFI_IF_STA;
    p.channel = 0;  // follow current channel
    esp_err_t err = esp_now_add_peer(&p);
    if (err == ESP_OK) {
        esp_now_rate_config_t rc = {};
        rc.phymode = cfg::kLongRange ? WIFI_PHY_MODE_LR : WIFI_PHY_MODE_11G;
        rc.rate = cfg::kLongRange ? WIFI_PHY_RATE_LORA_250K : WIFI_PHY_RATE_6M;
        esp_err_t rerr = esp_now_set_peer_rate_config(mac, &rc);
        if (rerr != ESP_OK) {
            ESP_LOGW(TAG, "failed to set rate config for peer: %s", esp_err_to_name(rerr));
        }
        ESP_LOGI(TAG, "registered unicast peer %02x:%02x:%02x:%02x:%02x:%02x (hardware retries enabled)",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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

void transmit(const uint8_t* frame, size_t len, bool force_broadcast = false) {
    uint8_t direct_macs[wt::kMaxPeers][6];
    int n_direct = 0;
    if (!force_broadcast) {
        xSemaphoreTake(s_peers_mtx, portMAX_DELAY);
        uint32_t now = now_ms();
        for (const wt::Peer* p = s_peers.begin(); p != s_peers.end(); ++p) {
            if (p->used && p->hops == 0 && (now - p->last_ms <= wt::kPeerTimeoutMs)) {
                // If link is marginal (RSSI < -80 dBm or Uplink < -85 dBm), avoid blocking on 7x hardware MAC retries;
                // fallback to broadcast so frames are sent instantly and forwarded by the mesh relay!
                if (p->rssi < -80 || (p->uplink_rssi != 0 && p->uplink_rssi < -85)) {
                    continue;
                }
                bool has_mac = false;
                for (int i = 0; i < 6; i++) {
                    if (p->mac[i] != 0) { has_mac = true; break; }
                }
                if (has_mac && n_direct < wt::kMaxPeers) {
                    memcpy(direct_macs[n_direct++], p->mac, 6);
                }
            }
        }
        xSemaphoreGive(s_peers_mtx);
    }

    // If 1 direct peer, use unicast (with 802.11 MAC ACK and hardware retries).
    // If multiple direct peers and transmitting audio, use broadcast to reach everyone in 1 frame (7 ms total),
    // avoiding airtime saturation. For control/data unicast, keep unicast loop.
    wt::Header h;
    bool is_audio = wt::read_header(frame, len, h) && (h.type == wt::MSG_AUDIO);
    if (n_direct == 1 && !force_broadcast) {
        esp_err_t e = esp_now_send(direct_macs[0], frame, len);
        if (e == ESP_ERR_ESPNOW_NO_MEM) {
            vTaskDelay(1);
            e = esp_now_send(direct_macs[0], frame, len);
        }
        portENTER_CRITICAL(&s_stats_mux);
        if (e == ESP_OK) s_stats.tx++;
        else s_stats.tx_fail++;
        portEXIT_CRITICAL(&s_stats_mux);
    } else if (n_direct > 1 && !force_broadcast && !is_audio) {
        for (int i = 0; i < n_direct; i++) {
            esp_err_t e = esp_now_send(direct_macs[i], frame, len);
            if (e == ESP_ERR_ESPNOW_NO_MEM) {
                vTaskDelay(1);
                e = esp_now_send(direct_macs[i], frame, len);
            }
            portENTER_CRITICAL(&s_stats_mux);
            if (e == ESP_OK) s_stats.tx++;
            else s_stats.tx_fail++;
            portEXIT_CRITICAL(&s_stats_mux);
        }
    } else {
        // Broadcast (for beacons, audio to multiple peers, discovery, or when no direct peers)
        esp_err_t e = esp_now_send(kBroadcast, frame, len);
        if (e == ESP_ERR_ESPNOW_NO_MEM) {
            vTaskDelay(1);
            e = esp_now_send(kBroadcast, frame, len);
        }
        portENTER_CRITICAL(&s_stats_mux);
        if (e == ESP_OK) s_stats.tx++;
        else s_stats.tx_fail++;
        portEXIT_CRITICAL(&s_stats_mux);
    }
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
    // Smart Selective Relay:
    // Emergency / control frames (SOS, channel switch) always relay if ttl > 1.
    // Audio and beacons only relay if there is a weak or multi-hop peer, or if the sender was weak.
    if (h.ttl > 1 && num_peers > 1) {
        bool do_relay = true;
        if (h.type == wt::MSG_AUDIO || h.type == wt::MSG_BEACON) {
            xSemaphoreTake(s_peers_mtx, portMAX_DELAY);
            do_relay = s_peers.should_relay(h.sender, r.rssi, now_ms());
            xSemaphoreGive(s_peers_mtx);
        }
        if (do_relay) {
            r.data[3] = h.ttl - 1;
            // 2..12 ms random hold-off; weaker receivers relay sooner because they are more
            // likely at the edge of the group, where relays matter.
            uint32_t base = r.rssi < -80 ? 2 : (r.rssi < -65 ? 5 : 8);
            s_relay.push(r.data, r.len, now_ms() + base + esp_random() % 5);
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
            wt::Header h;
            h.type = t.type;
            h.ttl = wt::kDefaultTtl;
            h.sender = s_self;
            h.session = s_session;
            h.seq = ++s_seq;
            size_t n;
            if (seal(h, t.plain, t.len, frame, &n)) {
                bool force_bcast = (t.type == wt::MSG_BEACON);
                transmit(frame, n, force_bcast);
                // Adaptive Audio Duplication: only duplicate if at least one peer is weak/remote
                if (cfg::kDuplicateAudioFrames && t.type == wt::MSG_AUDIO) {
                    xSemaphoreTake(s_peers_mtx, portMAX_DELAY);
                    bool need_dup = s_peers.has_weak_or_remote_peer(now_ms());
                    xSemaphoreGive(s_peers_mtx);
                    if (need_dup) {
                        vTaskDelay(pdMS_TO_TICKS(1));
                        transmit(frame, n, false);
                    }
                }
            }
        }
        uint32_t now = now_ms();
        size_t n;
        while ((n = s_relay.pop_due(now, frame)) > 0) {
            transmit(frame, n, true);
            s_stats.relayed++;
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

    // Configure manual country info (JP: Ch 1-14, up to 21 dBm) to enable Channel 13 at maximum power
    wifi_country_t country = {
        .cc = "JP",
        .schan = 1,
        .nchan = 14,
        .max_tx_power = cfg::kTxPowerQdBm,
        .policy = WIFI_COUNTRY_POLICY_MANUAL
    };
    esp_wifi_set_country(&country);

    // Protocol: LR for ESP-NOW + standard 802.11
    uint8_t proto = cfg::kLongRange ? (WIFI_PROTOCOL_LR | WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G) : (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, proto));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Disable power saving to keep radio awake at full readiness
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Force default ESP-NOW PHY rate (250 kbps for extreme sensitivity)
    esp_wifi_config_espnow_rate(WIFI_IF_STA, cfg::kLongRange ? WIFI_PHY_RATE_LORA_250K : WIFI_PHY_RATE_6M);

    // Maximize TX power (+20 dBm / +21 dBm)
    esp_wifi_set_max_tx_power(cfg::kTxPowerQdBm);
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(on_recv);
    esp_now_register_send_cb(on_sent);
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, kBroadcast, 6);
    p.ifidx = WIFI_IF_STA;
    p.channel = 0;  // follow the current channel
    ESP_ERROR_CHECK(esp_now_add_peer(&p));
    esp_now_rate_config_t rc = {};
    rc.phymode = cfg::kLongRange ? WIFI_PHY_MODE_LR : WIFI_PHY_MODE_11G;
    rc.rate = cfg::kLongRange ? WIFI_PHY_RATE_LORA_250K : WIFI_PHY_RATE_6M;
    esp_err_t rerr = esp_now_set_peer_rate_config(kBroadcast, &rc);
    if (rerr != ESP_OK) {
        ESP_LOGE(TAG, "failed to set broadcast rate config: %s", esp_err_to_name(rerr));
    }
    set_channel(ch);
    int8_t pwr = 0;
    esp_wifi_get_max_tx_power(&pwr);
    ESP_LOGI(TAG, "node %08lx session %lu ch %u (phy %s, tx %d = %.2f dBm)",
             (unsigned long)s_self, (unsigned long)session, ch,
             cfg::kLongRange ? "ESP-NOW LR 250k (-108 dBm)" : "11g 6M (-90 dBm)",
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

bool send(wt::MsgType type, const uint8_t* plain, size_t len) {
    if (len > wt::kMaxPlain) return false;
    Tx t;
    t.type = type;
    t.len = static_cast<uint8_t>(len);
    memcpy(t.plain, plain, len);
    if (xQueueSend(s_txq, &t, 0) != pdTRUE) return false;
    xTaskNotifyGive(s_task);
    return true;
}

wt::PeerTable& peers() { return s_peers; }
void peers_lock() { xSemaphoreTake(s_peers_mtx, portMAX_DELAY); }
void peers_unlock() { xSemaphoreGive(s_peers_mtx); }
Stats stats() {
    portENTER_CRITICAL(&s_stats_mux);
    Stats s = s_stats;
    portEXIT_CRITICAL(&s_stats_mux);
    return s;
}

}  // namespace radio
