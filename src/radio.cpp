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

uint32_t now_ms() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

// Wi-Fi task context: copy and return.
void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len < static_cast<int>(wt::kHeaderLen + wt::kTagLen) || len > static_cast<int>(wt::kMaxFrame)) return;
    if (data[0] != wt::kMagic) return;
    Raw r;
    r.len = static_cast<uint8_t>(len);
    r.rssi = info->rx_ctrl ? static_cast<int8_t>(info->rx_ctrl->rssi) : 0;
    memcpy(r.data, data, len);
    if (xQueueSend(s_rxq, &r, 0) != pdTRUE) s_stats.dup++;  // queue full: shed load
    xTaskNotifyGive(s_task);
}

void on_sent(const esp_now_send_info_t*, esp_now_send_status_t st) {
    if (st != ESP_NOW_SEND_SUCCESS) s_stats.tx_fail++;
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

void transmit(const uint8_t* frame, size_t len) {
    esp_err_t e = esp_now_send(kBroadcast, frame, len);
    if (e == ESP_ERR_ESPNOW_NO_MEM) {  // Wi-Fi TX queue full: one short retry
        vTaskDelay(1);
        e = esp_now_send(kBroadcast, frame, len);
    }
    if (e == ESP_OK) s_stats.tx++;
    else s_stats.tx_fail++;
}

void handle_raw(Raw& r) {
    wt::Header h;
    if (!wt::read_header(r.data, r.len, h)) return;
    if (h.sender == s_self) return;                      // our own frame, relayed back
    // A neighbour already relayed this one: our pending copy is redundant.
    s_relay.cancel(h.sender, h.session, h.seq);
    xSemaphoreTake(s_peers_mtx, portMAX_DELAY);
    bool fresh = s_peers.fresh(h);
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
    rx.hops = h.ttl <= wt::kDefaultTtl ? static_cast<uint8_t>(wt::kDefaultTtl - h.ttl) : 0;
    rx.rssi = r.rssi;
    xSemaphoreTake(s_peers_mtx, portMAX_DELAY);
    bool accepted = s_peers.accept(h, rx.hops, r.rssi, now_ms());
    xSemaphoreGive(s_peers_mtx);
    if (!accepted) return;
    s_stats.rx++;
    if (h.ttl > 1) {
        r.data[3] = h.ttl - 1;
        // 2..12 ms random hold-off; weaker receivers relay sooner because they are more
        // likely at the edge of the group, where relays matter.
        uint32_t base = r.rssi < -80 ? 2 : (r.rssi < -65 ? 5 : 8);
        s_relay.push(r.data, r.len, now_ms() + base + esp_random() % 5);
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
            if (seal(h, t.plain, t.len, frame, &n)) transmit(frame, n);
        }
        uint32_t now = now_ms();
        size_t n;
        while ((n = s_relay.pop_due(now, frame)) > 0) {
            transmit(frame, n);
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
    s_self = mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5];
    s_session = session;
    s_on_rx = on_rx;
    s_rxq = xQueueCreate(24, sizeof(Raw));
    s_txq = xQueueCreate(8, sizeof(Tx));
    s_peers_mtx = xSemaphoreCreateMutex();

    wifi_init_config_t wc = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wc));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);
    uint8_t proto = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
    if (cfg::kLongRange) proto |= WIFI_PROTOCOL_LR;
    esp_wifi_set_protocol(WIFI_IF_STA, proto);
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
    rc.rate = cfg::kLongRange ? WIFI_PHY_RATE_LORA_500K : WIFI_PHY_RATE_6M;
    esp_now_set_peer_rate_config(kBroadcast, &rc);
    set_channel(ch);
    xTaskCreatePinnedToCore(task, "radio", 6144, nullptr, 7, &s_task, 0);
    ESP_LOGI(TAG, "node %08lx session %lu ch %u", (unsigned long)s_self, (unsigned long)session, ch);
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
Stats stats() { return s_stats; }

}  // namespace radio
