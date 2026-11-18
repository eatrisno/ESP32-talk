#include "security.h"
#include <esp_random.h>
#include <cstring>

static const uint8_t DEFAULT_AES_KEY[16] = AES_KEY_BYTES;

Security& Security::getInstance() {
    static Security instance;
    return instance;
}

Security::Security() : _initialized(false) {
    _secMux = portMUX_INITIALIZER_UNLOCKED;
    mbedtls_aes_init(&_aesEncCtx);
    mbedtls_aes_init(&_aesDecCtx);
}

Security::~Security() {
    mbedtls_aes_free(&_aesEncCtx);
    mbedtls_aes_free(&_aesDecCtx);
}

bool Security::init(const uint8_t* key) {
    portENTER_CRITICAL(&_secMux);
    if (key != nullptr) {
        memcpy(_key, key, 16);
    } else {
        memcpy(_key, DEFAULT_AES_KEY, 16);
    }

    mbedtls_aes_init(&_aesEncCtx);
    mbedtls_aes_init(&_aesDecCtx);
    // In AES-CTR mode, encryption key schedule is used for both encrypt and decrypt!
    int ret1 = mbedtls_aes_setkey_enc(&_aesEncCtx, _key, 128);
    int ret2 = mbedtls_aes_setkey_enc(&_aesDecCtx, _key, 128);
    if (ret1 != 0 || ret2 != 0) {
        Serial.printf("[SECURITY] ERROR: mbedtls_aes_setkey_enc failed (%d, %d)\n", ret1, ret2);
        _initialized = false;
        portEXIT_CRITICAL(&_secMux);
        return false;
    }

    _initialized = true;
    portEXIT_CRITICAL(&_secMux);
    Serial.println("[SECURITY] AES-128 Hardware Encryption Module Initialized (Thread-Safe TX/RX)");
    return true;
}

void Security::buildNonce(uint8_t nonce[16], const uint8_t* iv, uint8_t seq, uint8_t type) {
    // 16-byte Nonce Counter layout:
    // bytes 0..7  : 8-byte random IV
    // byte 8      : packet sequence number
    // byte 9      : message type
    // bytes 10..11: protocol magic 0x57, 0x4B ('W', 'K')
    // bytes 12..15: 32-bit counter initialized to 0
    memcpy(nonce, iv, 8);
    nonce[8]  = seq;
    nonce[9]  = type;
    nonce[10] = ESPNOW_PAYLOAD_MAGIC_0;
    nonce[11] = ESPNOW_PAYLOAD_MAGIC_1;
    nonce[12] = 0;
    nonce[13] = 0;
    nonce[14] = 0;
    nonce[15] = 0;
}

void Security::generateIv(uint8_t* outIv) {
    if (!outIv) return;
    for (int i = 0; i < 8; i += 4) {
        uint32_t r = esp_random();
        memcpy(outIv + i, &r, 4);
    }
}

bool Security::encrypt(const uint8_t* in, uint8_t* out, size_t len,
                       const uint8_t* iv, uint8_t seq, uint8_t type) {
    if (!_initialized || !in || !out || len == 0 || !iv) return false;

    uint8_t nonce[16];
    uint8_t streamBlock[16] = {0};
    size_t ncOff = 0;

    buildNonce(nonce, iv, seq, type);

    int ret = mbedtls_aes_crypt_ctr(&_aesEncCtx, len, &ncOff, nonce, streamBlock, in, out);
    return (ret == 0);
}

bool Security::decrypt(const uint8_t* in, uint8_t* out, size_t len,
                       const uint8_t* iv, uint8_t seq, uint8_t type) {
    if (!_initialized || !in || !out || len == 0 || !iv) return false;

    uint8_t nonce[16];
    uint8_t streamBlock[16] = {0};
    size_t ncOff = 0;

    buildNonce(nonce, iv, seq, type);

    int ret = mbedtls_aes_crypt_ctr(&_aesDecCtx, len, &ncOff, nonce, streamBlock, in, out);
    return (ret == 0);
}
