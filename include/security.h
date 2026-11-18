#ifndef SECURITY_H
#define SECURITY_H

#include <Arduino.h>
#include <mbedtls/aes.h>
#include "config.h"
#include "messages.h"

// ============================================================
// Hardware AES-128 Cryptographic Security Module
// ============================================================

class Security {
public:
    static Security& getInstance();

    /// Initialize AES-128 hardware engine with secret key
    bool init(const uint8_t* key = nullptr);

    /// Encrypt payload buffer in CTR mode (hardware accelerated)
    bool encrypt(const uint8_t* in, uint8_t* out, size_t len,
                 const uint8_t* iv, uint8_t seq, uint8_t type);

    /// Decrypt payload buffer in CTR mode (hardware accelerated)
    bool decrypt(const uint8_t* in, uint8_t* out, size_t len,
                 const uint8_t* iv, uint8_t seq, uint8_t type);

    /// Generate cryptographically sound 8-byte random IV
    void generateIv(uint8_t* outIv);

private:
    Security();
    ~Security();

    mbedtls_aes_context _aesEncCtx;
    mbedtls_aes_context _aesDecCtx;
    portMUX_TYPE        _secMux;
    bool                _initialized;
    uint8_t             _key[16];

    void buildNonce(uint8_t nonce[16], const uint8_t* iv, uint8_t seq, uint8_t type);
};

#endif // SECURITY_H
