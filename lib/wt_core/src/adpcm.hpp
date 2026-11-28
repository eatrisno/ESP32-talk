#pragma once
// IMA/DVI ADPCM, 4 bits per sample. Encoder state is sent with every frame so a lost
// packet never desynchronises the decoder.
#include <cstddef>
#include <cstdint>

// IRAM_ATTR is defined by ESP-IDF; on host (native tests) it is a no-op.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

namespace wt {

struct AdpcmState {
    int16_t predictor = 0;
    int8_t  index     = 0;
};

// Encodes n samples (n MUST be even) into n/2 bytes. Low nibble = first sample.
// adpcm_encode silently drops the last sample if n is odd — guard with the assert below.
size_t IRAM_ATTR adpcm_encode(const int16_t* in, size_t n, uint8_t* out, AdpcmState& st);
// Decodes `bytes` bytes into 2*bytes samples.
size_t IRAM_ATTR adpcm_decode(const uint8_t* in, size_t bytes, int16_t* out, AdpcmState& st);

}  // namespace wt
