#pragma once
// IMA/DVI ADPCM, 4 bits per sample. Encoder state is sent with every frame so a lost
// packet never desynchronises the decoder.
#include <cstddef>
#include <cstdint>

namespace wt {

struct AdpcmState {
    int16_t predictor = 0;
    int8_t  index     = 0;
};

// Encodes n samples (n even) into n/2 bytes. Low nibble = first sample.
size_t adpcm_encode(const int16_t* in, size_t n, uint8_t* out, AdpcmState& st);
// Decodes `bytes` bytes into 2*bytes samples.
size_t adpcm_decode(const uint8_t* in, size_t bytes, int16_t* out, AdpcmState& st);

}  // namespace wt
