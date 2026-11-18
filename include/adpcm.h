#ifndef ADPCM_H
#define ADPCM_H

#include <cstdint>
#include <cstddef>

// ============================================================
// IMA-ADPCM 4:1 Audio Compression Engine
// Compresses 16-bit linear PCM (8000 Hz) down to 4-bit nibbles
// Reduces airtime bitrate from 128 kbps to 32 kbps (75% packet reduction)
// ============================================================

struct AdpcmState {
    int16_t prevsample; // Predicted audio sample (-32768 to 32767)
    int8_t  previndex;  // Step size table index (0 to 88)
};

class Adpcm {
public:
    /// Reset encoder / decoder state
    static void initState(AdpcmState& state);

    /// Encode 16-bit linear PCM samples into 4-bit IMA-ADPCM
    /// inSamples: array of 16-bit PCM samples
    /// sampleCount: total samples to encode
    /// outBytes: destination buffer (must hold at least sampleCount / 2 bytes)
    /// state: encoder continuity state
    /// Returns: encoded byte count
    static size_t encode(const int16_t* inSamples, size_t sampleCount,
                         uint8_t* outBytes, AdpcmState& state);

    /// Decode 4-bit IMA-ADPCM bytes into 16-bit linear PCM samples
    /// inBytes: array of compressed ADPCM bytes
    /// byteCount: total compressed bytes to decode
    /// outSamples: destination buffer (must hold at least byteCount * 2 samples)
    /// state: decoder continuity state
    /// Returns: decoded sample count
    static size_t decode(const uint8_t* inBytes, size_t byteCount,
                         int16_t* outSamples, AdpcmState& state);
};

#endif // ADPCM_H
