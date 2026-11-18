#include "adpcm.h"

// Intel/DVI/IMA-ADPCM Index Table
static const int8_t indexTable[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

// Intel/DVI/IMA-ADPCM Step Size Table (89 entries)
static const int16_t stepsizeTable[89] = {
    7,     8,     9,    10,    11,    12,    13,    14,    16,    17,
    19,    21,    23,    25,    28,    31,    34,    37,    41,    45,
    50,    55,    60,    66,    73,    80,    88,    97,   107,   118,
    130,   143,   157,   173,   190,   209,   230,   253,   279,   307,
    337,   371,   408,   449,   494,   544,   598,   658,   724,   796,
    876,   963,  1060,  1166,  1282,  1411,  1552,  1707,  1878,  2066,
    2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,
    5894,  6484,  7132,  7845,  8630,  9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

void Adpcm::initState(AdpcmState& state) {
    state.prevsample = 0;
    state.previndex = 0;
}

static inline uint8_t encodeSample(int16_t sample, AdpcmState& state) {
    int32_t valprev = state.prevsample;
    int32_t index = state.previndex;
    int32_t step = stepsizeTable[index];

    int32_t diff = sample - valprev;
    uint8_t delta = 0;

    if (diff < 0) {
        delta = 8;
        diff = -diff;
    }

    int32_t vpdiff = step >> 3;
    if (diff >= step) {
        delta |= 4;
        diff -= step;
        vpdiff += step;
    }
    step >>= 1;
    if (diff >= step) {
        delta |= 2;
        diff -= step;
        vpdiff += step;
    }
    step >>= 1;
    if (diff >= step) {
        delta |= 1;
        vpdiff += step;
    }

    if (delta & 8)
        valprev -= vpdiff;
    else
        valprev += vpdiff;

    // Saturate to 16-bit range
    if (valprev > 32767) valprev = 32767;
    else if (valprev < -32768) valprev = -32768;

    index += indexTable[delta];
    if (index < 0) index = 0;
    else if (index > 88) index = 88;

    state.prevsample = (int16_t)valprev;
    state.previndex = (int8_t)index;

    return delta;
}

static inline int16_t decodeSample(uint8_t delta, AdpcmState& state) {
    int32_t valprev = state.prevsample;
    int32_t index = state.previndex;
    int32_t step = stepsizeTable[index];

    int32_t vpdiff = step >> 3;
    if (delta & 4) vpdiff += step;
    if (delta & 2) vpdiff += (step >> 1);
    if (delta & 1) vpdiff += (step >> 2);

    if (delta & 8)
        valprev -= vpdiff;
    else
        valprev += vpdiff;

    // Saturate to 16-bit range
    if (valprev > 32767) valprev = 32767;
    else if (valprev < -32768) valprev = -32768;

    index += indexTable[delta];
    if (index < 0) index = 0;
    else if (index > 88) index = 88;

    state.prevsample = (int16_t)valprev;
    state.previndex = (int8_t)index;

    return (int16_t)valprev;
}

size_t Adpcm::encode(const int16_t* inSamples, size_t sampleCount,
                     uint8_t* outBytes, AdpcmState& state) {
    if (!inSamples || !outBytes || sampleCount == 0) return 0;

    size_t outIdx = 0;
    for (size_t i = 0; i < sampleCount; i += 2) {
        uint8_t delta0 = encodeSample(inSamples[i], state);
        uint8_t delta1 = 0;
        if (i + 1 < sampleCount) {
            delta1 = encodeSample(inSamples[i + 1], state);
        }
        outBytes[outIdx++] = (delta1 << 4) | (delta0 & 0x0F);
    }
    return outIdx;
}

size_t Adpcm::decode(const uint8_t* inBytes, size_t byteCount,
                     int16_t* outSamples, AdpcmState& state) {
    if (!inBytes || !outSamples || byteCount == 0) return 0;

    size_t outIdx = 0;
    for (size_t i = 0; i < byteCount; i++) {
        uint8_t byteVal = inBytes[i];
        uint8_t delta0 = byteVal & 0x0F;
        uint8_t delta1 = (byteVal >> 4) & 0x0F;

        outSamples[outIdx++] = decodeSample(delta0, state);
        outSamples[outIdx++] = decodeSample(delta1, state);
    }
    return outIdx;
}
