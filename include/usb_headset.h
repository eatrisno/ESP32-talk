#ifndef USB_HEADSET_H
#define USB_HEADSET_H

#include <Arduino.h>
#include <cstdint>
#include <cstddef>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "adpcm.h"

// ============================================================
// USB Type-C Headset Audio Manager (UAC Host)
// Real-Time Full Duplex Streaming Engine + Digital Audio Mixer
// ============================================================

// Audio Specifications for Walkie-Talkie Voice Transmission
#define HEADSET_TARGET_SAMPLE_RATE  8000   // 8 kHz voice bandwidth
#define HEADSET_TARGET_CHANNELS     1      // Mono
#define HEADSET_TARGET_BITS         16     // 16-bit PCM

// Live Streaming Chunk Parameters
#define STREAM_CHUNK_SAMPLES        400    // 50ms at 8000 Hz
#define STREAM_CHUNK_BYTES          (STREAM_CHUNK_SAMPLES * sizeof(int16_t)) // 800 bytes PCM
#define STREAM_ADPCM_BYTES          (STREAM_CHUNK_SAMPLES / 2)              // 200 bytes ADPCM (4:1)
#define JITTER_BUFFER_SLOTS         8      // 8 slots * 50ms = 400ms buffer capacity
#define JITTER_PREBUFFER_CHUNKS     2      // Prebuffer 2 chunks (100ms) for jitter immunity

// DC Blocker: pole at 31457/32768 ≈ 0.9596 → cutoff ≈ 50 Hz at 8 kHz (removes DC/hum, preserves voice)
struct DcBlocker {
    int32_t prevIn;
    int32_t prevOut;

    void reset() {
        prevIn = 0;
        prevOut = 0;
    }

    inline int16_t process(int16_t sample) {
        // y[n] = x[n] - x[n-1] + 0.9596 * y[n-1]   (high-pass, fc ≈ 50 Hz at 8 kHz)
        int32_t acc = ((31457 * prevOut) >> 15) + (int32_t)sample - prevIn;
        if (acc > 32767) acc = 32767;
        else if (acc < -32768) acc = -32768;
        prevIn = sample;
        prevOut = acc;
        return (int16_t)acc;
    }
};

// Jitter buffer slot for 50ms PCM chunks
struct JitterSlot {
    int16_t  samples[STREAM_CHUNK_SAMPLES];
    uint16_t sampleCount;
};

// Up to 2 concurrent incoming voice streams mixed simultaneously in Full Duplex
#define MAX_AUDIO_STREAMS 2

// Soft-Knee Limiter (preserves vocal clarity and prevents digital clipping)
static inline int16_t applyDigitalGainAndLimit(int16_t sample, int16_t gainMult = 1) {
    int32_t val = (int32_t)sample * gainMult;
    if (val > 28000) {
        val = 28000 + (val - 28000) / 4;
    } else if (val < -28000) {
        val = -28000 + (val + 28000) / 4;
    }
    if (val > 32767) val = 32767;
    else if (val < -32768) val = -32768;
    return (int16_t)val;
}

struct AudioRxChannel {
    bool              active;
    uint8_t           mac[6];
    uint8_t           msgId;
    uint8_t           lastChunkIdx;
    uint32_t          lastRxMs;
    AdpcmState        adpcmState;
    JitterSlot        slots[JITTER_BUFFER_SLOTS];
    volatile uint8_t  writeIdx;
    volatile uint8_t  readIdx;
    volatile uint8_t  count;
    volatile bool     prebuffering;

    void reset() {
        active = false;
        memset(mac, 0, 6);
        msgId = 255;
        lastChunkIdx = 255;
        lastRxMs = 0;
        Adpcm::initState(adpcmState);
        memset(slots, 0, sizeof(slots));
        writeIdx = 0;
        readIdx = 0;
        count = 0;
        prebuffering = true;
    }
};

class UsbHeadset {
public:
    static UsbHeadset& getInstance();

    // Initialize USB Host UAC driver on native USB OTG pins (GPIO 19 = D-, GPIO 20 = D+)
    bool begin();

    // Status queries
    bool isConnected()   const { return _deviceConnected; }
    bool isTransmitting() const { return _isTransmitting; }
    bool isPlaying()     const { return _isPlaying; }
    bool isVoiceActive() const;

    // HT Voice Join / Leave (Full Duplex Voice Room)
    bool joinVoice();
    void leaveVoice();
    bool toggleVoice(); // Press headset button to join/leave

    // Local audible tone generator for headset earpiece feedback (join, leave, volume)
    void playTone(uint16_t freq = 1000, uint16_t durationMs = 100);

    // Volume Control (0 - 100%)
    void    setVolume(uint8_t percent);
    void    volumeUp(uint8_t step = 10);
    void    volumeDown(uint8_t step = 10);
    uint8_t getVolume() const { return _volumePercent; }

    // Hardware Audio Properties
    uint32_t getSpkSampleRate() const { return _spkNativeSampleRate; }
    uint8_t  getSpkBitRes()     const { return _spkNativeBitRes; }
    uint8_t  getSpkChannels()   const { return _spkNativeChannels; }
    uint32_t getMicSampleRate() const { return _micNativeSampleRate; }
    uint8_t  getMicBitRes()     const { return _micNativeBitRes; }
    uint8_t  getMicChannels()   const { return _micNativeChannels; }

    // Handle incoming live audio chunk from ESP-NOW Mesh into jitter queue
    void receiveAudioChunk(const uint8_t* senderMac, uint8_t msgId, uint8_t chunkIdx, uint8_t totalChunks,
                           uint8_t mimeType, const uint8_t* data, uint16_t len);

    // Notification when remote node leaves voice stream
    void onRemotePttStop(const uint8_t* senderMac = nullptr);

    // Periodic maintenance in loop()
    void loop();

    // Internal callbacks from USB driver tasks / ISRs
    void onMicData(void* data, size_t dataBytes, uint16_t bitRes, uint32_t sampleFreq);
    void onDeviceState(bool connected);

    // Background playback task processor
    bool processPlayback();

private:
    UsbHeadset();
    ~UsbHeadset();

    bool          _initialized;
    volatile bool _deviceConnected;
    volatile bool _isTransmitting;
    volatile bool _isPlaying;
    volatile bool _pendingWelcomeChime;

    uint32_t _txStartTime;
    uint32_t _txBytesCount;

    // Headset DAC / Mic properties
    uint32_t _spkNativeSampleRate;
    uint8_t  _spkNativeChannels;
    uint8_t  _spkNativeBitRes;
    uint32_t _micNativeSampleRate;
    uint8_t  _micNativeChannels;
    uint8_t  _micNativeBitRes;

    // Volume level
    uint8_t  _volumePercent;

    void writeSpkPcm(const int16_t* monoSamples, size_t numSamples);

    // Audio DSP: 80 Hz Mild DC Blocker
    DcBlocker _dcBlocker;

    // Real-Time Live Streaming TX Buffer & State
    int16_t    _txChunkPcm[STREAM_CHUNK_SAMPLES];
    uint16_t   _txChunkCount;
    uint8_t    _streamMsgId;
    uint8_t    _streamChunkIdx;
    AdpcmState _txAdpcmState;

    // Multi-Stream Jitter Buffer (Full Duplex simultaneous reception & mixing)
    AudioRxChannel _rxChannels[MAX_AUDIO_STREAMS];
    portMUX_TYPE   _jitterMux;
    uint32_t       _lastRxChunkMs;

    // FreeRTOS Playback Task
    TaskHandle_t _playbackTaskHandle;
    static void  playbackTaskEntry(void* param);
};

#endif // USB_HEADSET_H
