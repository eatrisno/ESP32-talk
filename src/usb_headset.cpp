#include "usb_headset.h"
#include "USB_STREAM.h"
#include "messages.h"
#include "esp_now_comm.h"
#include "config.h"

static const char* TAG = "USB_HEADSET";
static USB_STREAM* s_usbStream = nullptr;

// Static Callbacks invoked by USB Host driver
static void onUsbMicFrame(mic_frame_t *frame, void *ptr) {
    if (!frame || !frame->data || frame->data_bytes == 0) return;
    UsbHeadset::getInstance().onMicData(frame->data, frame->data_bytes, frame->bit_resolution, frame->samples_frequence);
}

static void onUsbStreamStateChange(usb_stream_state_t event, void *arg) {
    bool connected = (event == STREAM_CONNECTED);
    UsbHeadset::getInstance().onDeviceState(connected);
}

// Background FreeRTOS Playback Task (runs on Core 1 for zero-stutter playback)
void UsbHeadset::playbackTaskEntry(void* param) {
    UsbHeadset* self = (UsbHeadset*)param;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // Deferred welcome chime: play when stream is active without blocking state callback
        if (self->_pendingWelcomeChime && self->_deviceConnected && s_usbStream) {
            self->_pendingWelcomeChime = false;
            vTaskDelay(pdMS_TO_TICKS(150));
            self->playTone(880, 100);
            vTaskDelay(pdMS_TO_TICKS(50));
            self->playTone(1320, 120);
            xLastWakeTime = xTaskGetTickCount();
        }

        // PLL Watermark Flow-Control: prevent ringbuffer overflow/underflow
        size_t ringLen = 0;
        if (s_usbStream && self->_deviceConnected) {
            ringLen = s_usbStream->uacSpkGetRingbufLen();
        }

        if (ringLen > 48000) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        bool played = self->processPlayback();

        if (played) {
            // Fixed 50ms pacing: processPlayback() emits exactly 400 samples @ 8kHz = 50ms.
            // After 6:1 upsample → 2400 samples @ 48kHz = exactly 50ms of USB isochronous frames.
            // Adaptive pacing caused underrun (48ms → only 2304 samples → 96-sample gap = clicks).
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
            xLastWakeTime = xTaskGetTickCount();
        }
    }
}

// Singleton
UsbHeadset& UsbHeadset::getInstance() {
    static UsbHeadset instance;
    return instance;
}

bool UsbHeadset::isVoiceActive() const {
    return _isTransmitting || _isPlaying || (_rxChannels[0].count > 0) || (_rxChannels[1].count > 0);
}

UsbHeadset::UsbHeadset()
    : _initialized(false)
    , _deviceConnected(false)
    , _isTransmitting(false)
    , _isPlaying(false)
    , _pendingWelcomeChime(false)
    , _txStartTime(0)
    , _txBytesCount(0)
    , _spkNativeSampleRate(48000)
    , _spkNativeChannels(2)
    , _spkNativeBitRes(16)
    , _micNativeSampleRate(48000)
    , _micNativeChannels(1)
    , _micNativeBitRes(16)
    , _volumePercent(80)
    , _txChunkCount(0)
    , _streamMsgId(0)
    , _streamChunkIdx(0)
    , _lastRxChunkMs(0)
    , _playbackTaskHandle(nullptr)
{
    _jitterMux = portMUX_INITIALIZER_UNLOCKED;
    _dcBlocker.reset();
    Adpcm::initState(_txAdpcmState);

    memset(_txChunkPcm, 0, sizeof(_txChunkPcm));
    for (int i = 0; i < MAX_AUDIO_STREAMS; i++) {
        _rxChannels[i].reset();
    }
}

UsbHeadset::~UsbHeadset() {
    if (_playbackTaskHandle) {
        vTaskDelete(_playbackTaskHandle);
        _playbackTaskHandle = nullptr;
    }
}

bool UsbHeadset::begin() {
    if (_initialized) return true;

    Serial.println("[USB-HEADSET] Initializing USB Host UAC Driver for Type-C Headset...");

    s_usbStream = new USB_STREAM();
    if (!s_usbStream) {
        Serial.println("[USB-HEADSET] ERROR: Failed to instantiate USB_STREAM!");
        return false;
    }

    // Configure UAC: Target 48000 Hz for perfect 1ms isochronous USB packet alignment (zero clicks/pops)
    s_usbStream->uacConfiguration(
        UAC_CH_ANY, UAC_BITS_ANY, 48000, 16384,
        2, UAC_BITS_ANY, 48000, 65536
    );

    // Register Callbacks
    s_usbStream->uacMicRegisterCb(onUsbMicFrame, s_usbStream);
    s_usbStream->_user_frame_cb_arg = s_usbStream;
    s_usbStream->registerState(onUsbStreamStateChange, this);

    // Start USB Host Engine
    s_usbStream->start();

    // Launch Background Jitter Buffer Audio Playback Task (pinned to Core 1)
    xTaskCreatePinnedToCore(
        playbackTaskEntry,
        "uac_playback",
        8192,
        this,
        5,
        &_playbackTaskHandle,
        1
    );

    _initialized = true;
    Serial.println("[USB-HEADSET] USB Host UAC Driver Active (GPIO 19=D-, GPIO 20=D+)");
    Serial.println("[USB-HEADSET] Full Duplex Streaming Engine & Digital Mixer Started (Core 1)");
    return true;
}

void UsbHeadset::onDeviceState(bool connected) {
    _deviceConnected = connected;
    if (connected) {
        Serial.println("\n[USB-HEADSET] ==================================");
        Serial.println("[USB-HEADSET] Type-C Headset CONNECTED & READY!");
        Serial.println("[USB-HEADSET] Full Duplex Voice Streaming Active");
        Serial.println("[USB-HEADSET] ==================================\n");
        setStatusLed(LED_COLOR_GREEN, 1000);

        if (s_usbStream) {
            _spkNativeSampleRate = s_usbStream->uacSpkGetSampleRate();
            _spkNativeChannels   = s_usbStream->uacSpkGetChannels();
            _spkNativeBitRes     = s_usbStream->uacSpkGetBitResolution();
            _micNativeChannels   = s_usbStream->uacMicGetChannels();
            if (_spkNativeSampleRate == 0) _spkNativeSampleRate = 48000;
            if (_spkNativeChannels == 0)   _spkNativeChannels = 2;
            if (_spkNativeBitRes == 0)     _spkNativeBitRes = 16;
            if (_micNativeChannels == 0)   _micNativeChannels = 1;

            Serial.printf("[USB-HEADSET] Negotiated DAC: %u Hz, %u-bit, %u ch | Mic: %u ch\n",
                          (unsigned int)_spkNativeSampleRate, _spkNativeBitRes, _spkNativeChannels, _micNativeChannels);

            // Signal welcome chime to background playback task (DO NOT block the USB driver task!)
            _pendingWelcomeChime = true;
        }
    } else {
        Serial.println("\n[USB-HEADSET] Type-C Headset DISCONNECTED.");
        _isTransmitting = false;
        _isPlaying = false;
        _pendingWelcomeChime = false;
    }
}

bool UsbHeadset::joinVoice() {
    if (_isTransmitting) return false;

    _isTransmitting = true;
    _txStartTime = millis();
    _txBytesCount = 0;
    _txChunkCount = 0;
    _streamChunkIdx = 0;
    _streamMsgId++;
    if (_streamMsgId == 0) _streamMsgId = 1;

    _dcBlocker.reset();
    Adpcm::initState(_txAdpcmState);

    // Notify mesh peers that local node has joined voice room
    EspNowComm::getInstance().sendPttStart();

    // SENDER: Solid RED LED indicates active live voice transmission
    setStatusLed(LED_COLOR_RED);

    // Play pleasant ascending chime into local headset: 800 Hz -> 1200 Hz
    playTone(800, 40);
    delay(40);
    playTone(1200, 60);

    Serial.printf("[HT-VOICE] 🎙️ JOINED Voice Channel [id=%u] (Full Duplex Transmit & Receive Active)\n", _streamMsgId);
    return true;
}

void UsbHeadset::leaveVoice() {
    if (!_isTransmitting) return;
    _isTransmitting = false;
    uint32_t durationMs = millis() - _txStartTime;

    EspNowComm& comm = EspNowComm::getInstance();

    // Flush any pending samples in accumulation buffer as final chunk
    if (_txChunkCount > 0) {
        while (_txChunkCount < STREAM_CHUNK_SAMPLES) {
            _txChunkPcm[_txChunkCount++] = 0;
        }
        uint8_t adpcmOut[STREAM_ADPCM_BYTES];
        size_t encBytes = Adpcm::encode(_txChunkPcm, STREAM_CHUNK_SAMPLES, adpcmOut, _txAdpcmState);
        comm.sendAudioChunk(_streamMsgId, _streamChunkIdx, _streamChunkIdx + 1,
                            AUDIO_MIME_ADPCM, adpcmOut, encBytes);
        _streamChunkIdx++;
        _txChunkCount = 0;
    }

    // Send PTT_STOP over ESP-NOW to notify peers
    comm.sendPttStop(_txBytesCount);

    // SENDER: Electric Blue pulse confirms RF transmission complete
    setStatusLed(LED_COLOR_BLUE, 300);

    // Play pleasant descending chime into local headset: 1100 Hz -> 600 Hz
    playTone(1100, 40);
    delay(40);
    playTone(600, 60);

    Serial.printf("[HT-VOICE] ⏹️ LEFT Voice Channel: %u chunks (%u bytes) streamed in %u ms -> Standby\n",
                  _streamChunkIdx, (unsigned int)_txBytesCount, (unsigned int)durationMs);
}

bool UsbHeadset::toggleVoice() {
    if (_isTransmitting) {
        leaveVoice();
        return false;
    } else {
        joinVoice();
        return true;
    }
}

void UsbHeadset::onMicData(void* data, size_t dataBytes, uint16_t bitRes, uint32_t sampleFreq) {
    if (!data || dataBytes == 0) return;

    _micNativeSampleRate = sampleFreq ? sampleFreq : 48000;
    _micNativeBitRes = bitRes ? bitRes : 16;
    if (_micNativeChannels == 0 && s_usbStream) {
        _micNativeChannels = s_usbStream->uacMicGetChannels();
    }
    uint8_t channels = _micNativeChannels ? _micNativeChannels : 1;

    size_t bytesPerSample = (_micNativeBitRes > 16) ? ((_micNativeBitRes == 24) ? 3 : 4) : 2;
    if (bytesPerSample * channels == 0) return;
    size_t numFrames = dataBytes / (channels * bytesPerSample);
    if (numFrames == 0) return;

    auto getSample = [&](size_t frame, uint8_t ch) -> int16_t {
        size_t offset = (frame * channels + ch) * bytesPerSample;
        if (offset + bytesPerSample > dataBytes) return 0;
        const uint8_t* p = ((const uint8_t*)data) + offset;
        if (bytesPerSample == 2) {
            return (int16_t)(p[0] | (p[1] << 8));
        } else if (bytesPerSample == 3) {
            // 24-bit PCM: bits [23:8] give 16-bit signed sample
            return (int16_t)(p[1] | (p[2] << 8));
        } else {
            // 32-bit PCM: bits [31:16] give 16-bit signed sample
            return (int16_t)(p[2] | (p[3] << 8));
        }
    };

    auto getMonoFrame = [&](size_t frame) -> int16_t {
        if (channels > 1) {
            int16_t ch0 = getSample(frame, 0);
            int16_t ch1 = getSample(frame, 1);
            // Smart channel mapping: if one channel is silent/disconnected while the other has signal,
            // use the active channel directly so microphone volume is not cut in half
            if (abs(ch1) < 50 && abs(ch0) > 100) return ch0;
            if (abs(ch0) < 50 && abs(ch1) > 100) return ch1;
            return (int16_t)(((int32_t)ch0 + (int32_t)ch1) / 2);
        } else {
            return getSample(frame, 0);
        }
    };

    // Calculate peak amplitude in this frame
    int16_t framePeak = 0;
    for (size_t i = 0; i < numFrames; i++) {
        int16_t v = abs(getMonoFrame(i));
        if (v > framePeak) framePeak = v;
    }

    // Mic activity blip when idle
    if (!_isTransmitting) {
        if (framePeak > 1500) {
            static uint32_t s_lastBlinkMs = 0;
            if (millis() - s_lastBlinkMs > 120) {
                s_lastBlinkMs = millis();
                setStatusLed(LED_COLOR_CYAN, 60);
            }
        }
        return;
    }

    // Fast 6:1 Decimator (48000 Hz -> 8000 Hz) with fractional fallback
    if (_micNativeSampleRate == 48000) {
        static int32_t s_accum = 0;
        static uint8_t s_accumCount = 0;

        for (size_t f = 0; f < numFrames; f++) {
            s_accum += getMonoFrame(f);
            s_accumCount++;

            if (s_accumCount >= 6) {
                int16_t rawSample = (int16_t)(s_accum / 6);
                s_accum = 0;
                s_accumCount = 0;

                // Mild 80 Hz DC Blocker
                int16_t cleanSample = _dcBlocker.process(rawSample);

                // Conversational speech pre-gain (2x with soft-knee limiter — avoids pre-ADPCM clipping)
                cleanSample = applyDigitalGainAndLimit(cleanSample, 2);

                // Accumulate into 50ms chunk buffer (400 samples)
                _txChunkPcm[_txChunkCount++] = cleanSample;
                _txBytesCount += sizeof(int16_t);

                if (_txChunkCount >= STREAM_CHUNK_SAMPLES) {
                    uint8_t adpcmOut[STREAM_ADPCM_BYTES];
                    size_t encBytes = Adpcm::encode(_txChunkPcm, STREAM_CHUNK_SAMPLES, adpcmOut, _txAdpcmState);

                    EspNowComm::getInstance().sendAudioChunk(
                        _streamMsgId, _streamChunkIdx++, 0xFF,
                        AUDIO_MIME_ADPCM, adpcmOut, encBytes
                    );

                    _txChunkCount = 0;
                }
            }
        }
    } else {
        // Fractional fallback for arbitrary microphone sample rates
        uint32_t step = ((uint64_t)_micNativeSampleRate << 16) / HEADSET_TARGET_SAMPLE_RATE;
        if (step == 0) step = 1 << 16;
        static uint32_t s_phase = 0;

        while ((s_phase >> 16) < numFrames) {
            size_t startIdx = s_phase >> 16;
            size_t endIdx = min(numFrames, (size_t)((s_phase + step) >> 16));
            if (endIdx <= startIdx) endIdx = startIdx + 1;

            int32_t sum = 0;
            size_t count = 0;
            for (size_t idx = startIdx; idx < endIdx; idx++) {
                sum += getMonoFrame(idx);
                count++;
            }
            int16_t rawSample = (count > 0) ? (int16_t)(sum / (int32_t)count) : 0;
            int16_t cleanSample = _dcBlocker.process(rawSample);
            cleanSample = applyDigitalGainAndLimit(cleanSample, 2);

            _txChunkPcm[_txChunkCount++] = cleanSample;
            _txBytesCount += sizeof(int16_t);

            if (_txChunkCount >= STREAM_CHUNK_SAMPLES) {
                uint8_t adpcmOut[STREAM_ADPCM_BYTES];
                size_t encBytes = Adpcm::encode(_txChunkPcm, STREAM_CHUNK_SAMPLES, adpcmOut, _txAdpcmState);
                EspNowComm::getInstance().sendAudioChunk(
                    _streamMsgId, _streamChunkIdx++, 0xFF,
                    AUDIO_MIME_ADPCM, adpcmOut, encBytes
                );
                _txChunkCount = 0;
            }
            s_phase += step;
        }
        if (numFrames > 0 && s_phase >= (numFrames << 16)) {
            s_phase -= (numFrames << 16);
        }
    }
}

void UsbHeadset::receiveAudioChunk(const uint8_t* senderMac, uint8_t msgId, uint8_t chunkIdx, uint8_t totalChunks,
                                   uint8_t mimeType, const uint8_t* data, uint16_t len) {
    if (!data || len == 0) return;

    uint32_t now = millis();
    _lastRxChunkMs = now;

    uint8_t dummyMac[6] = {0};
    const uint8_t* mac = senderMac ? senderMac : dummyMac;

    // 1. Locate existing channel matching this sender MAC
    int chanIdx = -1;
    for (int i = 0; i < MAX_AUDIO_STREAMS; i++) {
        if (_rxChannels[i].active && memcmp(_rxChannels[i].mac, mac, 6) == 0) {
            chanIdx = i;
            break;
        }
    }

    // 2. If not found, assign to first inactive channel
    if (chanIdx < 0) {
        for (int i = 0; i < MAX_AUDIO_STREAMS; i++) {
            if (!_rxChannels[i].active || (now - _rxChannels[i].lastRxMs > 1500)) {
                chanIdx = i;
                _rxChannels[i].reset();
                _rxChannels[i].active = true;
                memcpy(_rxChannels[i].mac, mac, 6);
                break;
            }
        }
    }

    if (chanIdx < 0) chanIdx = 0;

    AudioRxChannel& ch = _rxChannels[chanIdx];
    ch.lastRxMs = now;

    // New stream session check
    if (msgId != ch.msgId || chunkIdx == 0) {
        ch.msgId = msgId;
        ch.lastChunkIdx = 255;
        Adpcm::initState(ch.adpcmState);

        portENTER_CRITICAL(&_jitterMux);
        ch.writeIdx = 0;
        ch.readIdx = 0;
        ch.count = 0;
        ch.prebuffering = true;
        portEXIT_CRITICAL(&_jitterMux);
    } else if (chunkIdx == ch.lastChunkIdx && ch.lastChunkIdx != 255) {
        // Drop duplicate chunk
        return;
    }
    ch.lastChunkIdx = chunkIdx;

    // Overrun protection
    if (ch.count >= JITTER_BUFFER_SLOTS) {
        portENTER_CRITICAL(&_jitterMux);
        ch.readIdx = (ch.readIdx + 1) % JITTER_BUFFER_SLOTS;
        ch.count--;
        portEXIT_CRITICAL(&_jitterMux);
    }

    JitterSlot& slot = ch.slots[ch.writeIdx];

    if (mimeType == AUDIO_MIME_ADPCM) {
        size_t decoded = Adpcm::decode(data, len, slot.samples, ch.adpcmState);
        slot.sampleCount = decoded;
    } else {
        size_t samples = len / sizeof(int16_t);
        if (samples > STREAM_CHUNK_SAMPLES) samples = STREAM_CHUNK_SAMPLES;
        memcpy(slot.samples, data, samples * sizeof(int16_t));
        slot.sampleCount = samples;
    }

    ch.writeIdx = (ch.writeIdx + 1) % JITTER_BUFFER_SLOTS;
    portENTER_CRITICAL(&_jitterMux);
    ch.count++;
    portEXIT_CRITICAL(&_jitterMux);

    // Prebuffer 2 chunks (100ms) for rock-solid playback
    if (ch.count >= JITTER_PREBUFFER_CHUNKS) {
        ch.prebuffering = false;
    }
}

void UsbHeadset::onRemotePttStop(const uint8_t* senderMac) {
    if (senderMac) {
        for (int i = 0; i < MAX_AUDIO_STREAMS; i++) {
            if (_rxChannels[i].active && memcmp(_rxChannels[i].mac, senderMac, 6) == 0) {
                _rxChannels[i].prebuffering = false;
                return;
            }
        }
    }
    for (int i = 0; i < MAX_AUDIO_STREAMS; i++) {
        if (_rxChannels[i].active) {
            _rxChannels[i].prebuffering = false;
        }
    }
}

bool UsbHeadset::processPlayback() {
    uint32_t now = millis();

    // Deactivate channels that have expired (>800ms silence)
    for (int i = 0; i < MAX_AUDIO_STREAMS; i++) {
        if (_rxChannels[i].active && _rxChannels[i].count == 0 && (now - _rxChannels[i].lastRxMs > 800)) {
            _rxChannels[i].active = false;
        }
    }

    bool chReady[MAX_AUDIO_STREAMS] = { false, false };
    static JitterSlot s_poppedSlot[MAX_AUDIO_STREAMS];

    for (int i = 0; i < MAX_AUDIO_STREAMS; i++) {
        AudioRxChannel& ch = _rxChannels[i];
        if (!ch.active || ch.count == 0) continue;

        if (ch.prebuffering) {
            if (ch.count < JITTER_PREBUFFER_CHUNKS && (now - ch.lastRxMs < 120)) {
                continue;
            }
            ch.prebuffering = false;
        }

        portENTER_CRITICAL(&_jitterMux);
        s_poppedSlot[i] = ch.slots[ch.readIdx];
        ch.slots[ch.readIdx].sampleCount = 0;
        ch.readIdx = (ch.readIdx + 1) % JITTER_BUFFER_SLOTS;
        if (ch.count > 0) ch.count--;
        portEXIT_CRITICAL(&_jitterMux);

        if (s_poppedSlot[i].sampleCount > 0) {
            chReady[i] = true;
        }
    }

    if (!chReady[0] && !chReady[1]) {
        if (_isPlaying && (now - _lastRxChunkMs > 300)) {
            _isPlaying = false;
            if (!_isTransmitting) {
                setStatusLed(LED_COLOR_OFF);
            }
        }
        return false;
    }

    _isPlaying = true;

    static int16_t s_outSamples[STREAM_CHUNK_SAMPLES];
    uint16_t outCount = 0;

    if (chReady[0] && chReady[1]) {
        // Multi-Party Voice Mixing in Full Duplex!
        outCount = max(s_poppedSlot[0].sampleCount, s_poppedSlot[1].sampleCount);
        if (outCount > STREAM_CHUNK_SAMPLES) outCount = STREAM_CHUNK_SAMPLES;

        for (uint16_t s = 0; s < outCount; s++) {
            int32_t sample0 = (s < s_poppedSlot[0].sampleCount) ? s_poppedSlot[0].samples[s] : 0;
            int32_t sample1 = (s < s_poppedSlot[1].sampleCount) ? s_poppedSlot[1].samples[s] : 0;
            int32_t sum = sample0 + sample1;
            if (sum > 32767) sum = 32767;
            else if (sum < -32768) sum = -32768;
            s_outSamples[s] = (int16_t)sum;
        }
    } else if (chReady[0]) {
        outCount = s_poppedSlot[0].sampleCount;
        memcpy(s_outSamples, s_poppedSlot[0].samples, outCount * sizeof(int16_t));
    } else {
        outCount = s_poppedSlot[1].sampleCount;
        memcpy(s_outSamples, s_poppedSlot[1].samples, outCount * sizeof(int16_t));
    }

    if (outCount > 0) {
        // NOTE: No RX gain applied here. The TX side already applies 3x mic pre-gain.
        // Adding gain here causes 6x total (3x TX × 2x RX) = severe clipping / "sharp" distortion.
        if (_deviceConnected && s_usbStream) {
            // Upsample 8000 Hz Mono -> Native Headset DAC (typically 48000 Hz Stereo)
            static int16_t s_uacUpsampleBuf[STREAM_CHUNK_SAMPLES * 6];
            size_t targetRate = _spkNativeSampleRate ? _spkNativeSampleRate : 48000;

            size_t targetFrames = (targetRate * outCount) / HEADSET_TARGET_SAMPLE_RATE;
            if (targetFrames > STREAM_CHUNK_SAMPLES * 6) targetFrames = STREAM_CHUNK_SAMPLES * 6;

            static int16_t s_prevLastSample = 0;

            if (targetRate == 48000) {
                // Exact 1:6 Linear Interpolator (8000 Hz -> 48000 Hz)
                // Smooth interpolation between previous chunk's last sample and current samples (zero chunk-boundary clicks)
                for (size_t i = 0; i < outCount; i++) {
                    int32_t startVal = (i == 0) ? (int32_t)s_prevLastSample : (int32_t)s_outSamples[i - 1];
                    int32_t endVal   = (int32_t)s_outSamples[i];
                    for (int k = 0; k < 6; k++) {
                        int32_t interp = startVal + (((endVal - startVal) * (k + 1)) / 6);
                        s_uacUpsampleBuf[i * 6 + k] = (int16_t)constrain(interp, -32768, 32767);
                    }
                }
                s_prevLastSample = s_outSamples[outCount - 1];
            } else {
                // Fractional fallback for arbitrary DAC sample rates
                uint32_t step = ((uint64_t)HEADSET_TARGET_SAMPLE_RATE << 16) / targetRate;
                uint32_t phase = 0;

                for (size_t f = 0; f < targetFrames; f++) {
                    size_t idx0 = phase >> 16;
                    size_t idx1 = (idx0 + 1 < outCount) ? idx0 + 1 : idx0;
                    uint32_t frac = phase & 0xFFFF;
                    int32_t interp = (int32_t)s_outSamples[idx0] + ((((int32_t)s_outSamples[idx1] - (int32_t)s_outSamples[idx0]) * (int32_t)frac) >> 16);
                    s_uacUpsampleBuf[f] = (int16_t)constrain(interp, -32768, 32767);
                    phase += step;
                }
            }

            writeSpkPcm(s_uacUpsampleBuf, targetFrames);
        }
        return true;
    }
    return false;
}

void UsbHeadset::writeSpkPcm(const int16_t* monoSamples, size_t numSamples) {
    if (!s_usbStream || !_deviceConnected || numSamples == 0) return;

    uint8_t channels = _spkNativeChannels ? _spkNativeChannels : 2;
    uint8_t bitRes   = _spkNativeBitRes ? _spkNativeBitRes : 16;

    // Buffer chunk: 120 frames (120 * 2 ch * 3 bytes = 720 bytes max)
    // 120 frames at 48000 Hz = exactly 2.5 ms (aligned with 1ms USB isochronous packets)
    uint8_t rawBuf[120 * 2 * 3];
    size_t sampleIdx = 0;

    while (sampleIdx < numSamples) {
        size_t chunk = min((size_t)120, numSamples - sampleIdx);
        size_t byteIdx = 0;

        for (size_t i = 0; i < chunk; i++) {
            // Apply volume scaling (0 - 100%)
            int32_t s = ((int32_t)monoSamples[sampleIdx + i] * _volumePercent) / 100;
            if (s > 32767) s = 32767;
            else if (s < -32768) s = -32768;
            int16_t sample16 = (int16_t)s;

            if (bitRes == 24) {
                // 24-bit PCM right-justified, little-endian: [LSB, MSB, sign_extension]
                // sample16 occupies bits[15:0]; bit[23:16] = sign extension (0x00 or 0xFF)
                // This is the standard format for bBitResolution=24, bSubframeSize=3 USB headsets.
                uint8_t signExt = (sample16 < 0) ? 0xFF : 0x00;
                // Left Ear:
                rawBuf[byteIdx++] = (uint8_t)(sample16 & 0xFF);         // bits[7:0]
                rawBuf[byteIdx++] = (uint8_t)((sample16 >> 8) & 0xFF);  // bits[15:8]
                rawBuf[byteIdx++] = signExt;                             // bits[23:16] sign extension

                if (channels > 1) {
                    // Right Ear — identical sample for full stereo in BOTH earphones
                    rawBuf[byteIdx++] = (uint8_t)(sample16 & 0xFF);
                    rawBuf[byteIdx++] = (uint8_t)((sample16 >> 8) & 0xFF);
                    rawBuf[byteIdx++] = signExt;
                }
            } else {
                // 16-bit PCM: 2 bytes per channel (little endian)
                // Left Ear:
                rawBuf[byteIdx++] = (uint8_t)(sample16 & 0xFF);
                rawBuf[byteIdx++] = (uint8_t)((sample16 >> 8) & 0xFF);

                if (channels > 1) {
                    // Right Ear - Duplicate identical sample for full sound in BOTH earphones!
                    rawBuf[byteIdx++] = (uint8_t)(sample16 & 0xFF);
                    rawBuf[byteIdx++] = (uint8_t)((sample16 >> 8) & 0xFF);
                }
            }
        }

        s_usbStream->uacWriteSpk((uint16_t*)rawBuf, byteIdx, 50);
        sampleIdx += chunk;
    }
}

void UsbHeadset::playTone(uint16_t freq, uint16_t durationMs) {
    if (!s_usbStream || !_deviceConnected || durationMs == 0) return;

    size_t sampleRate = _spkNativeSampleRate ? _spkNativeSampleRate : 48000;
    size_t totalSamples = (sampleRate * durationMs) / 1000;
    size_t rampSamples = (sampleRate * 10) / 1000;

    int16_t toneBuf[64];
    size_t sent = 0;

    while (sent < totalSamples) {
        size_t chunk = min((size_t)64, totalSamples - sent);
        for (size_t i = 0; i < chunk; i++) {
            size_t cur = sent + i;
            float t = (float)cur / (float)sampleRate;
            float amp = 16000.0f;
            if (cur < rampSamples) {
                amp *= (float)cur / (float)rampSamples;
            } else if (cur > totalSamples - rampSamples) {
                amp *= (float)(totalSamples - cur) / (float)rampSamples;
            }
            toneBuf[i] = (int16_t)(sinf(2.0f * 3.14159265f * (float)freq * t) * amp);
        }
        writeSpkPcm(toneBuf, chunk);
        sent += chunk;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void UsbHeadset::setVolume(uint8_t percent) {
    _volumePercent = constrain(percent, (uint8_t)0, (uint8_t)100);
    Serial.printf("[VOLUME] 🔊 Headset Volume: %u%%\n", _volumePercent);
}

void UsbHeadset::volumeUp(uint8_t step) {
    uint8_t old = _volumePercent;
    uint16_t v = (uint16_t)_volumePercent + step;
    if (v > 100) v = 100;
    setVolume((uint8_t)v);

    if (_deviceConnected && !_isPlaying) {
        if (v == 100 && old == 100) {
            playTone(2200, 40); // Limit alert tone
        } else {
            playTone(1500, 35); // Volume up feedback beep
        }
    }
}

void UsbHeadset::volumeDown(uint8_t step) {
    uint8_t old = _volumePercent;
    int16_t v = (int16_t)_volumePercent - step;
    if (v < 0) v = 0;
    setVolume((uint8_t)v);

    if (_deviceConnected && !_isPlaying) {
        if (v == 0 && old == 0) {
            playTone(400, 40); // Limit alert tone
        } else {
            playTone(750, 35); // Volume down feedback beep
        }
    }
}

void UsbHeadset::loop() {
    // Safety release: auto-stop voice transmission if continuously held for > 30 seconds
    if (_isTransmitting) {
        if (millis() - _txStartTime > 30000) {
            Serial.println("[HT-VOICE] ⚠️ Max continuous transmission reached (30s). Auto-releasing.");
            leaveVoice();
        }
    }
}
