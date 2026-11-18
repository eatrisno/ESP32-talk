#!/usr/bin/env python3
"""
Convert WAV files from assets/audio into C++ voice prompt data.
Usage: python3 scripts/generate_prompts.py
"""

import wave
import os
import sys
import numpy as np
import scipy.signal as signal

PROMPT_MAP = [
    ('PROMPT_ZERO',          'zero.wav'),
    ('PROMPT_ONE',           'one.wav'),
    ('PROMPT_TWO',           'two.wav'),
    ('PROMPT_THREE',         'three.wav'),
    ('PROMPT_FOUR',          'four.wav'),
    ('PROMPT_FIVE',          'five.wav'),
    ('PROMPT_DEVICE',        'device.wav'),
    ('PROMPT_DEVICES',       'devices.wav'),
    ('PROMPT_NEARBY',        'nearby.wav'),
    ('PROMPT_MIDDLE',        'middle.wav'),
    ('PROMPT_FAR',           'far.wav'),
    ('PROMPT_MIC_ON',        'microphone enabled.wav'),
    ('PROMPT_MIC_OFF',       'microphone muted.wav'),
    ('PROMPT_VM_RECORDING',  'voice note recording.wav'),
    ('PROMPT_VM_SENT',       'message sent.wav'),
    ('PROMPT_VM_OUTBOX',     'saved to outbox.wav'),
    ('PROMPT_VM_INCOMING',   'incoming voice note.wav'),
    ('PROMPT_EMERGENCY_ON',  'emergency activated.wav'),
    ('PROMPT_EMERGENCY_OFF', 'emergency canceled.wav'),
    ('PROMPT_EMERGENCY_RX',  'emergency alert.wav'),
    ('PROMPT_CONNECTED',     'connected.wav'),
    ('PROMPT_DISCONNECTED',  'disconnected.wav'),
    ('PROMPT_READY',         'ready.wav'),
]

TARGET_SR = 16000
TARGET_PEAK = 24000.0  # Safe headroom below 32767

def process_file(path):
    with wave.open(path, 'rb') as w:
        sr_in = w.getframerate()
        ch = w.getnchannels()
        sw = w.getsampwidth()
        frames = w.readframes(w.getnframes())
        raw = np.frombuffer(frames, dtype=np.int16)
        if ch > 1:
            raw = raw[::ch]  # take first channel if stereo

        # Trim leading and trailing silence
        threshold = 500
        idx = np.where(np.abs(raw) > threshold)[0]
        if len(idx) > 0:
            start = max(0, idx[0] - int(0.03 * sr_in))
            end = min(len(raw), idx[-1] + int(0.04 * sr_in))
            raw = raw[start:end]

        # Resample to 16,000 Hz
        if sr_in != TARGET_SR:
            num_out = int(len(raw) * TARGET_SR / sr_in)
            resampled = signal.resample(raw, num_out).astype(np.int16)
        else:
            resampled = raw

        # Soft 5ms Hann fade in/out to prevent pops
        fade_len = min(80, len(resampled) // 4)
        if fade_len > 0:
            fade_in = (0.5 * (1 - np.cos(np.linspace(0, np.pi, fade_len)))).astype(np.float32)
            fade_out = (0.5 * (1 + np.cos(np.linspace(0, np.pi, fade_len)))).astype(np.float32)
            resampled[:fade_len] = (resampled[:fade_len] * fade_in).astype(np.int16)
            resampled[-fade_len:] = (resampled[-fade_len:] * fade_out).astype(np.int16)

        # Normalize volume
        peak = np.max(np.abs(resampled))
        if peak > 0:
            resampled = (resampled * (TARGET_PEAK / peak)).astype(np.int16)

        return resampled

def main():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    assets_dir = os.path.join(repo_root, 'assets', 'audio')
    out_hpp = os.path.join(repo_root, 'src', 'voice_prompts.hpp')
    out_cpp = os.path.join(repo_root, 'src', 'voice_prompts.cpp')

    processed = {}
    total_bytes = 0

    print("Processing audio files from:", assets_dir)
    for p_enum, fname in PROMPT_MAP:
        fpath = os.path.join(assets_dir, fname)
        if not os.path.exists(fpath):
            print(f"Warning: Missing {fname}, skipping")
            continue
        samples = process_file(fpath)
        processed[p_enum] = (fname, samples)
        total_bytes += len(samples) * 2
        print(f"  {p_enum:22s} ({fname:24s}): {len(samples)/TARGET_SR:.2f}s ({len(samples)} samples, {len(samples)*2/1024:.1f} KB)")

    print(f"\nTotal Prompts Size: {total_bytes / 1024:.1f} KB ({total_bytes} bytes)\n")

    # Generate voice_prompts.hpp
    with open(out_hpp, 'w') as f:
        f.write('#pragma once\n\n')
        f.write('#include <cstddef>\n#include <cstdint>\n\n')
        f.write('namespace wt {\n\n')
        f.write('enum PromptId : uint8_t {\n')
        f.write('    PROMPT_NONE = 0,\n')
        for p_enum, _ in PROMPT_MAP:
            f.write(f'    {p_enum},\n')
        f.write('    PROMPT_COUNT\n')
        f.write('};\n\n')
        f.write('struct PromptData {\n')
        f.write('    const int16_t* samples;\n')
        f.write('    size_t count;\n')
        f.write('};\n\n')
        f.write('const PromptData& get_prompt(PromptId id);\n\n')
        f.write('class PromptQueue {\n')
        f.write('public:\n')
        f.write('    PromptQueue() = default;\n')
        f.write('    void push(PromptId id);\n')
        f.write('    void clear();\n')
        f.write('    bool busy() const;\n')
        f.write('    bool render(int16_t* out, size_t n, float vol_scale = 1.0f);\n')
        f.write('private:\n')
        f.write('    static constexpr int kMaxQueue = 8;\n')
        f.write('    PromptId queue_[kMaxQueue] = {};\n')
        f.write('    int head_ = 0, tail_ = 0;\n')
        f.write('    size_t sample_pos_ = 0;\n')
        f.write('};\n\n')
        f.write('}  // namespace wt\n')

    # Generate voice_prompts.cpp
    with open(out_cpp, 'w') as f:
        f.write('#include "voice_prompts.hpp"\n')
        f.write('#include "dsp.hpp"\n')
        f.write('#include <cstring>\n\n')
        f.write('namespace wt {\n\n')

        # Arrays
        for p_enum, (fname, samples) in processed.items():
            f.write(f'// {p_enum}: {fname} ({len(samples)} samples, {len(samples)/TARGET_SR:.2f}s)\n')
            f.write(f'static const int16_t s_{p_enum}[] = {{\n')
            for i in range(0, len(samples), 16):
                chunk = samples[i:i+16]
                line = ', '.join(f'{x}' for x in chunk)
                f.write(f'    {line},\n')
            f.write('};\n\n')

        # Table lookup
        f.write('static const PromptData s_prompt_table[PROMPT_COUNT] = {\n')
        f.write('    { nullptr, 0 }, // PROMPT_NONE\n')
        for p_enum, _ in PROMPT_MAP:
            if p_enum in processed:
                f.write(f'    {{ s_{p_enum}, sizeof(s_{p_enum}) / sizeof(int16_t) }}, // {p_enum}\n')
            else:
                f.write('    { nullptr, 0 },\n')
        f.write('};\n\n')

        f.write('const PromptData& get_prompt(PromptId id) {\n')
        f.write('    if (id < PROMPT_COUNT) return s_prompt_table[id];\n')
        f.write('    return s_prompt_table[0];\n')
        f.write('}\n\n')

        # PromptQueue implementation
        f.write('''void PromptQueue::push(PromptId id) {
    if (id == PROMPT_NONE || id >= PROMPT_COUNT) return;
    int next = (tail_ + 1) % kMaxQueue;
    if (next != head_) {
        queue_[tail_] = id;
        tail_ = next;
    }
}

void PromptQueue::clear() {
    head_ = tail_ = 0;
    sample_pos_ = 0;
}

bool PromptQueue::busy() const {
    return head_ != tail_;
}

bool PromptQueue::render(int16_t* out, size_t n, float vol_scale) {
    if (head_ == tail_) return false;
    size_t out_idx = 0;
    while (out_idx < n && head_ != tail_) {
        PromptId cur_id = queue_[head_];
        const PromptData& p = s_prompt_table[cur_id];
        if (!p.samples || sample_pos_ >= p.count) {
            head_ = (head_ + 1) % kMaxQueue;
            sample_pos_ = 0;
            continue;
        }
        size_t avail = p.count - sample_pos_;
        size_t take = (n - out_idx < avail) ? (n - out_idx) : avail;
        for (size_t i = 0; i < take; i++) {
            int32_t val = out[out_idx + i] + static_cast<int32_t>(p.samples[sample_pos_ + i] * vol_scale);
            out[out_idx + i] = clip16(val);
        }
        sample_pos_ += take;
        out_idx += take;
        if (sample_pos_ >= p.count) {
            head_ = (head_ + 1) % kMaxQueue;
            sample_pos_ = 0;
        }
    }
    return true;
}

}  // namespace wt
''')

    print("Generated src/voice_prompts.hpp and src/voice_prompts.cpp successfully!")

if __name__ == '__main__':
    main()
