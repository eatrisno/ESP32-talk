#!/usr/bin/env python3
"""
Automated Voice Cue Generator using Microsoft Neural TTS (edge-tts) + macOS afconvert.
Generates all 26 walkie-talkie voice cues with consistent voice and speed,
then automatically invokes scripts/generate_prompts.py.
"""

import asyncio
import os
import subprocess
import sys

try:
    import edge_tts
except ImportError:
    print("Error: edge-tts not installed. Run: python3 -m pip install edge-tts")
    sys.exit(1)

VOICE = "en-US-AriaNeural"
RATE = "+40%"  # Crisp, articulate tactical radio speed (saves Flash while sounding completely natural)

CUES = [
    # System & Boot
    ('ready.wav',                    'System ready'),
    ('power off.wav',                'Power off'),
    ('connected.wav',                'Connected'),
    ('disconnected.wav',             'Disconnected'),

    # Microphone
    ('microphone enabled.wav',       'Microphone enabled'),
    ('microphone disabled.wav',      'Microphone disabled'),

    # Menu items (Audio Voice Menu)
    ('menu.wav',                     'Menu'),
    ('voice note.wav',               'Voice note'),
    ('radar.wav',                    'Radar'),
    ('walk mode.wav',                'Walk mode'),
    ('push to talk.wav',             'Push to talk'),
    ('hands free.wav',               'Hands free'),
    ('channel.wav',                  'Channel'),

    # Feature Toggles
    ('walk mode enabled.wav',        'Walk mode enabled'),
    ('walk mode disabled.wav',       'Walk mode disabled'),
    ('ptt enabled.wav',              'Push to talk enabled'),
    ('hands free enabled.wav',       'Hands free enabled'),

    # Voicemail / Voice note
    ('voice note recording.wav',     'Voice note recording'),
    ('message sent.wav',             'Message sent'),
    ('saved to outbox.wav',          'Saved to outbox'),
    ('incoming voice note.wav',      'Incoming voice note'),

    # Emergency SOS
    ('emergency activated.wav',      'Emergency activated'),
    ('emergency alert.wav',          'Emergency alert'),
    ('emergency canceled.wav',       'Emergency canceled'),

    # Unified Radar Phrases (No disjointed stitching)
    ('no devices found.wav',         'No devices found'),
    ('one device nearby.wav',        'One device nearby'),
    ('one device in range.wav',      'One device in range'),
    ('one device far.wav',           'One device far'),
    ('two devices nearby.wav',       'Two devices nearby'),
    ('two devices in range.wav',     'Two devices in range'),
    ('two devices far.wav',          'Two devices far'),
    ('multiple devices nearby.wav',  'Multiple devices nearby'),
    ('multiple devices in range.wav','Multiple devices in range'),
    ('multiple devices far.wav',     'Multiple devices far'),

    # Numbers 1..13 (for channel selection & numeric feedback)
    ('one.wav',                      'One'),
    ('two.wav',                      'Two'),
    ('three.wav',                    'Three'),
    ('four.wav',                     'Four'),
    ('five.wav',                     'Five'),
    ('six.wav',                      'Six'),
    ('seven.wav',                    'Seven'),
    ('eight.wav',                    'Eight'),
    ('nine.wav',                     'Nine'),
    ('ten.wav',                      'Ten'),
    ('eleven.wav',                   'Eleven'),
    ('twelve.wav',                   'Twelve'),
    ('thirteen.wav',                 'Thirteen'),
]

async def generate_cue(filename, text, output_dir):
    temp_mp3 = f"/tmp/tts_{os.getpid()}_{filename}.mp3"
    target_wav = os.path.join(output_dir, filename)

    comm = edge_tts.Communicate(text, VOICE, rate=RATE)
    await comm.save(temp_mp3)

    # Convert MP3 to 16-bit 16 kHz Mono WAV using native macOS afconvert
    cmd = [
        "/usr/bin/afconvert",
        temp_mp3,
        "-o", target_wav,
        "-d", "LEI16@16000",
        "-c", "1",
        "-f", "WAVE"
    ]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if os.path.exists(temp_mp3):
        os.remove(temp_mp3)

async def main():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    assets_dir = os.path.join(repo_root, "assets", "audio")
    os.makedirs(assets_dir, exist_ok=True)

    print(f"Generating {len(CUES)} voice cues with voice '{VOICE}' at speed {RATE}...")
    for idx, (filename, text) in enumerate(CUES, 1):
        print(f"  [{idx:2d}/{len(CUES)}] Generating {filename:26s} (\"{text}\")...")
        await generate_cue(filename, text, assets_dir)

    print("\nAll audio cues successfully generated in assets/audio/!")

if __name__ == "__main__":
    asyncio.run(main())
