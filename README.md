# ESP32-S3 Mesh Walkie-Talkie

Hands-free group voice over ESP-NOW. Plug a USB-C headset into an ESP32-S3 and every node in range joins
one conversation automatically, with no pairing, no router and no internet. Nodes out of direct range are
reached through the others (mesh relay, up to 2 hops).

```
  Node A ◄──────────► Node B ◄──────────► Node C
   (headset)            (headset)           (headset)
        A and C cannot hear each other: B relays.
        All three hear each other, up to 4 talkers mixed at once.
```

## What it does

| | |
|---|---|
| Conference | Every node on the same channel and key hears every talker. Up to 4 voices are mixed at once. |
| Mesh | Each node rebroadcasts new packets once (ttl 3). A node skips its relay if a neighbour already sent it. |
| Headset | Any USB Audio Class 1.0 headset. It picks the best format the headset offers (48/44.1/32/16 kHz…, mono or stereo, 16/24/32-bit, any container size) and starts exactly that alternate setting. Asynchronous headsets (own DAC clock) are fed at the rate they report, so they do not drift or click. |
| Headset buttons | Play/Pause and Mute on the headset act the same as the BOOT button. A latching Hook / Phone-mute control toggles the mic on every change. Vol+ and Vol- change volume. |
| Voice | 8 kHz, IMA-ADPCM (32 kbit/s), 40 ms frames. Mic has a DC filter, automatic gain (adapts only on speech, so hiss between words is not pumped up) and a voice gate that learns the room noise, so silence is not sent. Talkers are summed in float and soft-limited once, so several loud voices compress instead of clipping. |
| Losses | Each frame carries its own decoder state, so a lost packet costs 40 ms and does not corrupt the frames after it. |
| Security | AES-128-CCM on every frame (encryption and authentication). Replay protection per sender, kept for 10 minutes after the sender goes quiet. |

## Hardware

* ESP32-S3 board with native USB and at least 4 MB flash (built for ESP32-S3-DevKitC-1; the image is
  4 MB so it also runs on 8 and 16 MB modules). Not yet run on hardware, see Testing.
* USB-C headset (digital, UAC 1.0), connected to the native USB port (GPIO 19/20). The headset must get
  5 V on VBUS: use an OTG adapter with power, or wire 5 V to the port.
* Console/flashing goes through the other USB port (UART bridge).

| Pin | Use | Notes |
|---|---|---|
| GPIO 0 | BOOT button | Talk on/off, long press = ping |
| GPIO 4 | External talk button | Optional, to GND |
| GPIO 1 / 2 | Volume up / down | Optional, to GND |
| GPIO 48 | WS2812 LED | DevKitC v1.1 uses GPIO 38. Change `cfg::kPinLed` in `src/config.hpp` |
| GPIO 19 / 20 | USB D- / D+ | Headset |

## Using it

| Action | Result |
|---|---|
| Click BOOT, the external button or the headset button | Mic on/off. Rising chime = on, falling chime = off. Same chime and LED for every button. |
| Long press (1.5 s) | Ping all nodes, results on the console |
| Vol +/- (pins or headset) | ±10 %, saved. A higher beep means louder. A beep at the limit means it can't go further. |
| Another node turns its mic on/off | Short high/low beep |
| Headset plugged in | Rising chime once the speaker is ready |
| You send a ping | Short high beep (1800 Hz) |

With the mic on, audio is only sent while you speak (voice gate, 400 ms hangover). The mic turns itself off
after 2 minutes open.

| LED | Meaning |
|---|---|
| Blue | No headset speaker (also the colour at boot) |
| Off | Ready, mic muted |
| Amber | Mic on, waiting for speech |
| Red | Sending your voice |
| Green | Playing someone else |
| White | Loopback test |
| Cyan flash | Answered a ping |
| Red, not changing | Radio or settings storage failed at boot: the node is halted (see the console log) |

## Build and flash

Needs [PlatformIO](https://platformio.org/). The first build downloads ESP-IDF and the USB components.

```bash
# set your own network key once, the same on every node (32 hex characters)
export PLATFORMIO_BUILD_FLAGS='-DWT_NETWORK_KEY_HEX=\"'$(openssl rand -hex 16)'\"'
pio run -e esp32s3 -t upload -t monitor
```

Without `WT_NETWORK_KEY_HEX` the build uses a public default key, so anyone running this firmware can listen.
The build prints a warning and the device logs one at every boot when that happens.

Every node must run this firmware: the packet format (v3) does not work with the old 2.x Arduino firmware.

## Console (115200 baud)

```
t         mic on/off                 s         status, peers, counters
+ / -     volume                     ch N      channel 1..13 (same on all nodes, saved)
p         ping                       name X    node name, up to 11 chars (saved)
loop      hear yourself through the codec (headset check, no second node needed)
selftest  check the headset streams and the audio path, prints PASS/FAIL
reboot    restart                    anything else prints this list
```

## Testing

```bash
pio test -e native     # 55 host tests (46 core + 9 driver), no hardware
```

The host tests run the same code as the firmware. A 1 kHz tone goes from the mic rate through 8 kHz, ADPCM,
the packet format and the jitter buffer to the speaker rate. The test checks its level and SNR for every pair
of common headset rates, and with packet loss. They also cover format selection for headsets without 48 kHz,
stereo-mic handling (including no level flutter), out-of-order and duplicate frames, resync after a long
gap, a slow sender losing no frames, latency draining back after a stall, click-free loss concealment, the
replay window (including after a peer was pruned or evicted, and with more than 8 nodes), the 3-node relay
path, HID descriptors from four headset layouts with buttons across report ids, button debounce and long
press, the voice gate (sustained sound, boot hiss, fan noise), AGC not pumping hiss, and mixing two loud
talkers. The driver tests cover rate feedback from a fast and a slow DAC over 10 minutes, bad feedback
values, both feedback formats, packet size limits, feedback endpoint detection, and picking the right
alternate setting for 24-bit in 3 vs 4 bytes. Each of these was checked to fail on the code before its fix. The tests also pass under
AddressSanitizer and UBSan.

Not covered by any test: the ESP-IDF layer in `src/` (USB host, ESP-NOW, tasks, LED) is only checked by the
compiler. On a device:

1. `selftest`: the headset's mic and speaker streams are up, the mic delivers at its rate and is not digital
   silence, and a 1 kHz tone through the software voice path at the headset's rates comes out above 18 dB
   SNR. It does not prove the sound is clean in your ear: step 2 does.
2. `loop`, then speak: you hear yourself about 0.2 s later. This checks mic and speaker by ear.
3. Two nodes in one room: talk both ways, `s` on each shows the other as `direct`. With an asynchronous
   headset, `s` also shows `speaker clock 48000.xxx Hz (async, N feedback updates)` and N keeps rising.
4. Three nodes, A and C out of range of each other: `s` on A shows C as `relay`, and A and C hear each other.

If a headset fails, the boot log lists every format it offers (`usb: mic alt 1: ...`). Include that log in a
bug report.

## Layout

```
lib/wt_core/src/   hardware-free core, built for both the ESP32 and the host tests
  dsp           resampler, filters, AGC, limiter, feedback tones
  adpcm         IMA-ADPCM codec
  format        pick a USB stream format, PCM <-> mono
  jitter        per-talker jitter buffer, loss concealment, drift control
  protocol      wire format, replay window
  mesh          peer table, relay queue
  hid_buttons   HID report descriptor parser
  button        debounce, click, long press
  voice_gate    voice detection, pipeline self-test
components/usb_host_uac/  Espressif UAC host driver 1.5.0 + local patches (PATCHES.md)
src/               ESP-IDF firmware
  usb_headset   USB host: UAC mic + speaker, HID buttons
  audio         mic task, speaker task, mixer
  radio         ESP-NOW, AES-CCM, relay
  led, config, main (buttons, beacons, console)
test/test_core/    host tests for the core
test/test_uac/     host tests for the driver patches (rate feedback, alt selection)
```

## Protocol (v3)

16-byte header `'W' ver type ttl sender[4] session[4] seq[4]`, then the AES-128-CCM ciphertext and an
8-byte tag. The header with ttl zeroed is authenticated; receivers clamp ttl to 3, so a rebroadcast with a
higher ttl cannot travel further than 2 relays. The nonce is `sender|session|seq`. `session` is a boot counter
in NVS that starts at a random value in [2^30, 2^31) after a flash erase, and the radio stays off if the new
value cannot be stored, so a nonce is not reused. Not compatible with firmware 2.x.

## Limits

* Untested on hardware. Everything in `src/` (USB host, ESP-NOW, tasks) has only been compiled and reviewed.
* One shared key per group. A node that leaks the key lets anyone join. Use your own key and keep it out
  of the repo.
* 4 simultaneous talkers. A fifth is dropped until one stops. Peer list holds 8 nodes; larger groups still
  work but the status list only shows 8.
* A sender silent for over 10 minutes is forgotten: one old frame of it could then be replayed once.
* Two nodes whose MACs share the last 4 bytes collide (one ignores the other). Rare; not detected.
* The USB audio driver is a patched copy of Espressif's `usb_host_uac` 1.5.0 in `components/usb_host_uac`
  (rate feedback, exact alternate setting). See `PATCHES.md` there before updating it.
* No OTA update yet (the partition table keeps a second slot for it). Flash over USB.
* UAC 2.0-only headsets are not supported (the ESP-IDF UAC host driver is UAC 1.0). Most USB-C headsets
  are UAC 1.0.
* Range: 6 Mbit/s ESP-NOW at 20 dBm reaches roughly 100-300 m line of sight per hop (not measured with this
  build). `cfg::kLongRange` switches to Espressif LR (about 2x range, but roughly 12x the airtime per
  frame, so fewer talkers fit). Check the transmit power limit where you use it (`cfg::kTxPowerQdBm`).
