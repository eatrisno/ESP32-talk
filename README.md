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
| Headset | Any USB Audio Class 1.0 headset. It picks the best format the headset offers (48/44.1/32/16 kHz…, mono or stereo, 16/24/32-bit). |
| Headset buttons | Play/Pause, Hook and Mute on the headset act the same as the BOOT button. Its Vol+ and Vol- change volume. |
| Voice | 8 kHz, IMA-ADPCM (32 kbit/s), 40 ms frames. Mic has a DC filter, automatic gain and a voice gate, so silence is not sent. |
| Losses | Each frame carries its own decoder state, so a lost packet costs 40 ms and does not corrupt the frames after it. |
| Security | AES-128-CCM on every frame (encryption and authentication). Replay protection per sender. |

## Hardware

* ESP32-S3 board with native USB (tested target: ESP32-S3-DevKitC-1).
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

With the mic on, audio is only sent while you speak (voice gate, 400 ms hangover). The mic turns itself off
after 2 minutes open.

| LED | Meaning |
|---|---|
| Blue | No headset |
| Off | Ready, mic muted |
| Amber | Mic on, waiting for speech |
| Red | Sending your voice |
| Green | Playing someone else |
| White | Loopback test |
| Cyan flash | Answered a ping |

## Build and flash

Needs [PlatformIO](https://platformio.org/). The first build downloads ESP-IDF and the USB components.

```bash
# set your own network key once, the same on every node (32 hex characters)
export PLATFORMIO_BUILD_FLAGS='-DWT_NETWORK_KEY_HEX=\"'$(openssl rand -hex 16)'\"'
pio run -e esp32s3 -t upload -t monitor
```

Without `WT_NETWORK_KEY_HEX` the build uses a public default key, so anyone running this firmware can listen.

## Console (115200 baud)

```
t         mic on/off                 s         status, peers, counters
+ / -     volume                     ch N      channel 1..13 (same on all nodes)
p         ping                       name X    node name
loop      hear yourself through the codec (headset check, no second node needed)
selftest  check the headset streams and the audio path, prints PASS/FAIL
```

## Testing

```bash
pio test -e native     # 35 host tests, no hardware
```

The host tests run the same code as the firmware. A 1 kHz tone goes from the mic rate through 8 kHz, ADPCM,
the packet format and the jitter buffer to the speaker rate. The test checks its level and SNR for every pair
of common headset rates, and with packet loss. They also cover format selection for headsets without 48 kHz,
stereo-mic handling, out-of-order and duplicate frames, clock drift, the replay window, the 3-node relay
path, HID descriptors from three headset layouts, button debounce and long press, and the voice gate.

On a device, run `selftest`, then `loop`, then a 3-node walk test. Things the tests cannot prove (USB
enumeration of a given headset, RF range, how it sounds) still need the hardware.

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
src/               ESP-IDF firmware
  usb_headset   USB host: UAC mic + speaker, HID buttons
  audio         mic task, speaker task, mixer
  radio         ESP-NOW, AES-CCM, relay
  led, config, main (buttons, beacons, console)
test/test_core/    host tests
```

## Protocol (v3)

16-byte header `'W' ver type ttl sender[4] session[4] seq[4]`, then the AES-128-CCM ciphertext and an
8-byte tag. The header with ttl zeroed is authenticated. The nonce is `sender|session|seq`, and `session`
is a boot counter in NVS, so a nonce is never reused. Not compatible with firmware 2.x.

## Limits

* One shared key per group. A node that leaks the key lets anyone join. Use your own key and keep it out
  of the repo.
* 4 simultaneous talkers. A fifth is dropped until one stops.
* UAC 2.0-only headsets are not supported (the ESP-IDF UAC host driver is UAC 1.0). Most USB-C headsets
  are UAC 1.0.
* Range: 6 Mbit/s ESP-NOW at 20 dBm reaches roughly 100-300 m line of sight per hop (not measured with this
  build). `cfg::kLongRange` switches to Espressif LR (about 2x range, but roughly 12x the airtime per
  frame, so fewer talkers fit). Check the transmit power limit where you use it (`cfg::kTxPowerQdBm`).
