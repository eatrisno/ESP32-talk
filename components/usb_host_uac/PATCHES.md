# Local patches to espressif/usb_host_uac 1.5.0

This is Espressif's USB Host UAC driver 1.5.0 (Apache-2.0, see LICENSE) with two fixes the
walkie-talkie needs. The diff against the registry release is in `local.patch`; the added
files are `uac_feedback.c` and `uac_feedback.h`. Upstream baseline: `uac_host.c` sha256
`e765972a3afc979b14a278d381ad055af0a234e2981b9a72108e8e4c9cad5a59`.

## 1. Asynchronous speakers: rate feedback

Upstream sends every OUT packet at the nominal size and ignores the feedback endpoint. An
asynchronous headset runs its DAC from its own crystal, so after a while its buffer over- or
underflows and it clicks.

- The descriptor parser now records the feedback IN endpoint of an asynchronous OUT alternate
  setting (it used to overwrite the data endpoint with it).
- `uac_host_device_start` claims and polls it; each 10.14 (full speed) or 16.16 (high speed)
  value within 1% of nominal sets the packet size. Values outside that (0 before the device
  locks, or nonsense) are ignored and the last good rate stays.
- `uac_host_device_feedback()` reports whether feedback is in use and the device rate.

## 2. Alternate setting selection

Upstream matched only channels and bit resolution, ignored the subframe size, and for
discrete rates took the last matching alt. A headset offering 24-bit in both a 3-byte and a
4-byte container could be started in the wrong layout (noise).

- `uac_host_stream_config_t` gains `subframe_size` and `alt_idx` (0 = any, so existing
  callers behave as before apart from first-match).
- Matching uses all of them and returns the first match.

The decision logic is in `uac_feedback.c` as plain C and is covered by `test/test_uac`
(`pio test -e native`). Each test was checked to fail with upstream's behaviour.

## Updating

To move to a newer upstream release, apply `local.patch` to it (or drop it if upstream gained
feedback support and subframe matching), then run the host tests and a device check with an
asynchronous headset (`s` shows `speaker clock ... (async ...)`).
