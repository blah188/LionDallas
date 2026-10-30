# Changelog

All notable changes to this project are documented here.
The format is based on [Keep a Changelog](https://keepachangelog.com/).

## [1.2.0] - 2026-10-30

### Added
- `-D LIONDALLAS_LION_TASKS`: on ESP32, run the reads in a `LionTask`
  `LionCriticalTask` pumped from `loop()` — exactly as on ESP8266 — instead of a
  dedicated FreeRTOS task. For cooperative ESP32 builds (synchronous web server,
  LionTask scheduler, synchronous LionLogger): there the extra FreeRTOS task is
  pointless, and a synchronous LionLogger allows exactly one logging task. A
  flag rather than auto-detection because the library compiles in its own
  translation unit and never sees the consumer's headers. With the flag the
  consumer lists `leva/LionTask` in `lib_deps` itself (the manifests scope it to
  esp8266). Default behaviour on both platforms is unchanged.
- Constructor parameter `internalPullup` (last, default `false`): use the chip's
  internal pull-up on the bus pin instead of an external 4.7–10 kΩ resistor. It
  is set right after the `OneWire` constructor, whose `pinMode(INPUT)` would
  otherwise clear it; `OneWire` later toggles only the pin direction, so the
  pull-up survives. Weak (~45 kΩ on ESP32): short wire, single sensor. A
  parameter rather than a build flag because the pull-up belongs to the wiring
  of one bus, not to the whole build.

### Changed
- Source comments and the "conversion won't finish" warning text translated
  to English (were Russian).

## [1.1.0] - 2026-07-03

### Added
- **ESP32 support.** On ESP32 the periodic reads run in a dedicated
  `LionRtosTask` FreeRTOS task (type `EveryMs`), spawned automatically by the
  constructor — no `loop()` pump needed. ESP8266 is unchanged (reads run in a
  `LionTask` `LionCriticalTask`, pumped from `loop()`). The public API is
  identical on both platforms.
- ESP32 FreeRTOS task tunables: `LIOND_TASK_CORE`, `LIOND_TASK_PRIO`,
  `LIOND_TASK_STACK` (define before including / via `build_flags`).

### Changed
- Manifests now declare `espressif32` and platform-scoped dependencies
  (`LionTask` on esp8266, `LionRtosTask` on esp32).

### Notes
- On ESP32 the cached readings are read lock-free from the caller's context
  while the task writes them; this is safe because each slot is an aligned
  32-bit `float`/`uint32_t` (atomic load/store on Xtensa) — a reader sees the
  previous or the new value, never a torn one.
- On ESP32, build with `-D ASYNC_LOG` (recommended; `-D NO_LOG_CLEARING` is a
  minimal alternative). LionLogger's log-clearing path requires `ASYNC_LOG`, so
  without one of these `<Logger.h>` fails to compile. This is a LionLogger
  constraint, surfaced here because LionDallas logs through it.

## [1.0.0] - 2026-06-30

First public release. Extracted from a private monorepo and packaged as a
standalone library.

### Added
- `LionDallas` — DS18B20 driver over OneWire for the Lion* stack.
- Search-once address discovery with a keep-the-fullest-pass strategy, then
  reads by MatchROM (cures "the second sensor drops off" on long buses).
- Optional fixed addresses (`fixedAddrs` / `fixedCount`) to skip the unreliable
  SearchROM entirely on long/noisy wiring.
- Non-blocking periodic reads inside a `LionCriticalTask`.
- Last-good caching with a `TOLERANCE_127_MS` window that rides out transient
  `-127`/fault readings before exposing them.
- Per-sensor resolution (9..12 bit) and parasite-power detection.
- DallasTemperature 4.0.6 fault detection: power-on-reset (factory `0x0550`),
  insufficient-power, and open/short-to-GND/short-to-VDD sentinels.
- `GetSenorCount()`, `GetTemperature(idx)`, `IsParasite()`, `GetAddressHex(idx, out)`.

### Notes
- DS18B20 family (`0x28`) only.
- ESP8266 only — depends on `LionTask`'s `LionCriticalTask` and on `LionLogger`.
- Scratchpad / conversion / resolution / fault-detection primitives are ported
  from the Arduino Temperature Control Library (DallasTemperature),
  (c) Miles Burton, MIT — see `src/LionDallasPort.cpp` and `LICENSE`.
