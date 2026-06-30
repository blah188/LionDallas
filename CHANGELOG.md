# Changelog

All notable changes to this project are documented here.
The format is based on [Keep a Changelog](https://keepachangelog.com/).

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
