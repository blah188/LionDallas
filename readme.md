# LionDallas

Compact **DS18B20** driver over OneWire for the **Lion\*** stack (ESP8266).

- **Search once, keep the addresses.** Sensors are discovered at startup and then
  read by `MatchROM`. This cures the classic "the second sensor drops off after a
  while" failure on long buses, where a periodic `SearchROM` intermittently misses
  a device.
- **Non-blocking reads** inside a `LionCriticalTask`: each tick reads the result of
  the previous conversion, then kicks off the next one.
- **Last-good caching with a fault window.** A transient `-127`/fault reading is
  ignored for `TOLERANCE_127_MS` (the last good value is held); only after the
  window elapses is the fault exposed.
- **Real fault detection**, ported from DallasTemperature 4.0.6: power-on-reset
  (factory `0x0550`), insufficient power, and open / short-to-GND / short-to-VDD.
- **Optional fixed addresses** to skip `SearchROM` entirely on long/noisy wiring.

> **DS18B20 family (`0x28`) only.** Other 1-Wire families are ignored.

## Requirements / contract

LionDallas is part of the Lion\* stack and is **ESP8266 only**. It relies on two
sibling libraries, which the consumer must wire up:

- **LionLogger** — you **must define one global named `Logger`** (declared `extern`
  inside `<Logger.h>`). LionDallas logs discovery and diagnostics through it.
- **LionTask** — `LionDallas` is a `LionCriticalTask`, so you **must pump
  `LionTask::Loop()` from `loop()`** for periodic reads to run.

> On ESP32 there is no `LionCriticalTask` (that scheduler is `LionRtosTask`), so
> this library is not built for ESP32.

The constructor performs a **blocking** discovery + first read (~conversion time,
up to ~770 ms at 12-bit). Create LionDallas once (global/static or `new`); it is
**non-copyable** (it owns the `OneWire` instance).

`GetTemperature()` returns `-127` both for a disconnected sensor and for an
out-of-range index — treat any value `< -100` as "no reading".

## Install

PlatformIO (`platformio.ini`):

```ini
lib_deps = leva/LionDallas
```

This pulls in `paulstoffregen/OneWire`, `leva/LionArray`, `leva/LionLogger` and
`leva/LionTask` automatically.

## Usage

```cpp
#include <Logger.h>      // LionLogger
#include <LionTask.h>    // LionTask scheduler
#include <LionDallas.h>

MyLogger Logger(ILogger::SerialPort, ILogger::LvlDebug);   // required global

LionDallas *dallas = nullptr;

void setup() {
    Serial.begin(115200);
    Logger.Setup();
    LionTask::Setup();
    dallas = new LionDallas(/*pin*/ 4, /*intervalMs*/ 2000);   // auto-discover
}

void loop() {
    LionTask::Loop();   // drives the periodic reads

    for (int i = 0; i < dallas->GetSenorCount(); i++)
        Serial.printf("[%d] = %.2f C\n", i, dallas->GetTemperature(i));
}
```

### Fixed addresses (skip SearchROM)

On a long/noisy bus, pass known ROMs to read by `MatchROM` only:

```cpp
static const DeviceAddress addrs[] = {
    {0x28, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x01},
    {0x28, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77},
};
dallas = new LionDallas(4, 2000, addrs, 2);
```

## API

| Member | Description |
| --- | --- |
| `LionDallas(pin, intervalMs, fixedAddrs = nullptr, fixedCount = 0, resolution = 12)` | Construct, discover (or use `fixedAddrs`), set resolution, take a first reading. |
| `int GetSenorCount()` | Number of sensors found/stored. |
| `float GetTemperature(int idx = 0)` | Cached °C for sensor `idx`; `-127` if invalid/out-of-range. |
| `bool IsParasite() const` | `true` if any sensor is parasite-powered. |
| `void GetAddressHex(int idx, char *out)` | Sensor ROM as 16 hex chars into `out` (caller provides `char[17]`); empty string if `idx` out of range. |

### Compile-time tunables (define before including, or via `build_flags`)

| Macro | Default | Meaning |
| --- | --- | --- |
| `DALLAS_FIND_RETRIES` | `5` | SearchROM passes; the fullest pass wins. |
| `DALLAS_TEMP_RETRIES` | `3` | Scratchpad re-reads per sensor per tick. |
| `TOLERANCE_127_MS` | `10000` | How long a last-good value is held over a fault. |

## License

MIT — see [LICENSE](LICENSE). Portions of `src/LionDallasPort.cpp` are ported from
the [Arduino Temperature Control Library](https://github.com/milesburton/Arduino-Temperature-Control-Library)
(DallasTemperature), © Miles Burton, MIT.
