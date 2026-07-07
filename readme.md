# LionDallas

Compact **DS18B20** driver over OneWire for the **Lion\*** stack (ESP8266 & ESP32).

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

LionDallas is part of the Lion\* stack and runs on **ESP8266 and ESP32**. It
relies on sibling libraries the consumer must wire up:

- **LionLogger** — you **must define one global named `Logger`** (declared `extern`
  inside `<Logger.h>`). LionDallas logs discovery and diagnostics through it.
  **On ESP32, build with `-D ASYNC_LOG`** (recommended: offloads filesystem
  writes to a FreeRTOS task and satisfies LionLogger's log-clearing requirement;
  otherwise `<Logger.h>` fails to compile — `-D NO_LOG_CLEARING` is the minimal
  alternative if you don't want the async log task).
- **Scheduler — platform-dependent, chosen automatically:**
  - **ESP8266:** `LionDallas` is a `LionTask` `LionCriticalTask`, so you **must
    pump `LionTask::Loop()` from `loop()`** for the periodic reads to run.
  - **ESP32:** `LionDallas` runs its reads in its own `LionRtosTask` FreeRTOS
    task (type `EveryMs`), spawned by the constructor. **Nothing to pump.**

The constructor performs a **blocking** discovery + first read (~conversion time,
up to ~770 ms at 12-bit) in the caller's context. Create LionDallas once
(global/static or `new`); it is **non-copyable** (it owns the `OneWire` instance).
On **ESP32** it also owns a FreeRTOS task, so keep the instance alive for the
program's lifetime — don't `delete` it (a delete from another context would race
the read task).

> **ESP32 concurrency:** `GetTemperature()` / `GetSenorCount()` /
> `GetAddressHex()` may be called from any task while the read task updates the
> cache. Reads are lock-free: each cached slot is an aligned 32-bit
> `float`/`uint32_t` (atomic load/store on Xtensa), so a caller always sees the
> previous or the new value, never a torn one.

### ESP32 task tunables

Define before including `<LionDallas.h>` or via `build_flags`:

| Macro | Default | Meaning |
| --- | --- | --- |
| `LIOND_TASK_CORE` | `tskNO_AFFINITY` | FreeRTOS core for the read task. |
| `LIOND_TASK_PRIO` | `5` | Task priority. |
| `LIOND_TASK_STACK` | `4096` | Task stack size (bytes). |

`GetTemperature()` returns `-127` both for a disconnected sensor and for an
out-of-range index — treat any value `< -100` as "no reading".

## Install

PlatformIO (`platformio.ini`):

```ini
; ESP8266
lib_deps = leva/LionDallas

; ESP32
lib_deps = leva/LionDallas
build_flags = -D ASYNC_LOG              ; required by LionLogger on ESP32
```

This pulls in `paulstoffregen/OneWire`, `leva/LionArray`, `leva/LionLogger`, plus
`leva/LionTask` (ESP8266) or `leva/LionRtosTask` (ESP32) automatically.

## Usage

```cpp
#include <Logger.h>      // LionLogger
#ifndef ESP32
#include <LionTask.h>    // ESP8266 scheduler (pumped from loop())
#endif
#include <LionDallas.h>

MyLogger Logger(ILogger::SerialPort, ILogger::LvlDebug);   // required global

LionDallas *dallas = nullptr;

void setup() {
    Serial.begin(115200);
    Logger.Setup();
#ifndef ESP32
    LionTask::Setup();
#endif
    dallas = new LionDallas(/*pin*/ 4, /*intervalMs*/ 2000);   // auto-discover
}

void loop() {
#ifndef ESP32
    LionTask::Loop();   // ESP8266: drives the periodic reads
#endif
    // ESP32: reads run in LionDallas's own FreeRTOS task — nothing to pump.

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

## Troubleshooting — `SearchROM` reliability on ESP8266

**Symptom.** On some builds, startup discovery reports **0 devices** even with a
healthy sensor on a *short* bus (e.g. a D1-mini DS18B20 shield). The result is
**deterministic for a given binary** — retries within a run and power-cycles do
**not** change it — but it **flips on recompile** (a rebuild usually starts
finding the sensor again). Reading the *same* sensor by **fixed address
(`MatchROM`) works reliably** on the very binaries where `SearchROM` finds nothing.

**What it is (and isn't).** It is not the wiring (reproduces on a shield with short
traces), not Wi-Fi interrupts (reproduces *before* `WiFi.begin`), and not a "cold
bus" (the `reset()` presence pulse is detected fine). It looks like a
**timing-margin issue in the 64-bit `SearchROM` enumeration**: `SearchROM` has *no*
tolerance for a single mis-sampled read slot — one bad bit sends the search down the
wrong branch and the whole enumeration aborts — whereas a scratchpad read survives
the occasional bad bit via its CRC check plus `DALLAS_TEMP_RETRIES`. The exact
timing margin shifts with code layout, so one binary is consistently good and the
next consistently bad.

**Recommended fix.** For nodes with **known sensors, use fixed addresses**
(see *Fixed addresses* above). `MatchROM` is deterministic and unaffected by this,
so it is the supported, reliable path — not merely a long-bus workaround.

**Unverified avenue (do NOT assume it fixes this).** A common suspicion is
flash-cache (IROM) jitter in the timing-critical OneWire primitives, "fixable" by
forking `paulstoffregen/OneWire` to mark `reset` / `read_bit` / `write_bit`
`IRAM_ATTR`. **This is unverified here, and there is evidence against it:** those
same primitives back `MatchROM`, which stays reliable on the "bad" binaries — so a
globally-broken bit timing does not fit. If you investigate, treat IRAM placement
as an experiment to measure, not a known cure.

## License

MIT — see [LICENSE](LICENSE). Portions of `src/LionDallasPort.cpp` are ported from
the [Arduino Temperature Control Library](https://github.com/milesburton/Arduino-Temperature-Control-Library)
(DallasTemperature), © Miles Burton, MIT.
