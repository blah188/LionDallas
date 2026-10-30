#pragma once
//
// LionDallas — compact DS18B20 library on top of OneWire for the Lion* stack.
// Drop-in replacement for LocalLibs/DallasTemp: same public API (pin, read period,
// optional fixedAddrs). Addresses are searched once at startup and stored
// (cures "the second sensor drops out").
//
// The scheduler is chosen per platform:
//   * ESP8266 — LionCriticalTask (LionTask), ticked from LionTask::Loop();
//   * ESP32   — RtosTask of type EveryMs (LionRtosTask), a dedicated FreeRTOS task,
//               no pumping from loop() needed;
//   * ESP32 + -D LIONDALLAS_LION_TASKS — LionCriticalTask, as on ESP8266. For
//               cooperative ESP32 builds (synchronous web server, LionTasks): a
//               separate FreeRTOS task is redundant there, and with a synchronous
//               LionLogger it is forbidden — there must be exactly one logging task.
//               A flag rather than auto-detection via LION_TASKS: the library is
//               compiled in its own TU and does not see the consumer's headers.
//               The consumer declares LionTask in lib_deps itself.
// The public API is identical in all variants.
//
// Lifetime: create once (global/static/new) and keep for the whole program.
// On ESP32 (own task) the object also owns a FreeRTOS task, so do NOT delete it
// from another context (race with the reading task over _ds); the design is
// one-shot discovery anyway.
//
// The scratchpad/conversion/fault-detection algorithms are ported from
// DallasTemperature 4.x (Miles Burton, MIT) — see LionDallasPort.cpp.
// This header holds declarations only; no ported code bodies here.
//
#include <OneWire.h>
#include <Array.h>
#include <Logger.h>

// Own FreeRTOS task — only on ESP32 and only without LIONDALLAS_LION_TASKS. Below this
// point branch ONLY on LIOND_RTOS, never on ESP32.
#if defined(ESP32) && !defined(LIONDALLAS_LION_TASKS)
#define LIOND_RTOS 1
#endif

#ifdef LIOND_RTOS
#include <RtosTask.h> // RtosTask (LionRtosTask) — not available on ESP8266
#define LIOND_TASK_BASE RtosTask
// FreeRTOS parameters of the reading task (overridable via build_flags).
#ifndef LIOND_TASK_CORE
#define LIOND_TASK_CORE tskNO_AFFINITY
#endif
#ifndef LIOND_TASK_PRIO
#define LIOND_TASK_PRIO 5
#endif
#ifndef LIOND_TASK_STACK
#define LIOND_TASK_STACK 4096
#endif
#else
#include <LionTask.h> // LionCriticalTask (LION_TASKS)
#define LIOND_TASK_BASE LionCriticalTask
#endif

#ifndef DALLAS_FIND_RETRIES
#define DALLAS_FIND_RETRIES 5
#endif
#ifndef DALLAS_TEMP_RETRIES
#define DALLAS_TEMP_RETRIES 3
#endif
#ifndef TOLERANCE_127_MS
#define TOLERANCE_127_MS 10000l
#endif

// Same type/name as in DallasTemperature, so existing
// `static const DeviceAddress addrs[]` tables compile unchanged.
typedef uint8_t DeviceAddress[8];

class LionDallas : LIOND_TASK_BASE
{
public:
    // Compatible with DallasTemp: pin, read period, optional known addresses
    // (given → skip the unreliable SearchROM, read via MatchROM).
    // internalPullup — the chip's internal pull-up instead of an external 4.7–10 kOhm
    // resistor. OneWire's constructor calls pinMode(INPUT), which clears it; after that
    // it only switches the pin direction, so the pull-up enabled here stays on.
    // It is weak (~45 kOhm on ESP32, ~30–100 on 8266): fine for a short wire and a
    // single sensor; on a long line ones will start reading as zeros.
    LionDallas(uint8_t pin, int tempInterval,
               const DeviceAddress *fixedAddrs = nullptr, uint8_t fixedCount = 0,
               uint8_t resolution = 12, bool internalPullup = false);
    ~LionDallas();

    // Owns a raw OneWire* (_ds, deleted in the destructor) — a copy = double free.
    // Copying is already implicitly disabled (Array members are non-copyable), but
    // it is declared explicitly: the contract is visible and survives a refactor
    // that removes the Array members.
    LionDallas(const LionDallas &) = delete;
    LionDallas &operator=(const LionDallas &) = delete;

    int   GetSenorCount();
    float GetTemperature(int idx = 0); // cached; -127 if invalid/out of range
    bool  IsParasite() const { return _parasite; }
    // ROM of sensor idx -> hex into a caller's buffer out[17] (status/diagnostics).
    // Empty string if idx is out of range.
    void  GetAddressHex(int idx, char *out);

protected:
    // Tick method of the base scheduler for this platform; both call pollAndConvert().
#ifdef LIOND_RTOS
    void TaskBody() override;      // step of the EveryMs FreeRTOS task (LionRtosTask)
#else
    void TaskInnerBody() override; // LionCriticalTask tick (pumped by LionTask::Loop())
#endif

private:
    void pollAndConvert(); // read all (by address) -> start the next conversion
    OneWire *_ds;
    uint8_t  _pin;
    uint8_t  _resolution;
    bool     _parasite = false;
    Array<DeviceAddress> _addr;         // stored addresses (found once)
    Array<float>         _temperatures; // cache of the last valid values
    Array<uint32_t>      _times;        // time of each value — for TOLERANCE_127_MS

    // --- own code (Lion stack) ---
    bool   searchAndStore(const DeviceAddress *fixedAddrs, uint8_t fixedCount); // collecting + keep-best / fixedAddrs
    void   startConversion();              // STARTCONVO broadcast (non-blocking)
    float  readSensor(const uint8_t *rom); // getTemp wrapper with retries -> °C
    // ROM 8 bytes -> hex into a stack buffer out[17] (logging only; no String/heap).
    static void romHex(const uint8_t *rom, char *out);

    // --- ported from Miles Burton (DallasTemperature 4.x, MIT) — bodies in LionDallasPort.cpp ---
    bool     readScratch(const uint8_t *rom, uint8_t *sp);        // reset+select+READSCRATCH +crc8/all-zeros
    void     writeScratch(const uint8_t *rom, const uint8_t *sp); // reset+select+WRITESCRATCH (TH,TL,CFG)
    int32_t  calcRaw(const uint8_t *rom, uint8_t *sp);            // raw + POR/insufficient/fault detection (4.0.6)
    void     applyResolution(const uint8_t *rom, uint8_t res);    // writeScratch config 9..12 bits
    bool     readPowerSupply(const uint8_t *rom);                 // parasite detection
    static float    rawToC(int32_t raw);                          // raw -> °C, fault sentinels
    static uint16_t convMs(uint8_t res);                          // 94/188/375/750 ms
};
