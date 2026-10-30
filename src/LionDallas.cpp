//
// LionDallas.cpp — the Lion stack's own logic: search-once + stored addresses,
// non-blocking reads in the scheduler task, cache + TOLERANCE_127_MS.
// Primitives ported from DallasTemperature (MIT) live in LionDallasPort.cpp.
//
#include "LionDallas.h"

#define STARTCONVO 0x44 // start temperature conversion (broadcast)

// DS18B20 family code and the "bad value" threshold.
#define DS18B20_FAMILY 0x28
#define BAD_TEMP_C     -100 // anything below is a fault sentinel (-127/-250/-251/-25x)

LionDallas::LionDallas(uint8_t pin, int tempInterval,
                       const DeviceAddress *fixedAddrs, uint8_t fixedCount,
                       uint8_t resolution, bool internalPullup)
    :
#ifdef LIOND_RTOS
      RtosTask(false, tempInterval), // EveryMs FreeRTOS task (started by Setup below)
#else
      LionCriticalTask(F("DS"), tempInterval),
#endif
      _addr(3), _temperatures(3), _times(3)
{
    char rom[17];
    _ds = new OneWire(_pin = pin);
    if (internalPullup)
        pinMode(pin, INPUT_PULLUP); // after OneWire: its pinMode(INPUT) clears the pull-up
    _resolution = resolution;

    // 1) Search once + store (or fixedAddrs — long line).
    searchAndStore(fixedAddrs, fixedCount);

    // 2) Parasite detection (any sensor in parasite mode switches the whole bus).
    if (_addr.Length() > 0)
        _parasite = readPowerSupply(nullptr);

    // 3) Resolution for every sensor.
    for (int i = 0; i < _addr.Length(); i++)
        applyResolution(_addr[i], _resolution);

    // Ensure pollMs >= convMs(res): otherwise the conversion won't finish → POR/garbage.
    uint16_t cms = convMs(_resolution);
    if ((uint32_t)tempInterval < cms)
        Logger.Log_P(ILogger::LvlWarning,
                     PSTR("LionDallas: pollMs %d < convMs %u (res %d) - conversion won't finish"),
                     tempInterval, cms, _resolution);

    Logger.Log_P(ILogger::LvlDebug,
                 PSTR("LionDallas pin %d: %d devices, parasite=%d, res %d"),
                 pin, _addr.Length(), _parasite, _resolution);

    // 4) Initial (blocking) read so the cache is never empty.
    _temperatures.Reserve(_addr.Length());
    _times.Reserve(_addr.Length());
    startConversion();
    delay(cms + 20);
    for (int i = 0; i < _addr.Length(); i++)
    {
        float t = readSensor(_addr[i]);
        _temperatures[i] = t;
        _times[i] = millis();
        romHex(_addr[i], rom);
        Logger.Log_P(ILogger::LvlDebug, PSTR("Dallas[%d] (%s) = %.2f"), i, rom, t);
    }
    // Start the next conversion — it will be ready by the task's first tick.
    startConversion();

#ifdef LIOND_RTOS
    // LionCriticalTask (ESP8266, ESP32 with LIONDALLAS_LION_TASKS) has already registered
    // itself in the base constructor and is ticked from LionTask::Loop(). RtosTask does
    // not start on its own — spawn the EveryMs FreeRTOS task here, once the cache is
    // filled. The first TaskBody() runs after tempInterval ms (vTaskDelayUntil).
    Setup("DS", LIOND_TASK_CORE, LIOND_TASK_PRIO, LIOND_TASK_STACK);
#endif
}

LionDallas::~LionDallas()
{
    delete _ds;
}

int LionDallas::GetSenorCount()
{
    return _addr.Length();
}

float LionDallas::GetTemperature(int idx)
{
    if (idx < 0 || idx >= _addr.Length())
        return -127;
    return _temperatures[idx];
}

void LionDallas::GetAddressHex(int idx, char *out)
{
    if (idx < 0 || idx >= _addr.Length())
    {
        out[0] = 0;
        return;
    }
    romHex(_addr[idx], out);
}

// Non-blocking read: first read the previous conversion's result by stored
// address, then start the next one. Since tempInterval (≈2000 ms) ≫ convMs
// (≤750 ms), the conversion is always ready by the next tick — no state machine.
#ifdef LIOND_RTOS
void LionDallas::TaskBody() { pollAndConvert(); }      // FreeRTOS EveryMs step
#else
void LionDallas::TaskInnerBody() { pollAndConvert(); } // LionCriticalTask tick
#endif

void LionDallas::pollAndConvert()
{
    for (int i = 0; i < _addr.Length(); i++)
    {
        float t = readSensor(_addr[i]);
        if (t < BAD_TEMP_C)
        {
            // Bad/-127: keep the previous value for TOLERANCE_127_MS more.
            if (millis() - _times[i] < TOLERANCE_127_MS)
                continue;
            else if (_temperatures[i] > BAD_TEMP_C)
            {
                char rom[17];
                romHex(_addr[i], rom);
                Logger.Log_P(ILogger::LvlWarning,
                             PSTR("Dallas #%d (%s) bad %.0f for %ld ms"),
                             i, rom, t, (long)(millis() - _times[i]));
            }
        }
        _temperatures[i] = t;
        _times[i] = millis();
    }
    startConversion();
}

// --- own code ---

// Search once + store. fixedAddrs → skip SearchROM (long line).
// Otherwise a collecting search: repeat the pass, keep the most complete result.
bool LionDallas::searchAndStore(const DeviceAddress *fixedAddrs, uint8_t fixedCount)
{
    char rom[17];
    if (fixedAddrs && fixedCount)
    {
        for (uint8_t i = 0; i < fixedCount; i++)
        {
            memcpy(_addr[i], fixedAddrs[i], sizeof(DeviceAddress));
            romHex(_addr[i], rom);
            Logger.Log_P(ILogger::LvlDebug, PSTR("Fixed dallas #%d (%s)"), i, rom);
        }
        return _addr.Length() > 0;
    }

    const int MAX_DS = 8;
    int best = 0;
    for (int tn = 0; tn < DALLAS_FIND_RETRIES; tn++)
    {
        DeviceAddress tmp[MAX_DS];
        int found = 0;
        _ds->reset_search();
        DeviceAddress addr;
        while (found < MAX_DS && _ds->search(addr))
        {
            if (addr[0] == DS18B20_FAMILY && OneWire::crc8(addr, 7) == addr[7])
                memcpy(tmp[found++], addr, sizeof(DeviceAddress));
        }
        if (found > best) // remember the most complete pass
        {
            _addr.Clear();
            for (int i = 0; i < found; i++)
                memcpy(_addr[i], tmp[i], sizeof(DeviceAddress));
            best = found;
        }
        else
            Logger.Log_P(ILogger::LvlDebug, PSTR("Search pass %d/%d: %d found (best %d)"),
                         tn + 1, DALLAS_FIND_RETRIES, found, best);
        delay(50);
    }
    for (int i = 0; i < best; i++)
    {
        romHex(_addr[i], rom);
        Logger.Log_P(ILogger::LvlDebug, PSTR("Got dallas #%d (%s)"), i, rom);
    }
    return best > 0;
}

// requestTemperatures: reset+skip+STARTCONVO to all (non-blocking).
void LionDallas::startConversion()
{
    _ds->reset();
    _ds->skip();
    _ds->write(STARTCONVO, _parasite ? 1 : 0);
}

// getTemp wrapper: readScratch (+crc) → calcRaw → °C, with retries.
float LionDallas::readSensor(const uint8_t *rom)
{
    uint8_t sp[9];
    for (uint8_t r = 0; r <= DALLAS_TEMP_RETRIES; r++)
        if (readScratch(rom, sp))
            return rawToC(calcRaw(rom, sp));
    return -127; // DEVICE_DISCONNECTED_C
}

// ROM 8 bytes -> 16 hex chars into a stack buffer out[17]. No String/heap —
// logging only. One sprintf_P, byte by byte (endianness irrelevant).
void LionDallas::romHex(const uint8_t *rom, char *out)
{
    sprintf_P(out, PSTR("%02X%02X%02X%02X%02X%02X%02X%02X"),
              rom[0], rom[1], rom[2], rom[3], rom[4], rom[5], rom[6], rom[7]);
}
