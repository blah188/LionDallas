//
// LionDallas.cpp — собственная логика Lion-стека: поиск-один-раз + хранение
// адресов, неблокирующее чтение в LionCriticalTask, кэш + TOLERANCE_127_MS.
// Портированные из DallasTemperature (MIT) примитивы — в LionDallasPort.cpp.
//
#include "LionDallas.h"

#define STARTCONVO 0x44 // запустить конверсию температуры (broadcast)

// Семейство DS18B20 и порог «битого» значения.
#define DS18B20_FAMILY 0x28
#define BAD_TEMP_C     -100 // всё ниже — fault-сентинел (-127/-250/-251/-25x)

LionDallas::LionDallas(uint8_t pin, int tempInterval,
                       const DeviceAddress *fixedAddrs, uint8_t fixedCount,
                       uint8_t resolution)
    :
#ifdef ESP32
      RtosTask(false, tempInterval), // EveryMs FreeRTOS-задача (стартует в Setup ниже)
#else
      LionCriticalTask(F("DS"), tempInterval),
#endif
      _addr(3), _temperatures(3), _times(3)
{
    char rom[17];
    _ds = new OneWire(_pin = pin);
    _resolution = resolution;

    // 1) Поиск-один-раз + хранение (или fixedAddrs — длинная линия).
    searchAndStore(fixedAddrs, fixedCount);

    // 2) Parasite-детект (любой датчик в parasite-режиме переводит шину).
    if (_addr.Length() > 0)
        _parasite = readPowerSupply(nullptr);

    // 3) Разрешение каждому датчику.
    for (int i = 0; i < _addr.Length(); i++)
        applyResolution(_addr[i], _resolution);

    // Гарантия pollMs >= convMs(res): иначе конверсия не успеет → POR/мусор.
    uint16_t cms = convMs(_resolution);
    if ((uint32_t)tempInterval < cms)
        Logger.Log_P(ILogger::LvlWarning,
                     PSTR("LionDallas: pollMs %d < convMs %u (res %d) — конверсия не успеет"),
                     tempInterval, cms, _resolution);

    Logger.Log_P(ILogger::LvlDebug,
                 PSTR("LionDallas pin %d: %d devices, parasite=%d, res %d"),
                 pin, _addr.Length(), _parasite, _resolution);

    // 4) Стартовое (блокирующее) чтение, чтобы кэш не был пустым.
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
    // Запустить следующую конверсию — будет готова к первому тику задачи.
    startConversion();

#ifdef ESP32
    // ESP8266: LionCriticalTask уже зарегистрировался в базовом конструкторе и
    // будет тикать из LionTask::Loop(). ESP32: RtosTask сам не стартует —
    // порождаем FreeRTOS-задачу EveryMs здесь, когда кэш уже заполнен. Первый
    // TaskBody() выполнится через tempInterval мс (vTaskDelayUntil).
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

// Неблокирующее чтение: сначала читаем результат прошлой конверсии по
// хранимому адресу, затем запускаем следующую. Т.к. tempInterval (≈2000 мс) ≫
// convMs (≤750 мс), к след. тику конверсия всегда готова — автомат не нужен.
#ifdef ESP32
void LionDallas::TaskBody() { pollAndConvert(); }      // шаг FreeRTOS EveryMs
#else
void LionDallas::TaskInnerBody() { pollAndConvert(); } // тик LionCriticalTask
#endif

void LionDallas::pollAndConvert()
{
    for (int i = 0; i < _addr.Length(); i++)
    {
        float t = readSensor(_addr[i]);
        if (t < BAD_TEMP_C)
        {
            // Битое/-127: держим прошлое значение ещё TOLERANCE_127_MS.
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

// --- своё ---

// Поиск-один-раз + хранение. fixedAddrs → пропустить SearchROM (длинная линия).
// Иначе collecting-search: повторяем проход, оставляем самый полный результат.
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
        if (found > best) // запомнить самый полный проход
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

// requestTemperatures: reset+skip+STARTCONVO всем (неблокирующий).
void LionDallas::startConversion()
{
    _ds->reset();
    _ds->skip();
    _ds->write(STARTCONVO, _parasite ? 1 : 0);
}

// getTemp-обёртка: readScratch (+crc) → calcRaw → °C, с ретраями.
float LionDallas::readSensor(const uint8_t *rom)
{
    uint8_t sp[9];
    for (uint8_t r = 0; r <= DALLAS_TEMP_RETRIES; r++)
        if (readScratch(rom, sp))
            return rawToC(calcRaw(rom, sp));
    return -127; // DEVICE_DISCONNECTED_C
}

// ROM 8 байт -> 16 hex-символов в стековый буфер out[17]. Без String/кучи —
// только для лога. Один sprintf_P, побайтно (endian не важен).
void LionDallas::romHex(const uint8_t *rom, char *out)
{
    sprintf_P(out, PSTR("%02X%02X%02X%02X%02X%02X%02X%02X"),
              rom[0], rom[1], rom[2], rom[3], rom[4], rom[5], rom[6], rom[7]);
}
