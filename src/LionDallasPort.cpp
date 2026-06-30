//
// LionDallasPort.cpp — методы DS18B20, ПОРТИРОВАННЫЕ КАК ЕСТЬ из
// DallasTemperature 4.x (scratchpad / конверсия / разрешение / fault-детект).
// Главная ценность — детект POR (0x0550) и insufficient-power из 4.0.6.
//
// ---------------------------------------------------------------------------
// Portions ported from the Arduino Temperature Control Library (DallasTemperature).
// https://github.com/milesburton/Arduino-Temperature-Control-Library
//
// The MIT License (MIT)
//
// Copyright (c) 2024 Miles Burton
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Modifications for LionDallas: сведено к семейству DS18B20 (0x28); выброшены
// alarms / EEPROM save-recall / userData / мульти-семейства; методы превращены
// в члены класса LionDallas, использующие OneWire* _ds. © правки — Lion-стек.
// ---------------------------------------------------------------------------
//
#include "LionDallas.h"

// OneWire commands
#define STARTCONVO      0x44 // запустить конверсию температуры
#define READSCRATCH     0xBE // прочитать scratchpad
#define WRITESCRATCH    0x4E // записать scratchpad
#define READPOWERSUPPLY 0xB4 // определить, нужен ли parasite-power

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2 // у DS18B20 — байт TH (используем под config write)
#define LOW_ALARM_TEMP  3 // у DS18B20 — байт TL
#define CONFIGURATION   4
#define COUNT_REMAIN    6
#define SCRATCHPAD_CRC  8

// Device resolution (config register)
#define TEMP_9_BIT  0x1F
#define TEMP_10_BIT 0x3F
#define TEMP_11_BIT 0x5F
#define TEMP_12_BIT 0x7F

#define DS18B20MODEL 0x28
#define DSROM_FAMILY 0

// Error / sentinel raw codes (как в DallasTemperature 4.0.6)
#define DEVICE_DISCONNECTED_C      -127
#define DEVICE_DISCONNECTED_RAW    -7040
#define DEVICE_FAULT_OPEN_C        -254
#define DEVICE_FAULT_OPEN_RAW      -32512
#define DEVICE_FAULT_SHORTGND_C    -253
#define DEVICE_FAULT_SHORTGND_RAW  -32384
#define DEVICE_FAULT_SHORTVDD_C    -252
#define DEVICE_FAULT_SHORTVDD_RAW  -32256
#define DEVICE_POWER_ON_RESET_C    -251
#define DEVICE_POWER_ON_RESET_RAW  -32128
#define DEVICE_INSUFFICIENT_POWER_C   -250
#define DEVICE_INSUFFICIENT_POWER_RAW -32000

// --- readScratchPad + isAllZeros + crc8 (исходно readScratchPad/isConnected) ---
// reset+select+READSCRATCH, 9 байт; валидно если шина ответила, scratchpad не
// весь нулевой и crc8 сошёлся. Возвращает bool.
bool LionDallas::readScratch(const uint8_t *rom, uint8_t *sp)
{
    if (_ds->reset() == 0)
        return false;

    _ds->select(rom);
    _ds->write(READSCRATCH);

    for (uint8_t i = 0; i < 9; i++)
        sp[i] = _ds->read();

    if (_ds->reset() != 1)
        return false;

    // isAllZeros
    bool allZeros = true;
    for (uint8_t i = 0; i < 9; i++)
        if (sp[i] != 0) { allZeros = false; break; }
    if (allZeros)
        return false;

    return OneWire::crc8(sp, 8) == sp[SCRATCHPAD_CRC];
}

// --- writeScratchPad (DS18B20: TH, TL, CONFIG) ---
// Без EEPROM-сохранения (COPYSCRATCH): разрешение переустанавливаем на каждом
// старте, переживать перезагрузку датчика не требуется.
void LionDallas::writeScratch(const uint8_t *rom, const uint8_t *sp)
{
    _ds->reset();
    _ds->select(rom);
    _ds->write(WRITESCRATCH);
    _ds->write(sp[HIGH_ALARM_TEMP]); // TH
    _ds->write(sp[LOW_ALARM_TEMP]);  // TL
    _ds->write(sp[CONFIGURATION]);   // config (разрешение)
    _ds->reset();
}

// --- calculateTemperature (4.0.6) — raw + детект POR/insufficient/fault ---
int32_t LionDallas::calcRaw(const uint8_t *rom, uint8_t *sp)
{
    int32_t fpTemperature = 0;

    // бит 15 — знаковый у всех поддерживаемых датчиков
    int32_t neg = 0x0;
    if (sp[TEMP_MSB] & 0x80)
        neg = 0xFFF80000;

    fpTemperature = (((int16_t)sp[TEMP_MSB]) << 11)
                   | (((int16_t)sp[TEMP_LSB]) << 3)
                   | neg;

    // detect POR and insufficient power conditions (DS18B20)
    if (rom[DSROM_FAMILY] == DS18B20MODEL)
    {
        if (sp[TEMP_LSB] == 0x50 && sp[TEMP_MSB] == 0x05 && sp[COUNT_REMAIN] == 0x0C)
            return DEVICE_POWER_ON_RESET_RAW;     // заводской 0x0550 — конверсий не было
        else if (sp[TEMP_LSB] == 0xFF && sp[TEMP_MSB] == 0x07)
            return DEVICE_INSUFFICIENT_POWER_RAW; // не хватило питания на конверсию
    }

    return fpTemperature;
}

// --- rawToCelsius (4.0.6) — raw -> °C, fault-сентинелы наружу как C-значения ---
float LionDallas::rawToC(int32_t raw)
{
    switch (raw)
    {
    case DEVICE_DISCONNECTED_RAW:    return DEVICE_DISCONNECTED_C;
    case DEVICE_FAULT_OPEN_RAW:      return DEVICE_FAULT_OPEN_C;
    case DEVICE_FAULT_SHORTGND_RAW:  return DEVICE_FAULT_SHORTGND_C;
    case DEVICE_FAULT_SHORTVDD_RAW:  return DEVICE_FAULT_SHORTVDD_C;
    case DEVICE_POWER_ON_RESET_RAW:  return DEVICE_POWER_ON_RESET_C;
    case DEVICE_INSUFFICIENT_POWER_RAW: return DEVICE_INSUFFICIENT_POWER_C;
    default:
        return (float)raw * 0.0078125f; // 1/128
    }
}

// --- millisToWaitForConversion ---
uint16_t LionDallas::convMs(uint8_t res)
{
    switch (res)
    {
    case 9:  return 94;
    case 10: return 188;
    case 11: return 375;
    default: return 750;
    }
}

// --- setResolution(addr, res) — записать config-регистр 9..12 бит ---
void LionDallas::applyResolution(const uint8_t *rom, uint8_t res)
{
    if (res < 9)  res = 9;
    if (res > 12) res = 12;

    uint8_t sp[9];
    if (!readScratch(rom, sp))
        return;

    uint8_t newValue;
    switch (res)
    {
    case 12: newValue = TEMP_12_BIT; break;
    case 11: newValue = TEMP_11_BIT; break;
    case 10: newValue = TEMP_10_BIT; break;
    case 9:
    default: newValue = TEMP_9_BIT; break;
    }

    if (sp[CONFIGURATION] != newValue)
    {
        sp[CONFIGURATION] = newValue;
        writeScratch(rom, sp);
    }
}

// --- readPowerSupply — true, если датчик в parasite-режиме ---
bool LionDallas::readPowerSupply(const uint8_t *rom)
{
    bool parasiteMode = false;
    _ds->reset();
    if (rom == nullptr)
        _ds->skip();
    else
        _ds->select(rom);

    _ds->write(READPOWERSUPPLY);
    if (_ds->read_bit() == 0)
        parasiteMode = true;
    _ds->reset();
    return parasiteMode;
}
