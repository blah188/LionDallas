#pragma once
//
// LionDallas — компактная DS18B20-библиотека поверх OneWire под Lion*-стек.
// Drop-in замена LocalLibs/DallasTemp: тот же публичный API (pin, период чтения,
// опц. fixedAddrs). Поиск адресов выполняется один раз на старте и хранится
// (лечит «второй датчик отваливается»).
//
// Планировщик выбирается по платформе:
//   * ESP8266 — LionCriticalTask (LionTask), тик через LionTask::Loop();
//   * ESP32   — RtosTask типа EveryMs (LionRtosTask), отдельная FreeRTOS-задача,
//               pump из loop() НЕ нужен.
// Публичный API идентичен на обеих платформах.
//
// Время жизни: создать один раз (глобал/static/new) и держать всю программу.
// На ESP32 объект владеет ещё и FreeRTOS-задачей, поэтому НЕ удалять из чужого
// контекста (гонка с читающей задачей за _ds); дизайн и так one-shot-discovery.
//
// Алгоритмы scratchpad/конверсии/fault-детекта портированы из
// DallasTemperature 4.x (Miles Burton, MIT) — см. LionDallasPort.cpp.
// Здесь (.h) — только объявления; тел портированного кода нет.
//
#include <OneWire.h>
#include <Array.h>
#include <Logger.h>

#ifdef ESP32
#include <RtosTask.h> // RtosTask (LionRtosTask) — ESP8266-аналога LionCriticalTask нет
#define LIOND_TASK_BASE RtosTask
// FreeRTOS-параметры задачи чтения (переопределяемы через build_flags).
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

// Тот же тип/имя, что у DallasTemperature — чтобы
// `static const DeviceAddress villaDallasAddrs[]` в DallasTempGetter.cpp
// остался без изменений.
typedef uint8_t DeviceAddress[8];

class LionDallas : LIOND_TASK_BASE
{
public:
    // Совместимо с DallasTemp: pin, период чтения, опц. известные адреса
    // (заданы → пропустить ненадёжный SearchROM, читать по MatchROM).
    LionDallas(uint8_t pin, int tempInterval,
               const DeviceAddress *fixedAddrs = nullptr, uint8_t fixedCount = 0,
               uint8_t resolution = 12);
    ~LionDallas();

    // Владеет сырым OneWire* (_ds, удаляется в дтор) — копия = double-free.
    // Копия и так неявно запрещена (члены Array некопируемы), но объявляем
    // явно: контракт виден и переживёт рефактор, убравший Array-член.
    LionDallas(const LionDallas &) = delete;
    LionDallas &operator=(const LionDallas &) = delete;

    int   GetSenorCount();
    float GetTemperature(int idx = 0); // кэш; -127 при невалидном/out-of-range
    bool  IsParasite() const { return _parasite; }
    // ROM датчика idx -> hex в стековый буфер out[17] (для статуса/диагностики).
    // Пустая строка, если idx вне диапазона.
    void  GetAddressHex(int idx, char *out);

protected:
    // Платформенный тик-метод базового планировщика; оба зовут pollAndConvert().
#ifdef ESP32
    void TaskBody() override;      // шаг FreeRTOS-задачи EveryMs (LionRtosTask)
#else
    void TaskInnerBody() override; // тик LionCriticalTask (pump: LionTask::Loop())
#endif

private:
    void pollAndConvert(); // прочитать всех (по адресу) -> запустить конверсию
    OneWire *_ds;
    uint8_t  _pin;
    uint8_t  _resolution;
    bool     _parasite = false;
    Array<DeviceAddress> _addr;         // хранимые адреса (найдены один раз)
    Array<float>         _temperatures; // кэш последних валидных значений
    Array<uint32_t>      _times;        // время каждого значения — для TOLERANCE_127_MS

    // --- своё (Lion-стек) ---
    bool   searchAndStore(const DeviceAddress *fixedAddrs, uint8_t fixedCount); // collecting + keep-best / fixedAddrs
    void   startConversion();              // STARTCONVO broadcast (неблокирующий)
    float  readSensor(const uint8_t *rom); // getTemp-обёртка с ретраями -> °C
    // ROM 8 байт -> hex в стековый буфер out[17] (только для лога; без String/кучи).
    static void romHex(const uint8_t *rom, char *out);

    // --- порт из Miles Burton (DallasTemperature 4.x, MIT) — тела в LionDallasPort.cpp ---
    bool     readScratch(const uint8_t *rom, uint8_t *sp);        // reset+select+READSCRATCH +crc8/all-zeros
    void     writeScratch(const uint8_t *rom, const uint8_t *sp); // reset+select+WRITESCRATCH (TH,TL,CFG)
    int32_t  calcRaw(const uint8_t *rom, uint8_t *sp);            // raw + POR/insufficient/fault детект (4.0.6)
    void     applyResolution(const uint8_t *rom, uint8_t res);    // writeScratch config 9..12 бит
    bool     readPowerSupply(const uint8_t *rom);                 // parasite-детект
    static float    rawToC(int32_t raw);                          // raw -> °C, fault-сентинелы
    static uint16_t convMs(uint8_t res);                          // 94/188/375/750 мс
};
