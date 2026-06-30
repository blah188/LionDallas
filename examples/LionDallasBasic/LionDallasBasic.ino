// LionDallasBasic — minimal LionDallas usage on an ESP8266.
//
// Wiring: DS18B20 data line -> GPIO4 (D2 on a Wemos D1 mini), with a 4.7k
// pull-up between the data line and 3V3. Multiple DS18B20 may share the bus.
//
// LionDallas leans on two other Lion* libraries:
//   * LionLogger — you MUST define ONE global named `Logger` (it is declared
//     `extern` inside <Logger.h>); LionDallas logs discovery/diagnostics there.
//   * LionTask   — LionDallas is a LionCriticalTask, so you MUST pump
//     LionTask::Loop() from loop() for its periodic reads to run.

#include <Arduino.h>
#include <Logger.h>      // LionLogger
#include <LionTask.h>    // LionTask scheduler (LionCriticalTask)
#include <LionDallas.h>

// The one required global. Serial-only here; see LionLogger for filesystem
// targets and levels.
MyLogger Logger(ILogger::SerialPort, ILogger::LvlDebug);

static const uint8_t ONE_WIRE_PIN   = 4;     // GPIO4 / D2
static const int     READ_INTERVAL  = 2000;  // ms; must be >= conversion time

LionDallas *dallas = nullptr;

void setup()
{
    Serial.begin(115200);
    delay(200);
    Logger.Setup();
    LionTask::Setup();

    // Auto-discover sensors on the bus (search-once + stored addresses).
    // To skip SearchROM on a long/noisy bus, pass known addresses instead:
    //   static const DeviceAddress addrs[] = {
    //       {0x28,0x..,0x..,0x..,0x..,0x..,0x..,0x..},
    //   };
    //   dallas = new LionDallas(ONE_WIRE_PIN, READ_INTERVAL, addrs, 1);
    dallas = new LionDallas(ONE_WIRE_PIN, READ_INTERVAL);

    Serial.printf("LionDallas: %d sensor(s), parasite=%d\n",
                  dallas->GetSenorCount(), dallas->IsParasite());
}

void loop()
{
    LionTask::Loop();   // pumps the LionCriticalTask inside LionDallas

    static uint32_t last = 0;
    if (millis() - last >= (uint32_t)READ_INTERVAL)
    {
        last = millis();
        int n = dallas->GetSenorCount();
        for (int i = 0; i < n; i++)
        {
            char rom[17];
            dallas->GetAddressHex(i, rom);
            Serial.printf("  [%d] %s = %.2f C\n", i, rom, dallas->GetTemperature(i));
        }
    }
}
