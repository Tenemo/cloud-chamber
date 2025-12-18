/**
 * @file main.cpp
 * @brief Cloud chamber monitoring system - temperature sensing and power
 * control
 *
 * IMPORTANT: NEVER use Serial.print/println directly in this application.
 * Always use logger.log() for all output - it handles both display and Serial.
 *
 * Architecture:
 * - Logger: Manages TFT display (KV store + live log area) and Serial output
 * - PT100Sensor: Temperature monitoring via MAX31865 RTD interface (cold plate)
 * - DS18B20Sensor: Digital temperature sensors via OneWire (hot plate, ambient)
 * - DualPowerSupply: Symmetric control of two DPS5015 units via Modbus RTU
 * - ThermalController: Smart control system coordinating all components
 * persistence
 *   - SafetyMonitor: Centralized safety checks with consistent error handling
 *
 * All modules self-manage their update timing and display registration.
 *
 * SAFETY:
 * - Task Watchdog Timer (WDT) enabled - resets if loop() takes >10 seconds
 *
 * SHUTDOWN:
 * - Normal shutdown is via physical OFF switch (power cut)
 * - DPS5015 units always start in OFF state after power cycle
 * - No software-initiated clean shutdown needed
 * - Fault events are logged to Serial/display for visibility
 */

#include "Logger.h"
#include "TemperatureSensors.h"
#include "TimeService.h"
#include "config.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

#include "ThermalController.h"

Logger logger;

// Temperature sensors (owns PT100 and DS18B20)
TemperatureSensors sensors(logger);

// Thermal controller - coordinates sensors and PSUs (owns DualPowerSupply
// internally)
ThermalController thermalController(logger, sensors);

static constexpr float BENCHMARK_CURRENTS_A[] = {2.0f, 4.0f, 6.0f, 7.0f,
                                                 8.0f, 9.0f, 10.0f, 11.0f,
                                                 12.0f, 13.0f, 14.0f};
static constexpr size_t BENCHMARK_CURRENTS_COUNT =
    sizeof(BENCHMARK_CURRENTS_A) / sizeof(BENCHMARK_CURRENTS_A[0]);

static void initializeWatchdog() {
    // Configure Task Watchdog Timer
    // This will reset the ESP32 if loop() hangs for longer than timeout
    // Using older API compatible with Arduino ESP32 core
    esp_task_wdt_init(Watchdog::TIMEOUT_SECONDS,
                      true); // timeout, panic on trigger
    esp_task_wdt_add(NULL);  // Add current task (loopTask) to watchdog
}

static void initializeHardware() {
    logger.initializeDisplay();
    logger.log("=== BOOT ===");

    // Optional one-shot wall-clock sync via home WiFi + NTP
    TimeService::trySyncFromWifi(
        [](const char *msg, bool serialOnly) { logger.log(msg, serialOnly); });

    initializeWatchdog();
    sensors.begin();

    thermalController.beginBenchmark(BENCHMARK_CURRENTS_A,
                                     BENCHMARK_CURRENTS_COUNT,
                                     20UL * 60UL * 1000UL,
                                     20.0f);
}

void setup() { initializeHardware(); }

void loop() {
    esp_task_wdt_reset();
    logger.update();
    sensors.update();

    thermalController.update();
}
