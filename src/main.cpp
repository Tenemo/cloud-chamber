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
 * - DPS5015: Programmable power supply control via Modbus RTU
 * - ThermalController: Smart control system coordinating all components
 *
 * All modules self-manage their update timing and display registration.
 *
 * SAFETY:
 * - Task Watchdog Timer (WDT) enabled - resets if loop() takes >10 seconds
 * - Clean shutdown flag stored in NVS - detects crash recovery on boot
 */

#include "DPS5015.h"
#include "DS18B20.h"
#include "Logger.h"
#include "PT100.h"
#include "ThermalController.h"
#include "config.h"
#include <Arduino.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

// NVS storage for crash detection
Preferences preferences;
static constexpr const char *NVS_NAMESPACE = "cloud_chamber";
static constexpr const char *NVS_KEY_CLEAN_SHUTDOWN = "clean_shutdown";

Logger logger;

// Shared OneWire bus for all DS18B20 sensors
OneWire oneWire(PIN_DS18B20);
DallasTemperature dallasSensors(&oneWire);

// Cold plate sensor (PT100 via MAX31865)
PT100Sensor pt100_sensor(logger, "PT100");

// Hot plate and ambient sensors (DS18B20)
// TEMP_1 = hot plate, TEMP_2/TEMP_3 = ambient
DS18B20Sensor ds18b20_sensors[] = {
    {logger, dallasSensors, DS18B20_1_ADDRESS, "HOT"},
    {logger, dallasSensors, DS18B20_2_ADDRESS, "AMB_1"},
    {logger, dallasSensors, DS18B20_3_ADDRESS, "AMB_2"}};

// DPS5015 power supplies (Serial1 and Serial2)
DPS5015 psus[] = {{logger, "DC12", Serial1}, {logger, "DC34", Serial2}};

// Thermal controller - coordinates sensors and PSUs
ThermalController
    thermalController(logger,
                      pt100_sensor,       // Cold plate (PT100)
                      ds18b20_sensors[0], // Hot plate (first DS18B20)
                      &ds18b20_sensors[1],
                      2,   // Ambient sensors (remaining DS18B20s)
                      psus // PSU array
    );

static bool checkCleanShutdown() {
    preferences.begin(NVS_NAMESPACE, true); // Read-only
    bool was_clean = preferences.getBool(NVS_KEY_CLEAN_SHUTDOWN, true);
    preferences.end();
    return was_clean;
}

static void markShutdownState(bool clean) {
    preferences.begin(NVS_NAMESPACE, false); // Read-write
    preferences.putBool(NVS_KEY_CLEAN_SHUTDOWN, clean);
    preferences.end();
}

static void initializeWatchdog() {
    // Configure Task Watchdog Timer
    // This will reset the ESP32 if loop() hangs for longer than timeout
    // Using older API compatible with Arduino ESP32 core
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true); // timeout, panic on trigger
    esp_task_wdt_add(NULL); // Add current task (loopTask) to watchdog
}

static void initializeHardware() {
    logger.initializeDisplay();

    // Check for crash recovery
    if (!checkCleanShutdown()) {
        logger.log("WARN: Crash recovery!");
        // ThermalController will detect DPS running and shut down safely
    }

    // Mark as "not clean" until we properly shut down
    // (If we crash, next boot will see this)
    markShutdownState(false);

    // Initialize watchdog
    initializeWatchdog();

    // Initialize DS18B20 bus and sensors
    dallasSensors.begin();
    dallasSensors.setWaitForConversion(false);
    for (auto &sensor : ds18b20_sensors)
        sensor.begin();

    // Initialize PT100
    pt100_sensor.begin();

    // Initialize PSUs (ThermalController will configure them)
    psus[0].begin(PIN_DPS5015_1_RX, PIN_DPS5015_1_TX);
    psus[1].begin(PIN_DPS5015_2_RX, PIN_DPS5015_2_TX);

    // Initialize thermal controller
    thermalController.begin();
}

void setup() { initializeHardware(); }

void loop() {
    // Feed the watchdog - must be called regularly
    esp_task_wdt_reset();

    // Update all hardware drivers
    logger.update();
    pt100_sensor.update();
    for (auto &sensor : ds18b20_sensors)
        sensor.update();
    for (auto &psu : psus)
        psu.update();

    // Run thermal control logic
    thermalController.update();
}
