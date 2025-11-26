/**
 * @file main.cpp
 * @brief Cloud chamber monitoring system - current and temperature sensing
 *
 * IMPORTANT: NEVER use Serial.print/println directly in this application.
 * Always use logger.log() for all output - it handles both display and Serial.
 *
 * Architecture:
 * - Logger: Manages TFT display (KV store + live log area) and Serial output
 * - CurrentSensing: Dual ACS758 current sensor monitoring (Sensor1, Sensor2)
 * - PT100Sensor: Temperature monitoring via MAX31865 RTD interface
 *
 * All modules self-manage their update timing and display registration.
 */

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "CurrentSensing.h"
#include "DS18B20.h"
#include "Logger.h"
#include "PT100.h"
#include "config.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

Logger logger;
CurrentSensing current_sensors(logger);
PT100Sensor temp_sensor(logger);
DS18B20Sensor ds18b20_sensor(logger);

static void configureSystemLogging() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_task_wdt_init(60, true);
}

static void scanOneWireDevices() {
    OneWire ow(PIN_DS18B20);
    uint8_t addr[8];

    logger.log("Scanning 1-Wire...");

    ow.reset_search();
    int count = 0;
    while (ow.search(addr)) {
        count++;
        char buf[32];
        snprintf(buf, sizeof(buf), "%d: %02X%02X%02X%02X%02X%02X%02X%02X",
                 count, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
                 addr[6], addr[7]);
        logger.log(buf);

        // Check if it's a DS18B20 (family code 0x28)
        if (addr[0] == 0x28) {
            logger.log("  -> DS18B20");
        } else if (addr[0] == 0x10) {
            logger.log("  -> DS18S20");
        }
    }

    char countBuf[20];
    snprintf(countBuf, sizeof(countBuf), "Found %d device(s)", count);
    logger.log(countBuf);
}

static void initializeHardware() {
    logger.initializeDisplay();
    current_sensors.begin();
    temp_sensor.begin();

    // Debug: Scan for OneWire devices before initializing DS18B20
    scanOneWireDevices();

    ds18b20_sensor.begin();
}

void setup() {
    configureSystemLogging();
    initializeHardware();
}

void loop() {
    logger.update();
    current_sensors.update();
    temp_sensor.update();
    ds18b20_sensor.update();
}
