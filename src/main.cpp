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
 * - PT100Sensor: Temperature monitoring via MAX31865 RTD interface
 * - DS18B20Sensor: Digital temperature sensors via OneWire
 * - DPS5015: Programmable power supply control via Modbus RTU
 *
 * All modules self-manage their update timing and display registration.
 */

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "DPS5015.h"
#include "DS18B20.h"
#include "Logger.h"
#include "PT100.h"
#include "config.h"
#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <esp_task_wdt.h>

Logger logger;

// Shared OneWire bus for all DS18B20 sensors
OneWire oneWire(PIN_DS18B20);
DallasTemperature dallasSensors(&oneWire);

PT100Sensor pt100_sensor(logger, "PT100");
DS18B20Sensor ds18b20_sensor_1(logger, dallasSensors, DS18B20_1_ADDRESS,
                               "TEMP_1");
DS18B20Sensor ds18b20_sensor_2(logger, dallasSensors, DS18B20_2_ADDRESS,
                               "TEMP_2");
DS18B20Sensor ds18b20_sensor_3(logger, dallasSensors, DS18B20_3_ADDRESS,
                               "TEMP_3");

// DPS5015 power supply (uses Serial1 on TX/RX pins)
DPS5015 psu(logger, "PSU");

static void configureSystemLogging() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_task_wdt_init(60, true);
}

static void initializeHardware() {
    logger.initializeDisplay();
    dallasSensors.begin();
    dallasSensors.setWaitForConversion(false);
    ds18b20_sensor_1.begin();
    ds18b20_sensor_2.begin();
    ds18b20_sensor_3.begin();
    pt100_sensor.begin();
    psu.begin(PIN_DPS5015_RX,
              PIN_DPS5015_TX); // RX pin for receive, TX pin for transmit
}

void setup() {
    configureSystemLogging();
    initializeHardware();
}

void loop() {
    logger.update();
    ds18b20_sensor_1.update();
    ds18b20_sensor_2.update();
    ds18b20_sensor_3.update();
    pt100_sensor.update();
    psu.update();
}
