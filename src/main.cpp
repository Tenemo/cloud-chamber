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

#include "DPS5015.h"
#include "DS18B20.h"
#include "Logger.h"
#include "PT100.h"
#include "config.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

Logger logger;

// Shared OneWire bus for all DS18B20 sensors
OneWire oneWire(PIN_DS18B20);
DallasTemperature dallasSensors(&oneWire);

PT100Sensor pt100_sensor(logger, "PT100");
DS18B20Sensor ds18b20_sensors[] = {
    {logger, dallasSensors, DS18B20_1_ADDRESS, "TEMP_1"},
    {logger, dallasSensors, DS18B20_2_ADDRESS, "TEMP_2"},
    {logger, dallasSensors, DS18B20_3_ADDRESS, "TEMP_3"}};

// DPS5015 power supplies (Serial1 and Serial2)
DPS5015 psus[] = {{logger, "DC12", Serial1}, {logger, "DC34", Serial2}};

static void initializeHardware() {
    esp_task_wdt_init(60, true);
    logger.initializeDisplay();

    dallasSensors.begin();
    dallasSensors.setWaitForConversion(false);
    for (auto &sensor : ds18b20_sensors)
        sensor.begin();

    pt100_sensor.begin();

    psus[0].begin(PIN_DPS5015_1_RX, PIN_DPS5015_1_TX);
    psus[1].begin(PIN_DPS5015_2_RX, PIN_DPS5015_2_TX);

    // Configure PSUs - settings are applied automatically when they connect
    for (auto &psu : psus)
        psu.configure(12.0f, 2.0f, true);
}

void setup() { initializeHardware(); }

void loop() {
    logger.update();
    pt100_sensor.update();
    for (auto &sensor : ds18b20_sensors)
        sensor.update();
    for (auto &psu : psus)
        psu.update();
}
