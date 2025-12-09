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
DPS5015 psu1(logger, "PSU1", Serial1);
DPS5015 psu2(logger, "PSU2", Serial2);

static void configurePSU(DPS5015 &psu, float voltage, float current) {
    if (!psu.isConnected())
        return;
    psu.unlock();
    psu.setVoltage(voltage);
    psu.setCurrent(current);
    psu.setOutput(true);
}

static void initializeHardware() {
    esp_task_wdt_init(60, true);
    logger.initializeDisplay();

    dallasSensors.begin();
    dallasSensors.setWaitForConversion(false);
    for (auto &sensor : ds18b20_sensors)
        sensor.begin();

    pt100_sensor.begin();

    psu1.begin(PIN_DPS5015_1_RX, PIN_DPS5015_1_TX);
    psu2.begin(PIN_DPS5015_2_RX, PIN_DPS5015_2_TX);
    configurePSU(psu1, 12.0f, 2.0f);
    configurePSU(psu2, 12.0f, 2.0f);
}

void setup() { initializeHardware(); }

void loop() {
    logger.update();
    for (auto &sensor : ds18b20_sensors)
        sensor.update();
    pt100_sensor.update();
    psu1.update();
    psu2.update();
}
