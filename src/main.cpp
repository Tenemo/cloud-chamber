/**
 * @file main.cpp
 * @brief Cloud Chamber Monitoring System - Current and Temperature Sensing
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
#include "Logger.h"
#include "PT100.h"
#include "config.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

Logger logger;
CurrentSensing current_sensors(logger);
PT100Sensor temp_sensor(logger);

static void initLogging() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_task_wdt_init(60, true);
}

void setup() {
    initLogging();

    logger.initializeDisplay();
    current_sensors.begin();
    temp_sensor.begin();
}

void loop() {
    logger.update();
    current_sensors.update();
    temp_sensor.update();
}
