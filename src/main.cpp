#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "CurrentSensing.h"
#include "Logger.h"
#include "PT100.h"
#include "config.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

// === GLOBAL INSTANCES ===
Logger logger;
CurrentSensing current_sensors(logger);
PT100Sensor temp_sensor(logger);

static void initLogging() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_task_wdt_init(60, true);
    Serial.begin(115200);
}

void setup() {
    initLogging();

    logger.initializeDisplay();
    current_sensors.begin();
    temp_sensor.begin();

    Serial.println("=== Cloud Chamber Monitoring System Active ===\n");
}

void loop() {
    current_sensors.update();
    temp_sensor.update();
}
