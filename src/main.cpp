#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "Logger.h"
#include "PT100.h"
#include "TECController.h"
#include "config.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

// === GLOBAL INSTANCES ===
Logger logger;
TECController tec_controller(logger);
PT100Sensor temp_sensor(logger);

unsigned long last_control_update = 0;

static void initLogging() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_task_wdt_init(60, true);
    Serial.begin(115200);
}

static bool initHardwareAndCalibrate() {
    logger.initializeDisplay();
    tec_controller.begin();

    // Show initialization message
    logger.registerTextLine("init", "Status:", "Initializing...");
    delay(1000);

    bool cal_success = tec_controller.calibrateSensors();
    float offset1, offset2;
    tec_controller.getCalibrationOffsets(offset1, offset2);

    // Show calibration results
    logger.clearDisplay();
    if (cal_success) {
        logger.registerTextLine("cal_status", "Calibration:", "OK");

        char buf1[32], buf2[32];
        snprintf(buf1, sizeof(buf1), "%.4fV", offset1);
        snprintf(buf2, sizeof(buf2), "%.4fV", offset2);
        logger.registerTextLine("acs1_offset", "ACS1:", buf1);
        logger.registerTextLine("acs2_offset", "ACS2:", buf2);
    } else {
        logger.registerTextLine("cal_status", "Calibration:", "FAIL");
    }
    delay(2000);

    // Clear and set up main display
    logger.clearDisplay();
    logger.registerTextLine("status", "Status:", "INIT");
    logger.registerLine("tec1", "TEC1:", "A", 0.0f);
    logger.registerLine("tec2", "TEC2:", "A", 0.0f);
    logger.registerLine("total", "Total:", "A", 0.0f);

    if (PWM_ENABLED) {
        logger.registerLine("duty", "Duty:", "%", 0.0f);
        logger.registerLine("target", "Target:", "A",
                            TARGET_CURRENT_PER_TEC * 2.0f);
    }

    return cal_success;
}

static void waitForeverOnCalibrationFailure() {
    while (true) {
        delay(100);
    }
}

void setup() {
    initLogging();

    if (!initHardwareAndCalibrate()) {
        waitForeverOnCalibrationFailure();
    }

    // Initialize temperature sensor
    temp_sensor.begin();

    tec_controller.startPowerDetection();
    last_control_update = millis();
}

void loop() {
    unsigned long current_time = millis();

    if (current_time - last_control_update < CONTROL_INTERVAL_MS) {
        return;
    }
    last_control_update = current_time;

    tec_controller.update();
    temp_sensor.update();
}
