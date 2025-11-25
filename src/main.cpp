/*
 * PROJECT: Portable Muon Cloud Chamber Controller
 * PLATFORM: ESP32-S3 (FireBeetle 2)
 *
 * PURPOSE:
 * This firmware controls a portable thermoelectric cloud chamber designed to
 * visualize cosmic ray muons. The system creates a supersaturated alcohol vapor
 * layer by maintaining a high-precision temperature gradient:
 * - Cold Plate (Bottom): Cooled to -30°C using cascaded Peltier modules
 * (TEC2-19006).
 * - Hot End (Top): Heated/Cooled by a 420mm Liquid AIO cooler and auxiliary
 * heaters.
 *
 * POWER & CONTROL ARCHITECTURE (Synchronous Buck Topology):
 * The system drives TECs using a BTS7960 H-Bridge in a custom "Buck Converter"
 * configuration to provide smooth DC power rather than raw PWM (which degrades
 * TEC efficiency):
 * 1. Source: 24V DC Power Supply.
 * 2. Driver: BTS7960 H-Bridge. Both L_EN and R_EN are held HIGH to enable the
 * bridge.
 * 3. Switching:
 * - Pin RPWM receives the Control PWM signal (25kHz).
 * - Pin LPWM is directly grounded.
 * - Because both enables are HIGH, the bridge performs "Active Freewheeling"
 * (Synchronous Rectification) during the PWM off-state, minimizing losses.
 * 4. Filtering: The chopped 24V output passes through a high-current LC filter
 * (Inductor + Capacitor). This integrates the PWM into a stable DC voltage
 * (V_out ~= V_in * DutyCycle).
 * 5. Feedback Loop:
 * - ACS758 Hall-effect sensors measure the actual DC current flowing to the
 * TECs.
 * - A PI (Proportional-Integral) loop reads the current and adjusts the PWM
 * duty cycle to maintain a constant target amperage (Constant Current Mode).
 */

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "Logger.h"
#include "TECController.h"
#include "config.h"
#include <Adafruit_MAX31865.h>
#include <Arduino.h>
#include <SPI.h>
#include <esp_task_wdt.h>

// === GLOBAL INSTANCES ===
Logger logger;
TECController tec_controller(logger);
Adafruit_MAX31865 rtd(PIN_MAX31865_CS);

// MAX31865 Configuration
#define RNOMINAL 100.0f // PT100
#define RREF 438.0f     // Measured reference resistor

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

    // Initialize MAX31865 RTD interface (3-wire PT100)
    rtd.begin(MAX31865_3WIRE);

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

    // Read and log plate temperature
    float temp_c = rtd.temperature(RNOMINAL, RREF);

    // Register temperature line if not already registered
    if (!logger.hasLine("temp")) {
        logger.registerLine("temp", "Temp:", "C", temp_c);
    } else {
        logger.updateLine("temp", temp_c);
    }
}
