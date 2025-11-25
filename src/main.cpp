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
#include <Arduino.h>
#include <esp_task_wdt.h>

// === GLOBAL INSTANCES ===
Logger logger;
TECController tec_controller(logger);

unsigned long last_control_update = 0;

static void initLogging() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_task_wdt_init(60, true);
    Serial.begin(115200);
}

static bool initHardwareAndCalibrate() {
    logger.initializeDisplay();
    tec_controller.begin();

    bool cal_success = tec_controller.calibrateSensors();
    float offset1, offset2;
    tec_controller.getCalibrationOffsets(offset1, offset2);
    logger.showStartupSequence(cal_success, offset1, offset2);
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
}
