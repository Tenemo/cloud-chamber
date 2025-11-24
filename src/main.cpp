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

#include "Display.h"
#include "Logger.h"
#include "TECController.h"
#include "config.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

// === GLOBAL INSTANCES ===
Display display(TFT_DC, TFT_CS, TFT_RST, LCD_BL);
Logger logger;
TECController tec_controller(TEC_CONFIG, logger);

unsigned long last_control_update = 0;

void showCalibrationUI(bool success, float offset1, float offset2) {
    display.clear();
    display.printLine("Calibrating sensors...", 0, 20, 1);
    display.printLine("Please wait", 0, 40, 1);

    if (!success) {
        display.printLine("Calibration FAILED!", 0, 60, 1);
        while (true) {
            delay(1000);
        }
    }

    char buf1[32], buf2[32];
    snprintf(buf1, sizeof(buf1), "ACS1: %.3fV", offset1);
    snprintf(buf2, sizeof(buf2), "ACS2: %.3fV", offset2);
    display.printLine(buf1, 0, 60, 1);
    display.printLine(buf2, 0, 75, 1);
    display.printLine("Calibration complete", 0, 95, 1);

    delay(2000);
}

void setup() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_task_wdt_init(60, true);
    Serial.begin(115200);
    logger.logInitialization();

    // Initialize display
    display.begin();
    display.clear();
    display.printLine("TEC Controller", 0, 0, 1);
    display.printLine("Initializing...", 0, 15, 1);

    // Configure logger with display
    logger.setDisplay(&display, LINE_HEIGHT, VALUE_X, VALUE_WIDTH, Y_STATUS,
                      Y_TEC1, Y_TEC2, Y_TOTAL, Y_DUTY, Y_TARGET);

    // Status LED
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    delay(1000);

    // Initialize TEC controller hardware
    tec_controller.begin();

    // Calibrate sensors
    bool cal_success = tec_controller.calibrateSensors();
    float offset1, offset2;
    tec_controller.getCalibrationOffsets(offset1, offset2);
    showCalibrationUI(cal_success, offset1, offset2);

    // Start power detection
    digitalWrite(STATUS_LED_PIN, HIGH);
    tec_controller.startPowerDetection();

    last_control_update = millis();
}

void loop() {
    unsigned long current_time = millis();

    if (current_time - last_control_update < CONTROL_INTERVAL_MS) {
        return;
    }
    last_control_update = current_time;

    // Main control loop - all logic encapsulated in TECController
    tec_controller.update(CONTROL_INTERVAL_MS, DISPLAY_INTERVAL_MS);
}
