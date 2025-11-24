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

// === HARDWARE CONFIGURATION ===
constexpr int PIN_ACS1 = A1; // GPIO5  - CJMCU #1 (TEC1 branch)
constexpr int PIN_ACS2 = A2; // GPIO6  - CJMCU #2 (TEC2 branch)
constexpr int PIN_RPWM = A0; // GPIO4  - BTS7960 RPWM
constexpr int PIN_L_EN = A4; // GPIO10 - BTS7960 L_EN
constexpr int PIN_R_EN = D9; // GPIO0  - BTS7960 R_EN

// === CONTROL PARAMETERS ===
constexpr float TARGET_CURRENT_PER_TEC = 3.00f; // Amperes per TEC
constexpr float MAX_DUTY = 0.65f;               // 65% duty limit
constexpr float MIN_DUTY = 0.0f;
constexpr float KP = 0.02f;          // Proportional gain
constexpr float KI = 0.001f;         // Integral gain
constexpr float INTEGRAL_MAX = 0.2f; // Anti-windup limit

// === SENSOR PARAMETERS ===
constexpr float ADC_REF_V = 3.3f;
constexpr float ACS_SENS = 0.026f;   // 26 mV/A at 3.3V supply
constexpr float FILTER_ALPHA = 0.2f; // Exponential filter coefficient
constexpr int ADC_SAMPLES = 150;

// === TIMING ===
constexpr unsigned long CONTROL_INTERVAL_MS = 100;     // 10Hz control loop
constexpr unsigned long DISPLAY_INTERVAL_MS = 250;     // 4Hz display update
constexpr unsigned long SOFT_START_DURATION_MS = 5000; // 5 seconds ramp-up

// === POWER DETECTION ===
constexpr float DETECTION_DUTY = 0.25f;
constexpr float DETECTION_THRESHOLD = 0.2f; // 200mA minimum

// === DISPLAY LAYOUT ===
constexpr int LINE_HEIGHT = 12;
constexpr int VALUE_X = 60;
constexpr int VALUE_WIDTH = 70;
constexpr int Y_STATUS = 0;
constexpr int Y_TEC1 = LINE_HEIGHT * 2;
constexpr int Y_TEC2 = LINE_HEIGHT * 3;
constexpr int Y_TOTAL = LINE_HEIGHT * 4;
constexpr int Y_DUTY = LINE_HEIGHT * 5;
constexpr int Y_TARGET = LINE_HEIGHT * 6;

// === GLOBAL INSTANCES ===
Display display(TFT_DC, TFT_CS, TFT_RST, LCD_BL);
Logger logger;

// TEC Controller configuration
TECController::Config tec_config = {
    .pin_acs1 = PIN_ACS1,
    .pin_acs2 = PIN_ACS2,
    .pin_rpwm = PIN_RPWM,
    .pin_l_en = PIN_L_EN,
    .pin_r_en = PIN_R_EN,
    .adc_ref_v = ADC_REF_V,
    .acs_sensitivity = ACS_SENS,
    .filter_alpha = FILTER_ALPHA,
    .adc_samples = ADC_SAMPLES,
    .target_current_per_tec = TARGET_CURRENT_PER_TEC,
    .max_duty = MAX_DUTY,
    .min_duty = MIN_DUTY,
    .kp = KP,
    .ki = KI,
    .integral_max = INTEGRAL_MAX,
    .detection_duty = DETECTION_DUTY,
    .detection_threshold = DETECTION_THRESHOLD,
    .soft_start_duration_ms = SOFT_START_DURATION_MS,
    .overcurrent_multiplier = 1.25f // 25% over target triggers shutdown
};

TECController tec_controller(tec_config, logger);

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
