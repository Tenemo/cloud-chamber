#ifndef CONFIG_H
#define CONFIG_H

#include "TECController.h"

// === DISPLAY PINS ===
#define TFT_DC 3         // Display DC pin
#define TFT_CS 18        // Display CS pin
#define TFT_RST 38       // Display RST pin
#define LCD_BL 21        // Display backlight pin
#define STATUS_LED_PIN 8 // Status LED pin

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

// === SAFETY ===
constexpr float OVERCURRENT_MULTIPLIER =
    1.25f; // 25% over target triggers shutdown

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

// === TEC CONTROLLER CONFIGURATION ===
const TECController::Config TEC_CONFIG = {
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
    .overcurrent_multiplier = OVERCURRENT_MULTIPLIER};

#endif // CONFIG_H
