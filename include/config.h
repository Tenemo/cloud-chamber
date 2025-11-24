#ifndef CONFIG_H
#define CONFIG_H

#define TFT_DC 3
#define TFT_CS 18
#define TFT_RST 38
#define LCD_BL 21
#define STATUS_LED_PIN 8

constexpr int PIN_ACS1 = A1; // GPIO5
constexpr int PIN_ACS2 = A2; // GPIO6
constexpr int PIN_RPWM = A0; // GPIO4
constexpr int PIN_L_EN = A4; // GPIO10
constexpr int PIN_R_EN = D9; // GPIO0

constexpr float TARGET_CURRENT_PER_TEC = 3.00f;
constexpr float MAX_DUTY = 0.65f;
constexpr float MIN_DUTY = 0.0f;
constexpr float KP = 0.02f;
constexpr float KI = 0.001f;
constexpr float INTEGRAL_MAX = 0.2f;

constexpr float ADC_REF_V = 3.3f;
constexpr float ACS_SENS = 0.026f; // 26 mV/A at 3.3V supply
constexpr float FILTER_ALPHA = 0.2f;
constexpr int ADC_SAMPLES = 150;

constexpr unsigned long CONTROL_INTERVAL_MS = 100;
constexpr unsigned long DISPLAY_INTERVAL_MS = 250;
constexpr unsigned long SOFT_START_DURATION_MS = 5000;

constexpr float DETECTION_DUTY = 0.25f;
constexpr float DETECTION_THRESHOLD = 0.2f;

constexpr float OVERCURRENT_MULTIPLIER = 1.25f;

constexpr int LINE_HEIGHT = 12;
constexpr int VALUE_X = 60;
constexpr int VALUE_WIDTH = 70;
constexpr int Y_STATUS = 0;
constexpr int Y_TEC1 = LINE_HEIGHT * 2;
constexpr int Y_TEC2 = LINE_HEIGHT * 3;
constexpr int Y_TOTAL = LINE_HEIGHT * 4;
constexpr int Y_DUTY = LINE_HEIGHT * 5;
constexpr int Y_TARGET = LINE_HEIGHT * 6;

#endif
