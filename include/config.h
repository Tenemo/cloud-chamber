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

constexpr float TARGET_CURRENT_PER_TEC = 4.00f;
constexpr float MAX_DUTY = 0.60f;
constexpr float MIN_DUTY = 0.0f;

// Retuned PI for faster 50ms loop
constexpr float KP = 0.08f;          // Increased proportional gain
constexpr float KI = 0.004f;         // Increased integral gain
constexpr float INTEGRAL_MAX = 0.2f; // Increased integral limit

constexpr float ADC_REF_V = 3.3f;
constexpr float ACS_SENS = 0.026f; // 26 mV/A at 3.3V supply

// Reduced samples and faster filter for quicker response
constexpr float FILTER_ALPHA = 0.15f; // Faster response
constexpr int ADC_SAMPLES = 300;      // Good accuracy, faster reading

// 10 Hz control loop - slightly slower, still responsive
constexpr unsigned long CONTROL_INTERVAL_MS = 200;
constexpr unsigned long DISPLAY_INTERVAL_MS = 500;
constexpr unsigned long SOFT_START_DURATION_MS = 5000;

constexpr float DETECTION_DUTY = 0.25f;
constexpr float DETECTION_THRESHOLD = 0.2f;

// Overshoot protection
constexpr float PER_TEC_LIMIT_MULTIPLIER = 1.15f;     // 15% over per-TEC target
constexpr float TOTAL_OVERCURRENT_MULTIPLIER = 1.25f; // 25% over total target

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
