#ifndef CONFIG_H
#define CONFIG_H

#define TFT_DC 3
#define TFT_CS 18
#define TFT_RST 38
#define LCD_BL 21
#define STATUS_LED_PIN 8

// ACS758 Current Sensor Pins
constexpr int PIN_ACS1 = A1; // GPIO5 - Sensor 1 current sensor
constexpr int PIN_ACS2 = A2; // GPIO6 - Sensor 2 current sensor

// MAX31865 RTD interface (shares SPI bus with TFT)
constexpr int PIN_MAX31865_CS = A4; // GPIO10, used as MAX31865 chip select

// ACS758 Current Sensor Configuration
constexpr float ADC_REF_V = 3.3f;
constexpr float ACS_SENS = 0.026f; // 26 mV/A at 3.3V supply

// Current Sensor Filter Settings
constexpr float FILTER_ALPHA = 0.15f; // Low-pass filter coefficient
constexpr int ADC_SAMPLES = 300;      // Number of samples to average

// Update Intervals
constexpr unsigned long SENSOR_UPDATE_INTERVAL_MS = 200;
constexpr unsigned long DISPLAY_INTERVAL_MS = 500;

// Display Layout
constexpr int LINE_HEIGHT = 12;
constexpr int VALUE_X = 60;
constexpr int VALUE_WIDTH = 70;

// Live Log Area (bottom of screen)
constexpr int LOG_AREA_LINES =
    2; // Number of lines reserved for live logs at bottom

#endif
