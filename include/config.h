#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Display Pins (GDI FPC connector - occupied by TFT display)
// ============================================================================
#define TFT_DC 3   // GPIO3 - Data/Command selection
#define TFT_CS 18  // GPIO18 - Display chip select
#define TFT_RST 38 // GPIO38 - Display reset
#define LCD_BL 21  // GPIO21 - Backlight control
// SPI Bus (shared): GPIO15 (MOSI), GPIO16 (MISO), GPIO17 (SCLK)

// ============================================================================
// Sensor Pins
// ============================================================================
// ACS758 Current Sensor Pins
constexpr int PIN_ACS1 = A1; // GPIO5 - Sensor 1 current sensor
constexpr int PIN_ACS2 = A2; // GPIO6 - Sensor 2 current sensor

// MAX31865 RTD interface (shares SPI bus with TFT)
constexpr int PIN_MAX31865_CS = A4; // GPIO10 - MAX31865 chip select
// MAX31865 Wiring: SDI→GPIO15, SDO→GPIO16, CLK→GPIO17 (shared SPI bus)

// ============================================================================
// Available GPIO Pins (unused, available for future expansion)
// ============================================================================
// GPIO4  (A0)  - ADC capable, general purpose
// GPIO8  (A3)  - ADC capable, general purpose
// GPIO11 (A5)  - ADC capable, general purpose
// GPIO35       - ADC capable, general purpose
// GPIO36       - ADC capable, general purpose
// GPIO37       - ADC capable, general purpose
// GPIO39       - ADC capable, general purpose
// GPIO40       - ADC capable, general purpose
// GPIO41       - ADC capable, general purpose
// GPIO42       - ADC capable, general purpose
// GPIO45       - General purpose (no ADC)
// GPIO48       - General purpose (no ADC)
//
// Note: GPIO0, GPIO19/20 (USB), GPIO43/44 (UART), GPIO46, GPIO47
//       should be avoided due to boot/USB/special functions

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
