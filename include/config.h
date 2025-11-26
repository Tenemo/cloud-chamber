#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// FireBeetle 2 ESP32-S3 Pin Definitions (Board Labels → GPIO)
// ============================================================================
// Digital Pins
#define D2 3   // GPIO3  - LCD_DC (used by display)
#define D3 38  // GPIO38 - LCD_RST (used by display)
#define D5 7   // GPIO7  - FCS (font chip select)
#define D6 18  // GPIO18 - LCD_CS (used by display)
#define D7 9   // GPIO9  - SD_CS (SD card)
#define D9 0   // GPIO0  - Boot key (avoid)
#define D10 14 // GPIO14 - BUSY (tear sync)
#define D11 13 // GPIO13 - INT (touch interrupt)
#define D12 12 // GPIO12 - TCS (touch chip select)
#define D13 21 // GPIO21 - LCD_BL (backlight)
#define D14 47 // GPIO47 - User key

// Analog Pins
#define A0 4  // GPIO4
#define A1 5  // GPIO5 - Sensor 1 current sensor
#define A2 6  // GPIO6 - Sensor 2 current sensor
#define A3 8  // GPIO8
#define A4 10 // GPIO10 - MAX31865 (PT100) chip select
#define A5 11 // GPIO11

// Communication Pins
#define SCK 17  // GPIO17 - SPI clock
#define MOSI 15 // GPIO15 - SPI MOSI
#define MISO 16 // GPIO16 - SPI MISO
#define SCL 2   // GPIO2  - I2C clock
#define SDA 1   // GPIO1  - I2C data
#define TX 43   // GPIO43 - UART TX
#define RX 44   // GPIO44 - UART RX

// ============================================================================
// Display Pins (GDI FPC connector - occupied by TFT display)
// ============================================================================
#define TFT_DC D2  // GPIO3 - Data/Command selection
#define TFT_CS D6  // GPIO18 - Display chip select
#define TFT_RST D3 // GPIO38 - Display reset
#define LCD_BL D13 // GPIO21 - Backlight control
// SPI Bus (shared): MOSI (GPIO15), MISO (GPIO16), SCK (GPIO17)

// ============================================================================
// Sensor Pins
// ============================================================================
// ACS758 Current Sensor Pins
constexpr int PIN_ACS1 = A1; // GPIO5 - Sensor 1 current sensor
constexpr int PIN_ACS2 = A2; // GPIO6 - Sensor 2 current sensor

// MAX31865 RTD interface (shares SPI bus with TFT)
constexpr int PIN_MAX31865_CS = A4; // GPIO10 - MAX31865 chip select
// MAX31865 Wiring: SDI→MOSI, SDO→MISO, CLK→SCK (shared SPI bus)

// ============================================================================
// Available GPIO Pins (unused, available for future expansion)
// ============================================================================
// A0 (GPIO4)   - ADC capable, general purpose
// A3 (GPIO8)   - ADC capable, general purpose
// A5 (GPIO11)  - (ADC2) ADC capable, general purpose, reads may fail if Wi‑Fi
// is active GPIO39       - Digital only, general purpose GPIO40       - Digital
// only, general purpose GPIO41       - Digital only, general purpose GPIO42 -
// Digital only, general purpose GPIO48       - Digital only, general purpose
// GPIO43       - TX, digital only, general purpose, available for use
// GPIO44       - RX, digital only, general purpose, available for use
// D5 (GPIO7)   - FCS font chip (if not using font library)
// D7 (GPIO9)   - SD_CS (physically connected to the screen with the GDI cable,
// avoid) D10 (GPIO14) - BUSY (physically connected to the screen with the GDI
// cable, avoid) D11 (GPIO13) - INT (physically connected to the screen with the
// GDI cable, avoid) D12 (GPIO12) - TCS (physically connected to the screen with
// the GDI cable, avoid)
//
// Note: D9 (GPIO0), GPIO19/20 (USB), GPIO46, D14 (GPIO47)
//       should be avoided due to boot/USB/special functions

// ACS758 Current Sensor Configuration
constexpr float ADC_REF_V = 3.3f;
constexpr float ACS_SENS = 0.026f; // 26 mV/A at 3.3V supply

// Current Sensor Filter Settings
constexpr float FILTER_ALPHA = 0.15f; // Low-pass filter coefficient
constexpr int ADC_SAMPLES = 300;      // Number of samples to average

// Current Sensor Warning Settings
constexpr bool ENABLE_IMBALANCE_WARNINGS =
    false; // Enable current imbalance warnings

// Update Intervals
constexpr unsigned long SENSOR_UPDATE_INTERVAL_MS = 200;
constexpr unsigned long DISPLAY_INTERVAL_MS = 500;

// Display Layout
constexpr int LINE_HEIGHT = 12;
constexpr int VALUE_X = 60;
constexpr int VALUE_WIDTH = 70;

// Display Physical Properties
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 160;
constexpr int CHAR_WIDTH = 6; // Pixels per character
constexpr int MAX_CHARS_PER_LINE = SCREEN_WIDTH / CHAR_WIDTH; // 21 characters

// Display Update Timing
constexpr unsigned long SPINNER_UPDATE_MS = 250;
constexpr unsigned long SERIAL_TIMEOUT_MS = 1000;

// Live Log Area (bottom of screen)
constexpr int LOG_AREA_LINES =
    4; // Number of lines reserved for live logs at bottom

#endif
