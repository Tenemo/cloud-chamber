#ifndef CONFIG_H
#define CONFIG_H

#include <cstddef> // for size_t
#include <cstdint> // for uint8_t, etc.

// Display pins (directly mapped to GPIO)
// ============================================================================
#define TFT_DC 3   // GPIO3 - Data/Command selection
#define TFT_CS 18  // GPIO18 - Display chip select
#define TFT_RST 38 // GPIO38 - Display reset
#define LCD_BL 21  // GPIO21 - Backlight control

// ============================================================================
// Sensor pins
// ============================================================================
constexpr int PIN_MAX31865_CS = 10; // GPIO10 - MAX31865 chip select
constexpr int PIN_DS18B20 = 8;      // GPIO8 - DS18B20 OneWire data

// DPS5015 power supplies (Modbus RTU over UART)
constexpr int PIN_DPS5015_1_RX = 44; // GPIO44 - Serial1 RX
constexpr int PIN_DPS5015_1_TX = 43; // GPIO43 - Serial1 TX
constexpr int PIN_DPS5015_2_RX = 5;  // GPIO5 - Serial2 RX
constexpr int PIN_DPS5015_2_TX = 4;  // GPIO4 - Serial2 TX

// DS18B20 sensor addresses
constexpr uint8_t DS18B20_1_ADDRESS[8] = {0x28, 0xFF, 0x64, 0x1F,
                                          0x75, 0xA8, 0xDD, 0x0C};
constexpr uint8_t DS18B20_2_ADDRESS[8] = {0x28, 0xFF, 0x64, 0x1F,
                                          0x75, 0xB8, 0x5F, 0xD0};
constexpr uint8_t DS18B20_3_ADDRESS[8] = {0x28, 0xFF, 0x64, 0x1F,
                                          0x75, 0xB7, 0x33, 0x0E};

/** ============================================================================
 * Available, unused GPIO pins
 * ============================================================================
 * - A2 (GPIO6)   - ADC capable, general purpose
 * - A5 (GPIO11)  - ADC capable, general purpose
 * - D5 (GPIO7)   - FCS font chip (directly usable if not using font library)
 * - D11 (GPIO13) - INT pin (unused, no touch on DFR0928)
 * - D12 (GPIO12) - TCS pin (unused, no touch on DFR0928)
 */

/** ============================================================================
 * Available GPIO pins underside the board, require soldering
 * ============================================================================
 * - GPIO39       - Digital only, directly usable
 * - GPIO40       - Digital only, directly usable
 * - GPIO41       - Digital only, directly usable
 * - GPIO42       - Digital only, directly usable
 */

/** ============================================================================
 * Unavailable/used GPIO pins
 * ============================================================================
 * In use:
 * - A3 (GPIO8)   - DS18B20 OneWire data
 * - A4 (GPIO10)  - MAX31865 chip select (PT100)
 * - SCK (GPIO17) - SPI clock (shared bus between PT100 and the display)
 * - MOSI (GPIO15)- SPI MOSI (shared bus between PT100 and the display)
 * - MISO (GPIO16)- SPI MISO (shared bus between PT100 and the display)
 *
 * Display (directly connected via GDI FPC - DFR0928 non-touch):
 * - D2 (GPIO3)   - LCD_DC
 * - D3 (GPIO38)  - LCD_RST
 * - D6 (GPIO18)  - LCD_CS
 * - D13 (GPIO21) - LCD_BL (backlight)
 * - D7 (GPIO9)   - SD_CS (directly connected to screen)
 * - D10 (GPIO14) - BUSY (tear sync, directly connected)
 *
 * Unused GDI pins (no touch controller on DFR0928):
 * - D11 (GPIO13) - INT (available for other use)
 * - D12 (GPIO12) - TCS (available for other use)
 *
 * DPS5015 Power Supplies (Modbus RTU):
 * - TX (GPIO43)  - UART TX (DPS5015 #1 Modbus - Serial1)
 * - RX (GPIO44)  - UART RX (DPS5015 #1 Modbus - Serial1)
 * - A0 (GPIO4)   - UART RX (DPS5015 #2 Modbus - Serial2)
 * - A1 (GPIO5)   - UART TX (DPS5015 #2 Modbus - Serial2)
 *
 * System/Special (avoid):
 * - D9 (GPIO0)   - Boot button, strapping pin
 * - D14 (GPIO47) - User key
 * - GPIO19       - USB D-
 * - GPIO20       - USB D+
 * - GPIO46       - Strapping pin
 * - SCL (GPIO2)  - I2C clock (usable if I2C not needed)
 * - SDA (GPIO1)  - I2C data (usable if I2C not needed)
 */

// Update intervals
constexpr unsigned long PT100_UPDATE_INTERVAL_MS = 500;
constexpr unsigned long DS18B20_UPDATE_INTERVAL_MS =
    800; // 12-bit resolution needs 750ms conversion
constexpr unsigned long DS18B20_CONVERSION_TIME_MS =
    750; // 12-bit resolution conversion time
constexpr unsigned long DPS5015_UPDATE_INTERVAL_MS = 500;
constexpr int MODBUS_MAX_RETRIES = 3; // Consecutive failures before error state

// Modbus communication settings
// Default baud rate is 9600. If experiencing communication errors at high
// TEC power due to EMI from the DC lines, try:
// - Lowering baud rate to 4800 or 2400 (more robust but slower updates)
// - Adding ferrite cores to UART lines
// - Using shielded cables for Modbus connections
// - Increasing physical separation from high-current DC wiring
constexpr unsigned long MODBUS_BAUD_RATE = 9600;
constexpr unsigned long MODBUS_BYTE_TIMEOUT_MS = 10; // Per-byte timeout

// PT100 serial logging (serial only, not display)
constexpr bool PT100_SERIAL_LOGGING_ENABLED = true;
constexpr unsigned long PT100_SERIAL_LOG_INTERVAL_MS = 10000;
constexpr unsigned long DISPLAY_INTERVAL_MS = 100;

// Display update timing
constexpr unsigned long SPINNER_UPDATE_MS = 100;

// Watchdog configuration
constexpr unsigned long WDT_TIMEOUT_SECONDS = 10; // Task WDT timeout

// Live log area (bottom of screen)
constexpr int LOG_AREA_LINES =
    2; // number of lines reserved for live logs at bottom

// ============================================================================
// Sensor Validation Configuration
// ============================================================================

// PT100 sensor error detection thresholds
constexpr float PT100_ERROR_MIN_C = -100.0f; // Below this = sensor error
constexpr float PT100_ERROR_MAX_C = 500.0f;  // Above this = sensor error

#endif
