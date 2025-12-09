#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// FireBeetle 2 ESP32-S3 pin definitions (board labels → GPIO)
// ============================================================================
// Digital pins
#define D2 3   // GPIO3  - LCD_DC (used by display)
#define D3 38  // GPIO38 - LCD_RST (used by display)
#define D5 7   // GPIO7  - FCS (font chip select)
#define D6 18  // GPIO18 - LCD_CS (used by display)
#define D7 9   // GPIO9  - SD_CS (SD card)
#define D9 0   // GPIO0  - Boot key (avoid)
#define D10 14 // GPIO14 - BUSY (tear sync)
#define D11 13 // GPIO13 - INT (unused - no touch on DFR0928)
#define D12 12 // GPIO12 - TCS (unused - no touch on DFR0928)
#define D13 21 // GPIO21 - LCD_BL (backlight)
#define D14 47 // GPIO47 - User key

// Analog pins
#define A0 4  // GPIO4
#define A1 5  // GPIO5
#define A2 6  // GPIO6
#define A3 8  // GPIO8 - DS18B20 OneWire data
#define A4 10 // GPIO10 - MAX31865 (PT100) chip select
#define A5 11 // GPIO11

// Communication pins
#define SCK 17  // GPIO17 - SPI clock
#define MOSI 15 // GPIO15 - SPI MOSI
#define MISO 16 // GPIO16 - SPI MISO
#define SCL 2   // GPIO2  - I2C clock
#define SDA 1   // GPIO1  - I2C data
#define TX 43   // GPIO43 - UART TX
#define RX 44   // GPIO44 - UART RX

// ============================================================================
// Display pins (GDI FPC connector - occupied by TFT display)
// ============================================================================
#define TFT_DC D2  // GPIO3 - Data/Command selection
#define TFT_CS D6  // GPIO18 - Display chip select
#define TFT_RST D3 // GPIO38 - Display reset
#define LCD_BL D13 // GPIO21 - Backlight control
// SPI Bus (shared): MOSI (GPIO15), MISO (GPIO16), SCK (GPIO17)

// ============================================================================
// Sensor pins
// ============================================================================
// MAX31865 RTD interface (shares SPI bus with TFT)
constexpr int PIN_MAX31865_CS = A4; // GPIO10 - MAX31865 chip select
// MAX31865 Wiring: SDI→MOSI, SDO→MISO, CLK→SCK (shared SPI bus)

// DS18B20 digital temperature sensor (OneWire)
constexpr int PIN_DS18B20 =
    A3; // GPIO8 - DS18B20 data pin (3.3k pull-up to 3.3V)

// DPS5015 power supply 1 (Modbus RTU over UART - Serial1)
constexpr int PIN_DPS5015_1_RX = RX; // GPIO44 - UART RX (connect to DPS TX)
constexpr int PIN_DPS5015_1_TX = TX; // GPIO43 - UART TX (connect to DPS RX)

// DPS5015 power supply 2 (Modbus RTU over UART - Serial2)
constexpr int PIN_DPS5015_2_RX = A1; // GPIO5 - UART RX (connect to DPS TX)
constexpr int PIN_DPS5015_2_TX = A0; // GPIO4 - UART TX (connect to DPS RX)

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
constexpr unsigned long DPS5015_UPDATE_INTERVAL_MS = 500;

// PT100 serial logging (serial only, not display)
constexpr bool PT100_SERIAL_LOGGING_ENABLED = true;
constexpr unsigned long PT100_SERIAL_LOG_INTERVAL_MS = 10000;
constexpr unsigned long DISPLAY_INTERVAL_MS =
    100; // Reduced from 200ms - char-by-char drawing minimizes pixel writes

// Display layout
constexpr int LINE_HEIGHT = 12;
constexpr int VALUE_X = 60;
constexpr int VALUE_WIDTH = 70;

// Display physical properties
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 160;
constexpr int CHAR_WIDTH = 6; // pixels per character
constexpr int MAX_CHARS_PER_LINE = SCREEN_WIDTH / CHAR_WIDTH; // 21 characters

// Display update timing
constexpr unsigned long SPINNER_UPDATE_MS = 250;
constexpr unsigned long SERIAL_TIMEOUT_MS = 1000;

// Live log area (bottom of screen)
constexpr int LOG_AREA_LINES =
    2; // number of lines reserved for live logs at bottom

#endif
