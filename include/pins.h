/**
 * @file pins.h
 * @brief Hardware pin definitions and sensor addresses
 *
 * This file contains all GPIO pin mappings and hardware-specific addresses.
 * Separate from config.h to keep hardware mapping distinct from tunable
 * behavior.
 */

#ifndef PINS_H
#define PINS_H

#include <cstdint>

// =============================================================================
// Display Pins (directly mapped to GPIO)
// =============================================================================
#define TFT_DC 3   // GPIO3 - Data/Command selection
#define TFT_CS 18  // GPIO18 - Display chip select
#define TFT_RST 38 // GPIO38 - Display reset
#define LCD_BL 21  // GPIO21 - Backlight control

// =============================================================================
// Sensor Pins
// =============================================================================
constexpr int PIN_MAX31865_CS = 10; // GPIO10 - MAX31865 chip select
constexpr int PIN_DS18B20 = 8;      // GPIO8 - DS18B20 OneWire data

// =============================================================================
// DPS5015 Power Supplies (Modbus RTU over UART)
// =============================================================================
constexpr int PIN_DPS5015_1_RX = 44; // GPIO44 - Serial1 RX
constexpr int PIN_DPS5015_1_TX = 43; // GPIO43 - Serial1 TX
constexpr int PIN_DPS5015_2_RX = 5;  // GPIO5 - Serial2 RX
constexpr int PIN_DPS5015_2_TX = 4;  // GPIO4 - Serial2 TX

// =============================================================================
// DS18B20 Sensor Addresses
// =============================================================================
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

#endif // PINS_H
