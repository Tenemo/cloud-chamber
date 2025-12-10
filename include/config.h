#ifndef CONFIG_H
#define CONFIG_H

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
// Thermal Controller Configuration
// ============================================================================

// Voltage and current limits
constexpr float TEC_VOLTAGE_SETPOINT = 16.0f; // Fixed voltage for cascade TECs
constexpr float MAX_CURRENT_PER_CHANNEL = 10.6f; // Maximum current per DPS
constexpr float MIN_CURRENT_PER_CHANNEL = 0.5f;  // Minimum operational current
constexpr float STARTUP_CURRENT = 2.0f; // Initial current during startup
constexpr float DEGRADED_MODE_CURRENT =
    5.0f; // Max current when one DPS disconnects
constexpr float DPS_OCP_LIMIT = 11.0f; // Hardware Over Current Protection
                                       // Failsafe in case ESP commands >10.6A

// Timing intervals (milliseconds)
constexpr unsigned long STARTUP_HOLD_DURATION_MS = 60000; // 60s startup hold
constexpr unsigned long RAMP_ADJUSTMENT_INTERVAL_MS =
    20000; // 20s between ramp steps
constexpr unsigned long CURRENT_EVALUATION_DELAY_MS =
    60000; // 60s to evaluate current change
constexpr unsigned long OPTIMIZATION_EVALUATION_WINDOW_MS =
    90000; // 90s evaluation window
constexpr unsigned long OPTIMIZATION_REFINEMENT_INTERVAL_MS =
    60000; // 60s refinement steps
constexpr unsigned long STEADY_STATE_RECHECK_INTERVAL_MS =
    900000; // 15min recheck
constexpr unsigned long SENSOR_RECOVERY_TIMEOUT_MS =
    60000;                                       // 60s sensor recovery
constexpr unsigned long INIT_TIMEOUT_MS = 30000; // 30s init timeout

// Temperature thresholds (Celsius)
constexpr float HOT_SIDE_WARNING_C = 55.0f; // Reduce ramp aggressiveness
constexpr float HOT_SIDE_WARNING_EXIT_C =
    50.0f;                                // Exit warning state (hysteresis)
constexpr float HOT_SIDE_ALARM_C = 65.0f; // Stop increasing current
constexpr float HOT_SIDE_ALARM_EXIT_C = 60.0f; // Exit alarm state (hysteresis)
constexpr float HOT_SIDE_FAULT_C = 70.0f;      // Emergency shutdown
constexpr float HOT_SIDE_RATE_FAULT_C_PER_MIN =
    5.0f; // Thermal runaway detection
constexpr float COOLING_STALL_THRESHOLD_C =
    0.5f; // Consider stalled if less than this
constexpr float OVERCURRENT_WARMING_THRESHOLD_C =
    0.3f; // Back off if cold plate warms this much
constexpr float COOLING_RATE_DEGRADATION_THRESHOLD =
    0.3f; // K/min - detect when cooling rate slows significantly

// Control parameters
constexpr float RAMP_CURRENT_STEP_A = 0.5f; // Current step during ramp
constexpr float MANUAL_OVERRIDE_VOLTAGE_TOLERANCE_V =
    0.15f; // Tolerance for override detection
constexpr float MANUAL_OVERRIDE_CURRENT_TOLERANCE_A =
    0.15f; // Tolerance for override detection
constexpr unsigned long MANUAL_OVERRIDE_GRACE_MS =
    3000; // Settling window: time for DPS to process command and for ESP
          // to read back new value before checking for override
constexpr float EMERGENCY_RAMP_DOWN_RATE_A_PER_SEC =
    2.0f; // Emergency shutdown ramp
constexpr unsigned long EMERGENCY_SHUTDOWN_STEP_MS =
    500; // Time between ramp-down steps (non-blocking)

// History buffer
constexpr size_t HISTORY_BUFFER_SIZE = 300;                // 5 minutes at 1Hz
constexpr unsigned long HISTORY_SAMPLE_INTERVAL_MS = 1000; // 1 second samples
constexpr size_t COOLING_RATE_WINDOW_SAMPLES =
    30; // 30s window for rate calculation

#endif
