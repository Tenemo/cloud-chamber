/**
 * @file Logger.h
 * @brief Generic display manager for TFT screen with KV display and live log
 * area
 *
 * USAGE:
 * ------
 * 1. Initialize the display (also initializes Serial):
 *    logger.initializeDisplay();
 *
 * 2. Register display lines (do this once during setup):
 *    - For numeric values with units:
 *      logger.registerLine("temp", "Temp:", "C", 25.0f);
 *      logger.registerLine("current", "Current:", "A", 0.0f);
 *
 *    - For text values:
 *      logger.registerTextLine("status", "Status:", "INIT");
 *
 * 3. Update values during runtime (only redraws if value changed):
 *    logger.updateLine("temp", 26.5f);
 *    logger.updateLineText("status", "RUNNING");
 *
 * 4. Output logs (appears in bottom area and Serial):
 *    logger.log("Calibration complete");
 *    logger.log("Sensor initialized");
 *
 * AUTOMATIC WRAPPING:
 * -------------------
 * - If a label is too long (>60 pixels), the value wraps to the next line
 * - If a value is too long to fit in remaining space, it wraps to the next line
 * - Extra slots are automatically reserved only when wrapping is needed
 *
 * LIVE LOG AREA:
 * --------------
 * - Bottom of screen reserved for live logs (configurable via LOG_AREA_LINES in
 * config.h)
 * - Separated from KV display with 1px white line
 * - Logs scroll up automatically when full
 * - All logs also output to Serial
 *
 * FEATURES:
 * ---------
 * - Values are cached and only redrawn when they change
 * - Display updates are throttled (DISPLAY_INTERVAL_MS)
 * - No application-specific logic - purely generic KV display + logging
 * - Supports both numeric (with units) and text values
 * - Integrated Serial output - NEVER use Serial.print/println directly, always
 * use logger.log()
 *
 * EXAMPLE:
 * --------
 *   logger.initializeDisplay();
 *   logger.log("System starting...");
 *   logger.registerTextLine("status", "Status:", "Initializing...");
 *   logger.registerLine("voltage", "Voltage:", "V", 12.0f);
 *
 *   // Later in loop:
 *   logger.updateLine("voltage", measured_voltage);
 *   logger.updateLineText("status", "Running");
 *   logger.log("Measurement updated");
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "DFRobot_GDL.h"
#include "config.h"
#include <Arduino.h>

// Display layout
constexpr int LINE_HEIGHT = 12;
constexpr int VALUE_X = 60;
constexpr int VALUE_WIDTH = 70;

// Display physical properties
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 160;
constexpr int CHAR_WIDTH = 6; // pixels per character
constexpr int MAX_CHARS_PER_LINE = SCREEN_WIDTH / CHAR_WIDTH; // 21 characters

// Serial initialization timeout
constexpr unsigned long SERIAL_TIMEOUT_MS = 1000;

// Fixed buffer sizes (avoid heap fragmentation from String)
constexpr size_t MAX_DISPLAY_LINES = 16;  // Maximum display lines
constexpr size_t MAX_LINE_NAME_LEN = 12;  // e.g., "TC_I1", "DC12_P"
constexpr size_t MAX_LINE_LABEL_LEN = 12; // e.g., "DC12 V:", "State:"
constexpr size_t MAX_LINE_VALUE_LEN = 16; // e.g., "12.34 V", "INIT..."
constexpr size_t MAX_LINE_UNIT_LEN = 6;   // e.g., "K/m", "A"

struct DisplayLine {
    char name[MAX_LINE_NAME_LEN];        // Line identifier
    char label[MAX_LINE_LABEL_LEN];      // display label (e.g., "Temp:")
    char value[MAX_LINE_VALUE_LEN];      // current display value
    char prev_value[MAX_LINE_VALUE_LEN]; // previous value for comparison
    char unit[MAX_LINE_UNIT_LEN];        // unit suffix (e.g., "A", "C")
    int slot;                            // Y position slot on screen
    bool uses_wrap;    // true if value wraps to next line due to overflow
    bool needs_redraw; // true if value changed and needs to be redrawn
    bool active;       // true if this slot is in use
};

struct DisplayLayout {
    int line_height;
    int value_x;
    int value_width;
    int next_slot;
};

class Logger {
  public:
    Logger();

    // Core display management
    void initializeDisplay();
    void update(); // update spinner and other periodic display elements

    // Utility for formatting display labels (adds colon suffix)
    static void formatLabel(char *buf, size_t size, const char *label);

    // Generic KV store interface for display lines (const char* versions)
    void registerLine(const char *name, const char *label,
                      const char *unit = "", float initial_value = 0.0f);
    void registerTextLine(const char *name, const char *label,
                          const char *initial_text = "");

    void updateLine(const char *name, float value);
    void updateLineText(const char *name, const char *text);

    // Live log output (to screen and serial, or serial only)
    void log(const char *message, bool serialOnly = false);

  private:
    void clearValueArea(int y);
    void drawLineLabel(const char *label, int slot);
    void drawLineValue(DisplayLine &line, bool force_full_redraw = false);
    void drawChangedCharacters(const char *old_val, const char *new_val, int x,
                               int y);
    void printLine(const char *text, int x, int y, uint8_t textSize = 1);
    void fillBox(int x, int y, int w, int h, uint16_t color);
    void registerLineInternal(const char *name, const char *label,
                              const char *value, const char *unit);
    bool shouldWrap(const char *label, const char *value) const;
    DisplayLine *findLine(const char *name);
    DisplayLine *findFreeSlot();

    DFRobot_ST7735_128x160_HW_SPI *_screen;
    int8_t _backlight;
    bool _display_initialized;
    unsigned long _last_display_update;

    DisplayLayout _layout;
    DisplayLine _lines[MAX_DISPLAY_LINES]; // Fixed array instead of std::map

    // Spinner state
    int _spinner_index;
    unsigned long _last_spinner_update;

    // Live log area
    char _log_lines[LOG_AREA_LINES][MAX_CHARS_PER_LINE + 1];
    int _log_count;
    int _log_area_y_start;

    void drawLogArea();
    void drawSeparatorLine();
    void updateSpinner();
};

#endif // LOGGER_H
