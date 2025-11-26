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
#include <Arduino.h>
#include <map>

struct DisplayLine {
    String label; // Display label (e.g., "Temp:", "Sensor1:")
    String value; // Current display value (formatted string)
    String
        unit; // Unit suffix (e.g., "A", "C", "%") - stored for numeric updates
    int slot; // Y position slot on screen
    bool initialized; // Whether this line has been drawn on screen
    bool uses_wrap;   // True if value wraps to next line due to overflow
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
    void clearDisplay();
    void update(); // Update spinner and other periodic display elements

    // Generic KV store interface for display lines
    void registerLine(const String &name, const String &label,
                      const String &unit = "", float initial_value = 0.0f);
    void registerTextLine(const String &name, const String &label,
                          const String &initial_text = "");

    void updateLine(const String &name, float value);
    void updateLineText(const String &name, const String &text);

    // Check if a line exists
    bool hasLine(const String &name) const;

    // Live log output (to screen and serial)
    void log(const String &message);

  private:
    void clearValueArea(int y);
    void drawLineLabel(const String &label, int slot);
    void drawLineValue(const DisplayLine &line);
    void printLine(const char *text, int x, int y, uint8_t textSize = 1);
    void fillBox(int x, int y, int w, int h, uint16_t color);
    void registerLineInternal(const String &name, const String &label,
                              const String &value, const String &unit);

    DFRobot_ST7735_128x160_HW_SPI *_screen;
    int8_t _backlight;
    bool _display_initialized;
    unsigned long _last_display_update;

    DisplayLayout _layout;
    std::map<String, DisplayLine> _lines;

    // Spinner state
    int _spinner_index;
    unsigned long _last_spinner_update;

    // Live log area
    String _log_lines[10]; // Support up to 10 log lines (configurable via
                           // LOG_AREA_LINES)
    int _log_count;
    int _log_area_y_start;

    void drawLogArea();
    void drawSeparatorLine();
    void updateSpinner();
};

#endif // LOGGER_H
