/**
 * @file Logger.h
 * @brief Generic display manager for TFT screen that acts as a key-value store
 *
 * USAGE:
 * ------
 * 1. Initialize the display:
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
 * AUTOMATIC WRAPPING:
 * -------------------
 * - If a label is too long (>60 pixels), the value wraps to the next line
 * - If a value is too long to fit in remaining space, it wraps to the next line
 * - Extra slots are automatically reserved only when wrapping is needed
 *
 * FEATURES:
 * ---------
 * - Values are cached and only redrawn when they change
 * - Display updates are throttled (DISPLAY_INTERVAL_MS)
 * - No application-specific logic - purely generic KV display
 * - Supports both numeric (with units) and text values
 *
 * EXAMPLE:
 * --------
 *   logger.initializeDisplay();
 *   logger.registerTextLine("status", "Status:", "Initializing...");
 *   logger.registerLine("voltage", "Voltage:", "V", 12.0f);
 *
 *   // Later in loop:
 *   logger.updateLine("voltage", measured_voltage);
 *   logger.updateLineText("status", "Running");
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "DFRobot_GDL.h"
#include <Arduino.h>
#include <map>

struct DisplayLine {
    String label;      // Display label (e.g., "Temp:", "TEC1:")
    String unit;       // Unit suffix (e.g., "°C", "A", "%")
    String text_value; // For text-based values
    float num_value;   // Current cached numeric value
    int slot;          // Y position slot on screen
    bool is_text;      // True if displaying text, false if displaying number
    bool initialized;  // Whether this line has been drawn on screen
    bool uses_wrap;    // True if value wraps to next line due to overflow
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

    // Generic KV store interface for display lines
    void registerLine(const String &name, const String &label,
                      const String &unit = "", float initial_value = 0.0f);
    void registerTextLine(const String &name, const String &label,
                          const String &initial_text = "");

    void updateLine(const String &name, float value);
    void updateLineText(const String &name, const String &text);

    // Check if a line exists
    bool hasLine(const String &name) const;

  private:
    void clearValueArea(int y);
    void drawLineLabel(const String &label, int slot);
    void drawLineValue(const DisplayLine &line);
    void printLine(const char *text, int x, int y, uint8_t textSize = 1);
    void fillBox(int x, int y, int w, int h, uint16_t color);

    DFRobot_ST7735_128x160_HW_SPI *_screen;
    int8_t _backlight;
    bool display_initialized;
    unsigned long last_display_update;

    DisplayLayout _layout;
    std::map<String, DisplayLine> _lines;
};

#endif // LOGGER_H
