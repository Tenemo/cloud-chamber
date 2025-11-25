#include "Logger.h"
#include "config.h"

Logger::Logger()
    : _screen(nullptr), _backlight(-1), display_initialized(false),
      last_display_update(0), _layout{0} {}

void Logger::initializeDisplay() {
    _backlight = LCD_BL;
    _layout = {LINE_HEIGHT, VALUE_X, VALUE_WIDTH, 0};
    _screen = new DFRobot_ST7735_128x160_HW_SPI(TFT_DC, TFT_CS, TFT_RST);

    pinMode(_backlight, OUTPUT);
    digitalWrite(_backlight, HIGH);
    _screen->begin();
    _screen->setRotation(0);
    _screen->fillScreen(COLOR_RGB565_BLACK);
    _screen->setTextColor(COLOR_RGB565_WHITE);
    _screen->setTextSize(1);
    _screen->setTextWrap(false); // Disable automatic text wrapping
    _screen->setCursor(0, 0);

    display_initialized = true;
}

void Logger::clearDisplay() {
    if (!_screen)
        return;
    _screen->fillScreen(COLOR_RGB565_BLACK);
    _lines.clear();
    _layout.next_slot = 0;
}

void Logger::printLine(const char *text, int x, int y, uint8_t textSize) {
    if (!_screen)
        return;
    _screen->setTextSize(textSize);
    _screen->setTextColor(COLOR_RGB565_WHITE);
    _screen->setCursor(x, y);
    _screen->println(text);
}

void Logger::fillBox(int x, int y, int w, int h, uint16_t color) {
    if (!_screen)
        return;
    _screen->fillRect(x, y, w, h, color);
}

void Logger::clearValueArea(int y) {
    if (_screen) {
        fillBox(_layout.value_x, y, _layout.value_width, _layout.line_height,
                0x0000);
    }
}

void Logger::drawLineLabel(const String &label, int slot) {
    if (!_screen)
        return;
    int y = slot * _layout.line_height;

    // Always print the full label
    // If it's long, the value will wrap to the next line automatically
    _screen->setTextSize(1);
    _screen->setTextColor(COLOR_RGB565_WHITE);
    _screen->setCursor(0, y);
    _screen->print(label); // Use print() instead of println() to avoid newline
}

void Logger::drawLineValue(const DisplayLine &line) {
    if (!_screen)
        return;

    int y = line.slot * _layout.line_height;

    char buf[32];
    if (line.is_text) {
        snprintf(buf, sizeof(buf), "%s", line.text_value.c_str());
    } else {
        if (line.unit.length() > 0) {
            snprintf(buf, sizeof(buf), "%.2f %s", line.num_value,
                     line.unit.c_str());
        } else {
            snprintf(buf, sizeof(buf), "%.2f", line.num_value);
        }
    }

    // Check if we need to wrap (either label too long or value too long)
    int label_width = line.label.length() * 6;
    int text_width = strlen(buf) * 6;
    int available_width = 128 - _layout.value_x;
    bool should_wrap =
        (label_width > _layout.value_x) || (text_width > available_width);

    if (should_wrap) {
        // Wrap to next line below label
        clearValueArea(y); // Clear the original value area
        fillBox(0, y + _layout.line_height, 128, _layout.line_height, 0x0000);
        printLine(buf, 0, y + _layout.line_height, 1);
    } else {
        // Normal display on same line
        clearValueArea(y);
        if (line.uses_wrap) {
            // Clear wrapped line if we previously used it
            fillBox(0, y + _layout.line_height, 128, _layout.line_height,
                    0x0000);
        }
        printLine(buf, _layout.value_x, y, 1);
    }
}

void Logger::registerLine(const String &name, const String &label,
                          const String &unit, float initial_value) {
    if (!display_initialized)
        return;

    if (_lines.find(name) != _lines.end()) {
        // Line already exists, just update it
        updateLine(name, initial_value);
        return;
    }

    // Check if label is too long or if value will overflow
    int label_width = label.length() * 6;
    char buf[32];
    if (unit.length() > 0) {
        snprintf(buf, sizeof(buf), "%.2f %s", initial_value, unit.c_str());
    } else {
        snprintf(buf, sizeof(buf), "%.2f", initial_value);
    }
    int text_width = strlen(buf) * 6;
    int available_width = 128 - _layout.value_x;
    bool will_wrap =
        (label_width > _layout.value_x) || (text_width > available_width);

    DisplayLine line;
    line.label = label;
    line.unit = unit;
    line.num_value = initial_value;
    line.text_value = "";
    line.is_text = false;
    line.slot = _layout.next_slot++;
    line.initialized = false;
    line.uses_wrap = will_wrap;

    // Only reserve extra slot if text actually wraps
    if (will_wrap) {
        _layout.next_slot++;
    }

    _lines[name] = line;

    // Draw label immediately
    drawLineLabel(label, line.slot);
    // Draw initial value
    drawLineValue(line);
    _lines[name].initialized = true;
}

void Logger::registerTextLine(const String &name, const String &label,
                              const String &initial_text) {
    if (!display_initialized)
        return;

    if (_lines.find(name) != _lines.end()) {
        // Line already exists, just update it
        updateLineText(name, initial_text);
        return;
    }

    // Check if label is too long or if value will overflow
    int label_width = label.length() * 6;
    int text_width = initial_text.length() * 6;
    int available_width = 128 - _layout.value_x;
    bool will_wrap =
        (label_width > _layout.value_x) || (text_width > available_width);

    DisplayLine line;
    line.label = label;
    line.unit = "";
    line.num_value = 0.0f;
    line.text_value = initial_text;
    line.is_text = true;
    line.slot = _layout.next_slot++;
    line.initialized = false;
    line.uses_wrap = will_wrap;

    // Only reserve extra slot if text actually wraps
    if (will_wrap) {
        _layout.next_slot++;
    }

    _lines[name] = line;

    // Draw label immediately
    drawLineLabel(label, line.slot);
    // Draw initial text
    drawLineValue(line);
    _lines[name].initialized = true;
}

void Logger::updateLine(const String &name, float value) {
    if (!display_initialized)
        return;

    auto it = _lines.find(name);
    if (it == _lines.end()) {
        // Line doesn't exist, ignore
        return;
    }

    DisplayLine &line = it->second;

    // Only update if value has changed (avoid unnecessary screen writes)
    if (abs(line.num_value - value) < 0.001f) {
        return;
    }

    line.num_value = value;
    line.is_text = false;

    // Throttle display updates
    unsigned long current_time = millis();
    if (current_time - last_display_update < DISPLAY_INTERVAL_MS) {
        return;
    }
    last_display_update = current_time;

    // Check if text will overflow
    char buf[32];
    if (line.unit.length() > 0) {
        snprintf(buf, sizeof(buf), "%.2f %s", value, line.unit.c_str());
    } else {
        snprintf(buf, sizeof(buf), "%.2f", value);
    }
    int text_width = strlen(buf) * 6;
    int available_width = 128 - _layout.value_x;
    line.uses_wrap = (text_width > available_width);

    drawLineValue(line);
}

void Logger::updateLineText(const String &name, const String &text) {
    if (!display_initialized)
        return;

    auto it = _lines.find(name);
    if (it == _lines.end()) {
        // Line doesn't exist, ignore
        return;
    }

    DisplayLine &line = it->second;

    // Only update if text has changed
    if (line.text_value == text) {
        return;
    }

    line.text_value = text;
    line.is_text = true;

    // Throttle display updates
    unsigned long current_time = millis();
    if (current_time - last_display_update < DISPLAY_INTERVAL_MS) {
        return;
    }
    last_display_update = current_time;

    // Check if text will overflow
    int text_width = text.length() * 6;
    int available_width = 128 - _layout.value_x;
    line.uses_wrap = (text_width > available_width);

    drawLineValue(line);
}

bool Logger::hasLine(const String &name) const {
    return _lines.find(name) != _lines.end();
}
