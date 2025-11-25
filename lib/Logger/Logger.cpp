#include "Logger.h"
#include "config.h"

Logger::Logger()
    : _screen(nullptr), _backlight(-1), display_initialized(false),
      last_display_update(0), _layout{0}, _log_count(0), _log_area_y_start(0),
      _spinner_index(0), _last_spinner_update(0) {}

void Logger::initializeDisplay() {
    // Initialize Serial
    Serial.begin(115200);
    while (!Serial && millis() < 1000) {
        ; // Wait up to 1 second for serial
    }

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

    // Calculate log area position (160px height - (LOG_AREA_LINES *
    // LINE_HEIGHT) - 1px separator)
    _log_area_y_start = 160 - (LOG_AREA_LINES * LINE_HEIGHT);

    // Draw separator line
    drawSeparatorLine();

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

    // Check if we need to wrap (either label too long or value too long)
    int label_width = line.label.length() * 6;
    int text_width = line.value.length() * 6;
    int available_width = 128 - _layout.value_x;
    bool should_wrap =
        (label_width > _layout.value_x) || (text_width > available_width);

    if (should_wrap) {
        // Wrap to next line below label
        // Don't clear the label line at all - only clear the wrapped line
        fillBox(0, y + _layout.line_height, 128, _layout.line_height, 0x0000);
        printLine(line.value.c_str(), 0, y + _layout.line_height, 1);
    } else {
        // Normal display on same line - only clear the value area
        clearValueArea(y);
        if (line.uses_wrap) {
            // Clear wrapped line if we previously used it
            fillBox(0, y + _layout.line_height, 128, _layout.line_height,
                    0x0000);
        }
        printLine(line.value.c_str(), _layout.value_x, y, 1);
    }
}

void Logger::registerLine(const String &name, const String &label,
                          const String &unit, float initial_value) {
    if (!display_initialized)
        return;

    // Format value string
    char buf[32];
    if (unit.length() > 0) {
        snprintf(buf, sizeof(buf), "%.2f %s", initial_value, unit.c_str());
    } else {
        snprintf(buf, sizeof(buf), "%.2f", initial_value);
    }
    String formatted_value = String(buf);

    if (_lines.find(name) != _lines.end()) {
        // Line already exists, update it
        DisplayLine &line = _lines[name];
        line.value = formatted_value;
        line.unit = unit; // Update unit field for numeric conversion

        // Force redraw by clearing the area and drawing the value
        int y = line.slot * _layout.line_height;
        fillBox(0, y, 128, _layout.line_height * 2,
                0x0000); // Clear both lines in case of wrap
        drawLineLabel(label, line.slot);
        drawLineValue(line);
        return;
    }

    // Check if label is too long or if value will overflow
    int label_width = label.length() * 6;
    int text_width = formatted_value.length() * 6;
    int available_width = 128 - _layout.value_x;
    bool will_wrap =
        (label_width > _layout.value_x) || (text_width > available_width);

    DisplayLine line;
    line.label = label;
    line.value = formatted_value;
    line.unit = unit;
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
    line.value = initial_text;
    line.unit = "";
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

    // Format new value with unit if present
    char buf[32];
    if (line.unit.length() > 0) {
        snprintf(buf, sizeof(buf), "%.2f %s", value, line.unit.c_str());
    } else {
        snprintf(buf, sizeof(buf), "%.2f", value);
    }
    String new_value = String(buf);

    // Only update if value has changed (avoid unnecessary screen writes)
    if (line.value == new_value) {
        return;
    }

    line.value = new_value;

    // Throttle display updates
    unsigned long current_time = millis();
    if (current_time - last_display_update < DISPLAY_INTERVAL_MS) {
        return;
    }
    last_display_update = current_time;

    // Check if text will overflow
    int text_width = new_value.length() * 6;
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
    if (line.value == text) {
        return;
    }

    line.value = text;

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

void Logger::update() { updateSpinner(); }

bool Logger::hasLine(const String &name) const {
    return _lines.find(name) != _lines.end();
}

void Logger::drawSeparatorLine() {
    if (!_screen)
        return;
    // Draw 1px white line above log area
    _screen->drawFastHLine(0, _log_area_y_start - 1, 128, COLOR_RGB565_WHITE);
}

void Logger::updateSpinner() {
    if (!_screen)
        return;

    unsigned long now = millis();

    // Always check if enough time has passed, regardless of when this is called
    if (now - _last_spinner_update >= 250) {
        _last_spinner_update = now;

        // Unicode spinner characters: ⠋ ⠙ ⠹ ⠸ ⠼ ⠴ ⠦ ⠧ ⠇ ⠏
        const char *spinner_chars[] = {"|", "/", "-", "\\"};
        _spinner_index = (_spinner_index + 1) % 4;

        // Position: far right, just above separator line
        int x = 128 - 7; // 6px per char + 1px margin
        int y = _log_area_y_start - 1 - LINE_HEIGHT;

        // Clear spinner area (7x12 pixels)
        fillBox(x, y, 7, LINE_HEIGHT, COLOR_RGB565_BLACK);

        // Draw spinner character
        _screen->setTextSize(1);
        _screen->setTextColor(COLOR_RGB565_WHITE);
        _screen->setCursor(x, y);
        _screen->print(spinner_chars[_spinner_index]);
    }
}

void Logger::drawLogArea() {
    if (!_screen)
        return;

    // Clear log area (below separator line)
    fillBox(0, _log_area_y_start, 128, LOG_AREA_LINES * LINE_HEIGHT,
            COLOR_RGB565_BLACK);

    // Draw each log line (with 1px margin from separator)
    for (int i = 0; i < _log_count && i < LOG_AREA_LINES; i++) {
        int y = _log_area_y_start + (i * LINE_HEIGHT) + 1; // +1 for margin
        _screen->setTextSize(1);
        _screen->setTextColor(COLOR_RGB565_WHITE);
        _screen->setCursor(0, y);
        _screen->print(_log_lines[i]);
    }
}

void Logger::log(const String &message) {
    // Output to Serial
    Serial.println(message);

    if (!display_initialized || !_screen)
        return;

    // Word wrap: split message into chunks that fit on screen (21 chars @
    // 6px/char)
    const int MAX_CHARS_PER_LINE = 21;
    String remaining = message;

    while (remaining.length() > 0) {
        String line;
        if (remaining.length() <= MAX_CHARS_PER_LINE) {
            line = remaining;
            remaining = "";
        } else {
            // Find last space within limit
            int split_pos = remaining.lastIndexOf(' ', MAX_CHARS_PER_LINE);
            if (split_pos == -1 || split_pos == 0) {
                // No space found, hard break
                split_pos = MAX_CHARS_PER_LINE;
            }
            line = remaining.substring(0, split_pos);
            remaining = remaining.substring(split_pos);
            remaining.trim(); // Remove leading space
        }

        // Add line to log buffer (with scrolling)
        if (_log_count >= LOG_AREA_LINES) {
            // Shift all logs up
            for (int i = 0; i < LOG_AREA_LINES - 1; i++) {
                _log_lines[i] = _log_lines[i + 1];
            }
            _log_lines[LOG_AREA_LINES - 1] = line;
        } else {
            _log_lines[_log_count] = line;
            _log_count++;
        }
    }

    // Redraw entire log area
    drawLogArea();
}
