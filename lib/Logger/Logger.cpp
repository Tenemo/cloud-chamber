#include "Logger.h"
#include "config.h"

Logger::Logger()
    : _screen(nullptr), _backlight(-1), _display_initialized(false),
      _last_display_update(0), _layout{0}, _log_count(0), _log_area_y_start(0),
      _spinner_index(0), _last_spinner_update(0) {}

void Logger::initializeDisplay() {
    if (_display_initialized)
        return; // Prevent re-initialization

    // Initialize Serial
    Serial.begin(115200);
    while (!Serial && millis() < SERIAL_TIMEOUT_MS) {
        ; // wait for serial connection
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
    _screen->setTextWrap(false); // disable automatic text wrapping
    _screen->setCursor(0, 0);

    // Calculate log area position
    _log_area_y_start = SCREEN_HEIGHT - (LOG_AREA_LINES * LINE_HEIGHT);

    // Draw separator line
    drawSeparatorLine();

    _display_initialized = true;
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
    _screen->print(label);
}

void Logger::drawChangedCharacters(const String &old_val, const String &new_val,
                                   int x, int y) {
    if (!_screen)
        return;

    _screen->setTextSize(1);

    int max_len = max(old_val.length(), new_val.length());

    for (int i = 0; i < max_len; i++) {
        int char_x = x + (i * CHAR_WIDTH);
        char old_char = (i < (int)old_val.length()) ? old_val[i] : '\0';
        char new_char = (i < (int)new_val.length()) ? new_val[i] : '\0';

        // Only redraw if character changed
        if (old_char != new_char) {
            // Clear the character position
            fillBox(char_x, y, CHAR_WIDTH, _layout.line_height, 0x0000);

            // Draw new character if not null
            if (new_char != '\0') {
                _screen->setTextColor(COLOR_RGB565_WHITE);
                _screen->setCursor(char_x, y);
                _screen->print(new_char);
            }
        }
    }
}

void Logger::drawLineValue(DisplayLine &line, bool force_full_redraw) {
    if (!_screen)
        return;

    int y = line.slot * _layout.line_height;

    // Check if we need to wrap (either label too long or value too long)
    int label_width = line.label.length() * CHAR_WIDTH;
    int text_width = line.value.length() * CHAR_WIDTH;
    int available_width = SCREEN_WIDTH - _layout.value_x;
    bool should_wrap =
        (label_width > _layout.value_x) || (text_width > available_width);

    // Determine if we need a full redraw (wrap state changed, or forced)
    bool wrap_changed = (should_wrap != line.uses_wrap);
    bool need_full_redraw =
        force_full_redraw || wrap_changed || line.prev_value.length() == 0;

    if (should_wrap) {
        int value_y = y + _layout.line_height;
        if (need_full_redraw) {
            // Full redraw for wrapped line
            fillBox(0, value_y, SCREEN_WIDTH, _layout.line_height, 0x0000);
            printLine(line.value.c_str(), 0, value_y, 1);
        } else {
            // Smart character-by-character update for wrapped line
            drawChangedCharacters(line.prev_value, line.value, 0, value_y);
        }
    } else {
        if (need_full_redraw) {
            // Full redraw for normal line
            clearValueArea(y);
            if (line.uses_wrap) {
                // Clear wrapped line if we previously used it
                fillBox(0, y + _layout.line_height, SCREEN_WIDTH,
                        _layout.line_height, 0x0000);
            }
            printLine(line.value.c_str(), _layout.value_x, y, 1);
        } else {
            // Smart character-by-character update
            drawChangedCharacters(line.prev_value, line.value, _layout.value_x,
                                  y);
        }
    }

    // Update previous value and wrap state
    line.prev_value = line.value;
    line.uses_wrap = should_wrap;
}

void Logger::registerLineInternal(const String &name, const String &label,
                                  const String &value, const String &unit) {
    if (_lines.find(name) != _lines.end()) {
        // Line already exists, update it
        DisplayLine &line = _lines[name];
        line.value = value;
        line.unit = unit;

        // Force redraw by clearing the area and drawing the value
        // Only clear 2 lines if this line uses wrapping, otherwise just clear 1
        int y = line.slot * _layout.line_height;
        int clear_height =
            line.uses_wrap ? _layout.line_height * 2 : _layout.line_height;
        fillBox(0, y, SCREEN_WIDTH, clear_height, 0x0000);
        drawLineLabel(label, line.slot);
        line.prev_value = ""; // Force full redraw
        drawLineValue(line, true);
        return;
    }

    // Check if label is too long or if value will overflow
    int label_width = label.length() * CHAR_WIDTH;
    int text_width = value.length() * CHAR_WIDTH;
    int available_width = SCREEN_WIDTH - _layout.value_x;
    bool will_wrap =
        (label_width > _layout.value_x) || (text_width > available_width);

    DisplayLine line;
    line.label = label;
    line.value = value;
    line.prev_value = ""; // Empty to force full initial draw
    line.unit = unit;
    line.slot = _layout.next_slot++;
    line.initialized = false;
    line.uses_wrap = will_wrap;
    line.needs_redraw = false; // Initial draw happens immediately below

    // Only reserve extra slot if text actually wraps
    if (will_wrap) {
        _layout.next_slot++;
    }

    _lines[name] = line;

    // Draw label immediately
    drawLineLabel(label, _lines[name].slot);
    // Draw initial value (force full redraw)
    drawLineValue(_lines[name], true);
    _lines[name].initialized = true;
}

void Logger::registerLine(const String &name, const String &label,
                          const String &unit, float initial_value) {
    if (!_display_initialized)
        return;

    // Format value string
    char buf[32];
    if (unit.length() > 0) {
        snprintf(buf, sizeof(buf), "%.2f %s", initial_value, unit.c_str());
    } else {
        snprintf(buf, sizeof(buf), "%.2f", initial_value);
    }
    String formatted_value = String(buf);

    registerLineInternal(name, label, formatted_value, unit);
}

void Logger::registerTextLine(const String &name, const String &label,
                              const String &initial_text) {
    if (!_display_initialized)
        return;

    registerLineInternal(name, label, initial_text, "");
}

void Logger::updateLine(const String &name, float value) {
    if (!_display_initialized)
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

    // Check if text will overflow
    int text_width = new_value.length() * CHAR_WIDTH;
    int available_width = SCREEN_WIDTH - _layout.value_x;
    line.uses_wrap = (text_width > available_width);

    // Mark for redraw in next update() cycle
    line.needs_redraw = true;
}

void Logger::updateLineText(const String &name, const String &text) {
    if (!_display_initialized)
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

    // Check if text will overflow
    int text_width = text.length() * CHAR_WIDTH;
    int available_width = SCREEN_WIDTH - _layout.value_x;
    line.uses_wrap = (text_width > available_width);

    // Mark for redraw in next update() cycle
    line.needs_redraw = true;
}

void Logger::update() {
    updateSpinner();

    // Throttle batch redraws
    unsigned long current_time = millis();
    if (current_time - _last_display_update < DISPLAY_INTERVAL_MS) {
        return;
    }

    // Redraw all dirty lines
    bool any_redrawn = false;
    for (auto &pair : _lines) {
        if (pair.second.needs_redraw) {
            drawLineValue(pair.second);
            pair.second.needs_redraw = false;
            any_redrawn = true;
        }
    }

    if (any_redrawn) {
        _last_display_update = current_time;
    }
}

void Logger::drawSeparatorLine() {
    if (!_screen)
        return;
    // Draw 1px white line above log area
    _screen->drawFastHLine(0, _log_area_y_start - 1, SCREEN_WIDTH,
                           COLOR_RGB565_WHITE);
}

void Logger::updateSpinner() {
    if (!_screen)
        return;

    unsigned long now = millis();

    // Always check if enough time has passed, regardless of when this is called
    if (now - _last_spinner_update >= SPINNER_UPDATE_MS) {
        _last_spinner_update = now;

        // Unicode spinner characters: ⠋ ⠙ ⠹ ⠸ ⠼ ⠴ ⠦ ⠧ ⠇ ⠏
        const char *spinner_chars[] = {"|", "/", "-", "\\"};
        _spinner_index = (_spinner_index + 1) % 4;

        // Position: far right, just above separator line
        int x = SCREEN_WIDTH - (CHAR_WIDTH + 1); // char width + 1px margin
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
    fillBox(0, _log_area_y_start, SCREEN_WIDTH, LOG_AREA_LINES * LINE_HEIGHT,
            COLOR_RGB565_BLACK);

    // Draw each log line (with 1px margin from separator)
    for (int i = 0; i < _log_count && i < LOG_AREA_LINES; i++) {
        int y = _log_area_y_start + (i * LINE_HEIGHT) + 1; // +1 for margin
        printLine(_log_lines[i], 0, y, 1);
    }
}

void Logger::log(const String &message, bool serialOnly) {
    // Output to Serial
    Serial.println(message);

    if (serialOnly || !_display_initialized || !_screen)
        return;

    // Word wrap: split message into chunks that fit on screen
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
                strncpy(_log_lines[i], _log_lines[i + 1], MAX_CHARS_PER_LINE);
                _log_lines[i][MAX_CHARS_PER_LINE] = '\0';
            }
            strncpy(_log_lines[LOG_AREA_LINES - 1], line.c_str(),
                    MAX_CHARS_PER_LINE);
            _log_lines[LOG_AREA_LINES - 1][MAX_CHARS_PER_LINE] = '\0';
        } else {
            strncpy(_log_lines[_log_count], line.c_str(), MAX_CHARS_PER_LINE);
            _log_lines[_log_count][MAX_CHARS_PER_LINE] = '\0';
            _log_count++;
        }
    }

    // Redraw entire log area
    drawLogArea();
}
