/**
 * @file Logger.cpp
 * @brief Implementation of display manager with fixed-size buffers
 *
 * This implementation uses fixed-size char arrays instead of Arduino String
 * to prevent heap fragmentation during long-running operation.
 */

#include "Logger.h"
#include "TimeService.h"
#include "config.h"
#include <cstdarg>
#include <cstring>

Logger::Logger()
    : _screen(nullptr), _backlight(-1), _display_initialized(false),
      _last_display_update(0), _layout{0}, _log_count(0), _log_area_y_start(0),
      _spinner_index(0), _last_spinner_update(0) {
    // Initialize all display lines as inactive
    for (size_t i = 0; i < MAX_DISPLAY_LINES; i++) {
        _lines[i].active = false;
    }
}

void Logger::formatLabel(char *buf, size_t size, const char *label) {
    snprintf(buf, size, "%s:", label);
}

bool Logger::shouldWrap(const char *label, const char *value) const {
    int label_width = static_cast<int>(strlen(label)) * CHAR_WIDTH;
    int text_width = static_cast<int>(strlen(value)) * CHAR_WIDTH;
    int available_width = SCREEN_WIDTH - _layout.value_x;
    return (label_width > _layout.value_x) || (text_width > available_width);
}

DisplayLine *Logger::findLine(const char *name) {
    for (size_t i = 0; i < MAX_DISPLAY_LINES; i++) {
        if (_lines[i].active && strcmp(_lines[i].name, name) == 0) {
            return &_lines[i];
        }
    }
    return nullptr;
}

DisplayLine *Logger::findFreeSlot() {
    for (size_t i = 0; i < MAX_DISPLAY_LINES; i++) {
        if (!_lines[i].active) {
            return &_lines[i];
        }
    }
    return nullptr;
}

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
    _log_area_y_start =
        SCREEN_HEIGHT - (Display::LOG_AREA_LINES * LINE_HEIGHT);

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

void Logger::drawDeltaChar(int x, int y) {
    if (!_screen)
        return;

    // Draw a delta (triangle) character: Δ
    // Triangle fits in a 5x7 pixel box (standard char size)
    _screen->drawLine(x + 2, y + 1, x, y + 6, COLOR_RGB565_WHITE); // Left edge
    _screen->drawLine(x + 2, y + 1, x + 4, y + 6,
                      COLOR_RGB565_WHITE); // Right edge
    _screen->drawLine(x, y + 6, x + 4, y + 6,
                      COLOR_RGB565_WHITE); // Bottom edge
}

void Logger::drawLineLabel(const char *label, int slot) {
    if (!_screen)
        return;
    int y = slot * _layout.line_height;

    _screen->setTextSize(1);
    _screen->setTextColor(COLOR_RGB565_WHITE);

    // Check if label starts with special delta marker "\delta"
    if (strncmp(label, "\\delta", 6) == 0) {
        // Draw custom delta character
        drawDeltaChar(0, y);
        // Print the rest of the label after the marker
        _screen->setCursor(CHAR_WIDTH, y);
        _screen->print(&label[6]);
    } else {
        // Normal label printing
        _screen->setCursor(0, y);
        _screen->print(label);
    }
}

void Logger::drawChangedCharacters(const char *old_val, const char *new_val,
                                   int x, int y) {
    if (!_screen)
        return;

    _screen->setTextSize(1);

    size_t old_len = strlen(old_val);
    size_t new_len = strlen(new_val);
    size_t max_len = (old_len > new_len) ? old_len : new_len;

    for (size_t i = 0; i < max_len; i++) {
        int char_x = x + (static_cast<int>(i) * CHAR_WIDTH);
        char old_char = (i < old_len) ? old_val[i] : '\0';
        char new_char = (i < new_len) ? new_val[i] : '\0';

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

    // Check if we need to wrap
    bool should_wrap_now = shouldWrap(line.label, line.value);

    // Determine if we need a full redraw
    bool wrap_changed = (should_wrap_now != line.uses_wrap);
    bool need_full_redraw =
        force_full_redraw || wrap_changed || line.prev_value[0] == '\0';

    if (should_wrap_now) {
        int value_y = y + _layout.line_height;
        if (need_full_redraw) {
            // Full redraw for wrapped line
            fillBox(0, value_y, SCREEN_WIDTH, _layout.line_height, 0x0000);
            printLine(line.value, 0, value_y, 1);
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
            printLine(line.value, _layout.value_x, y, 1);
        } else {
            // Smart character-by-character update
            drawChangedCharacters(line.prev_value, line.value, _layout.value_x,
                                  y);
        }
    }

    // Update previous value and wrap state
    strncpy(line.prev_value, line.value, MAX_LINE_VALUE_LEN - 1);
    line.prev_value[MAX_LINE_VALUE_LEN - 1] = '\0';
    line.uses_wrap = should_wrap_now;
}

void Logger::registerLineInternal(const char *name, const char *label,
                                  const char *value, const char *unit) {
    // Check if line already exists
    DisplayLine *existing = findLine(name);
    if (existing != nullptr) {
        // Update existing line
        strncpy(existing->value, value, MAX_LINE_VALUE_LEN - 1);
        existing->value[MAX_LINE_VALUE_LEN - 1] = '\0';
        strncpy(existing->unit, unit, MAX_LINE_UNIT_LEN - 1);
        existing->unit[MAX_LINE_UNIT_LEN - 1] = '\0';

        // Force redraw
        int y = existing->slot * _layout.line_height;
        int clear_height =
            existing->uses_wrap ? _layout.line_height * 2 : _layout.line_height;
        fillBox(0, y, SCREEN_WIDTH, clear_height, 0x0000);
        drawLineLabel(label, existing->slot);
        existing->prev_value[0] = '\0'; // Force full redraw
        drawLineValue(*existing, true);
        return;
    }

    // Find a free slot
    DisplayLine *line = findFreeSlot();
    if (line == nullptr) {
        // No free slots available
        Serial.println("Logger: No free display slots!");
        return;
    }

    // Initialize the line
    line->active = true;
    strncpy(line->name, name, MAX_LINE_NAME_LEN - 1);
    line->name[MAX_LINE_NAME_LEN - 1] = '\0';
    strncpy(line->label, label, MAX_LINE_LABEL_LEN - 1);
    line->label[MAX_LINE_LABEL_LEN - 1] = '\0';
    strncpy(line->value, value, MAX_LINE_VALUE_LEN - 1);
    line->value[MAX_LINE_VALUE_LEN - 1] = '\0';
    line->prev_value[0] = '\0'; // Empty to force full initial draw
    strncpy(line->unit, unit, MAX_LINE_UNIT_LEN - 1);
    line->unit[MAX_LINE_UNIT_LEN - 1] = '\0';
    line->slot = _layout.next_slot++;
    line->uses_wrap = shouldWrap(label, value);
    line->needs_redraw = false;

    // Reserve extra slot if text wraps
    if (line->uses_wrap) {
        _layout.next_slot++;
    }

    // Draw label immediately
    drawLineLabel(line->label, line->slot);
    // Draw initial value (force full redraw)
    drawLineValue(*line, true);
}

void Logger::registerLine(const char *name, const char *label, const char *unit,
                          float initial_value) {
    if (!_display_initialized)
        return;

    // Format value string
    char buf[MAX_LINE_VALUE_LEN];
    if (unit != nullptr && unit[0] != '\0') {
        snprintf(buf, sizeof(buf), "%.2f %s", initial_value, unit);
    } else {
        snprintf(buf, sizeof(buf), "%.2f", initial_value);
    }

    registerLineInternal(name, label, buf, unit ? unit : "");
}

void Logger::registerTextLine(const char *name, const char *label,
                              const char *initial_text) {
    if (!_display_initialized)
        return;

    registerLineInternal(name, label, initial_text ? initial_text : "", "");
}

void Logger::updateLine(const char *name, float value) {
    if (!_display_initialized)
        return;

    DisplayLine *line = findLine(name);
    if (line == nullptr) {
        return; // Line doesn't exist
    }

    // Format new value with unit if present
    char buf[MAX_LINE_VALUE_LEN];
    if (line->unit[0] != '\0') {
        snprintf(buf, sizeof(buf), "%.2f %s", value, line->unit);
    } else {
        snprintf(buf, sizeof(buf), "%.2f", value);
    }

    // Only update if value has changed
    if (strcmp(line->value, buf) == 0) {
        return;
    }

    strncpy(line->value, buf, MAX_LINE_VALUE_LEN - 1);
    line->value[MAX_LINE_VALUE_LEN - 1] = '\0';

    // Check if text will overflow
    line->uses_wrap = shouldWrap(line->label, line->value);

    // Mark for redraw in next update() cycle
    line->needs_redraw = true;
}

void Logger::updateLineText(const char *name, const char *text) {
    if (!_display_initialized)
        return;

    DisplayLine *line = findLine(name);
    if (line == nullptr) {
        return; // Line doesn't exist
    }

    // Only update if text has changed
    if (strcmp(line->value, text) == 0) {
        return;
    }

    strncpy(line->value, text, MAX_LINE_VALUE_LEN - 1);
    line->value[MAX_LINE_VALUE_LEN - 1] = '\0';

    // Check if text will overflow
    line->uses_wrap = shouldWrap(line->label, line->value);

    // Mark for redraw in next update() cycle
    line->needs_redraw = true;
}

void Logger::update() {
    updateSpinner();

    // Throttle batch redraws
    unsigned long current_time = millis();
    if (current_time - _last_display_update < Display::DISPLAY_INTERVAL_MS) {
        return;
    }

    // Redraw all dirty lines
    bool any_redrawn = false;
    for (size_t i = 0; i < MAX_DISPLAY_LINES; i++) {
        if (_lines[i].active && _lines[i].needs_redraw) {
            drawLineValue(_lines[i]);
            _lines[i].needs_redraw = false;
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
    if (now - _last_spinner_update >= Display::SPINNER_UPDATE_MS) {
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
    fillBox(0, _log_area_y_start, SCREEN_WIDTH,
            Display::LOG_AREA_LINES * LINE_HEIGHT,
            COLOR_RGB565_BLACK);

    // Draw each log line (with 1px margin from separator)
    for (int i = 0; i < _log_count && i < Display::LOG_AREA_LINES; i++) {
        int y = _log_area_y_start + (i * LINE_HEIGHT) + 1;
        printLine(_log_lines[i], 0, y, 1);
    }
}

void Logger::log(const char *message, bool serialOnly) {
    const char *iso = TimeService::getIsoTimestamp();

    // Build boot-relative timestamp (always included)
    unsigned long ms = millis();
    unsigned int hours = (ms / 3600000) % 24;
    unsigned int mins = (ms / 60000) % 60;
    unsigned int secs = (ms / 1000) % 60;
    unsigned int millis_part = ms % 1000;

    // Serial output format:
    // - With wall time: [YYYY-MM-DDTHH:MM:SS][HH:MM:SS.mmm] message
    // - Without wall time: [HH:MM:SS.mmm] message
    const char *serial_out = message;
    char stamped[256];
    if (iso != nullptr) {
        snprintf(stamped, sizeof(stamped), "[%s][%02u:%02u:%02u.%03u] %s", iso,
                 hours, mins, secs, millis_part, message);
        serial_out = stamped;
    } else {
        snprintf(stamped, sizeof(stamped), "[%02u:%02u:%02u.%03u] %s", hours,
                 mins, secs, millis_part, message);
        serial_out = stamped;
    }

    // Output to Serial
    Serial.println(serial_out);

    if (serialOnly || !_display_initialized || !_screen)
        return;

    // Use local buffer for word wrap processing
    char remaining[192];
    strncpy(remaining, message, sizeof(remaining) - 1);
    remaining[sizeof(remaining) - 1] = '\0';

    size_t remaining_len = strlen(remaining);
    size_t pos = 0;

    while (pos < remaining_len) {
        char line_buf[MAX_CHARS_PER_LINE + 1];
        size_t chars_left = remaining_len - pos;

        if (chars_left <= MAX_CHARS_PER_LINE) {
            // Rest fits on one line
            strncpy(line_buf, &remaining[pos], chars_left);
            line_buf[chars_left] = '\0';
            pos = remaining_len;
        } else {
            // Find last space within limit
            size_t split_pos = MAX_CHARS_PER_LINE;
            for (size_t i = MAX_CHARS_PER_LINE; i > 0; i--) {
                if (remaining[pos + i] == ' ') {
                    split_pos = i;
                    break;
                }
            }
            strncpy(line_buf, &remaining[pos], split_pos);
            line_buf[split_pos] = '\0';
            pos += split_pos;
            // Skip leading space
            while (pos < remaining_len && remaining[pos] == ' ')
                pos++;
        }

        // Add line to log buffer (with scrolling)
        if (_log_count >= Display::LOG_AREA_LINES) {
            // Shift all logs up
            for (int i = 0; i < Display::LOG_AREA_LINES - 1; i++) {
                strncpy(_log_lines[i], _log_lines[i + 1], MAX_CHARS_PER_LINE);
                _log_lines[i][MAX_CHARS_PER_LINE] = '\0';
            }
            strncpy(_log_lines[Display::LOG_AREA_LINES - 1], line_buf,
                    MAX_CHARS_PER_LINE);
            _log_lines[Display::LOG_AREA_LINES - 1][MAX_CHARS_PER_LINE] = '\0';
        } else {
            strncpy(_log_lines[_log_count], line_buf, MAX_CHARS_PER_LINE);
            _log_lines[_log_count][MAX_CHARS_PER_LINE] = '\0';
            _log_count++;
        }
    }

    // Redraw entire log area
    drawLogArea();
}

void Logger::logf(const char *format, ...) {
    char buf[192];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    log(buf, false);
}

void Logger::logf(bool serialOnly, const char *format, ...) {
    char buf[192];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    log(buf, serialOnly);
}
