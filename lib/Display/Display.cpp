#include "Display.h"

Display::Display(int8_t dc, int8_t cs, int8_t rst, int8_t backlight)
    : _screen(dc, cs, rst), _backlight(backlight), _textSize(1),
      _textColor(COLOR_RGB565_WHITE) {}

void Display::begin() {
    pinMode(_backlight, OUTPUT);
    digitalWrite(_backlight, HIGH);
    _screen.begin();
    _screen.setRotation(0);
    _screen.fillScreen(COLOR_RGB565_BLACK);
    _screen.setTextColor(_textColor);
    _screen.setTextSize(_textSize);
    _screen.setCursor(0, 0);
}

void Display::clear() { _screen.fillScreen(COLOR_RGB565_BLACK); }

void Display::printLine(const char *text, int x, int y, uint8_t textSize) {
    _screen.setTextSize(textSize);
    _screen.setTextColor(COLOR_RGB565_WHITE);
    _screen.setCursor(x, y);
    _screen.println(text);
}

void Display::fillBox(int x, int y, int w, int h, uint16_t color) {
    _screen.fillRect(x, y, w, h, color);
}

void Display::drawBox(int x, int y, int w, int h, uint16_t color) {
    _screen.drawRect(x, y, w, h, color);
}

void Display::setRotation(uint8_t rotation) { _screen.setRotation(rotation); }
