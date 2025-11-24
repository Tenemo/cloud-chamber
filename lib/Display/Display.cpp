#include "Display.h"

Display::Display(int8_t dc, int8_t cs, int8_t rst, int8_t backlight)
    : tft(dc, cs, rst), _backlight(backlight), _textSize(1),
      _textColor(COLOR_RGB565_WHITE) {}

void Display::begin() {
    pinMode(_backlight, OUTPUT);
    digitalWrite(_backlight, HIGH);
    tft.begin();
    tft.setRotation(0);
    tft.fillScreen(COLOR_RGB565_BLACK);
    tft.setTextColor(_textColor);
    tft.setTextSize(_textSize);
    tft.setCursor(0, 0);
}

void Display::clear() { tft.fillScreen(COLOR_RGB565_BLACK); }

void Display::printLine(const char *text, int x, int y, uint16_t color,
                        uint8_t textSize) {
    tft.setTextSize(textSize);
    tft.setTextColor(color);
    tft.setCursor(x, y);
    tft.println(text);
}

void Display::fillBox(int x, int y, int w, int h, uint16_t color) {
    tft.fillRect(x, y, w, h, color);
}

void Display::drawBox(int x, int y, int w, int h, uint16_t color) {
    tft.drawRect(x, y, w, h, color);
}

void Display::setRotation(uint8_t rotation) { tft.setRotation(rotation); }

DFRobot_ST7789_240x320_HW_SPI *Display::getTFT() { return &tft; }
