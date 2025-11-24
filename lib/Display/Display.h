#ifndef DISPLAY_H
#define DISPLAY_H

#include "DFRobot_GDL.h"
#include <Arduino.h>

class Display {
  public:
    Display(int8_t dc, int8_t cs, int8_t rst, int8_t backlight);

    void begin();
    void clear();
    void printLine(const char *text, int x, int y,
                   uint16_t color = COLOR_RGB565_WHITE, uint8_t textSize = 1);
    void fillBox(int x, int y, int w, int h, uint16_t color);
    void drawBox(int x, int y, int w, int h, uint16_t color);
    void setRotation(uint8_t rotation);

    DFRobot_ST7789_240x320_HW_SPI *getTFT();

  private:
    DFRobot_ST7789_240x320_HW_SPI tft;
    int8_t _backlight;
    uint8_t _textSize;
    uint16_t _textColor;
};

#endif // DISPLAY_H
