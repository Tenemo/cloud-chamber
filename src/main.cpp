#include "Display.h"
#include "config.h"
#include <Arduino.h>

Display display(TFT_DC, TFT_CS, TFT_RST, LCD_BL);

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing...");

    pinMode(LED_PIN, OUTPUT);

    display.begin();

    display.printLine("Cloud Chamber", 0, 0);
    display.printLine("LED Blink Demo", 0, 10);

    Serial.println("Initialized.");
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    display.printLine("LED ON", 0, 30, COLOR_RGB565_GREEN);
    Serial.println("LED ON");
    delay(1000);

    digitalWrite(LED_PIN, LOW);
    display.printLine("LED OFF", 0, 30, COLOR_RGB565_RED);
    Serial.println("LED OFF");
    delay(1000);
}
