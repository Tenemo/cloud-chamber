#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "Display.h"
#include "config.h"
#include <Arduino.h>

Display display(TFT_DC, TFT_CS, TFT_RST, LCD_BL);

void setup() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    Serial.begin(115200);
    Serial.println("Initializing...");

    pinMode(LED_PIN, OUTPUT);

    display.begin();

    display.printLine("Cloud Chamber", 0, 0);

    Serial.println("Initialized.");
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);

    digitalWrite(LED_PIN, LOW);
    delay(1000);
}
