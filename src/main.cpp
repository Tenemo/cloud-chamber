#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "Display.h"
#include "config.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

Display display(TFT_DC, TFT_CS, TFT_RST, LCD_BL);

void setup() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    // Watchdog timer set to 60 seconds,
    // because it doesn't like file uploads.
    esp_task_wdt_init(60, true);
    Serial.begin(115200);
    Serial.println("Initializing...");
    display.printLine("Initializing...", 0, 0, 1);

    pinMode(LED_PIN, OUTPUT);

    display.begin();
    display.clear();

    display.printLine("Initialized.", 0, 0, 1);

    Serial.println("Initialized.");
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);

    digitalWrite(LED_PIN, LOW);
    delay(1000);
}
