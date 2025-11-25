#include "PT100.h"
#include "config.h"

PT100Sensor::PT100Sensor(Logger &logger)
    : _logger(logger), _rtd(nullptr), _last_temperature(0.0f),
      _initialized(false), _last_update_time(0) {}

void PT100Sensor::begin() {
    _rtd = new Adafruit_MAX31865(PIN_MAX31865_CS);
    _rtd->begin(MAX31865_3WIRE);
    _initialized = true;

    // Register display line immediately
    _logger.registerLine("temp", "Temp:", "C", 0.0f);
    _logger.log("PT100 initialized.");
}

void PT100Sensor::update() {
    if (!_initialized || !_rtd)
        return;

    unsigned long current_time = millis();
    if (current_time - _last_update_time < SENSOR_UPDATE_INTERVAL_MS) {
        return;
    }
    _last_update_time = current_time;

    float temp_c = _rtd->temperature(RNOMINAL, RREF);
    _logger.updateLine("temp", temp_c);
    _last_temperature = temp_c;
}
