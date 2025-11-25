#include "PT100.h"
#include "config.h"

PT100Sensor::PT100Sensor(Logger &logger)
    : _logger(logger), _rtd(nullptr), _last_temperature(0.0f),
      _initialized(false) {}

void PT100Sensor::begin() {
    _rtd = new Adafruit_MAX31865(PIN_MAX31865_CS);
    _rtd->begin(MAX31865_3WIRE);
    _initialized = true;
}

void PT100Sensor::update() {
    if (!_initialized || !_rtd)
        return;

    float temp_c = _rtd->temperature(RNOMINAL, RREF);

    // Register temperature line on first update
    if (!_logger.hasLine("temp")) {
        _logger.registerLine("temp", "Temp:", "C", temp_c);
    } else {
        _logger.updateLine("temp", temp_c);
    }

    _last_temperature = temp_c;
}
