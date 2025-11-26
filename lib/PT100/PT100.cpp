#include "PT100.h"
#include "config.h"

PT100Sensor::PT100Sensor(Logger &logger)
    : _logger(logger), _rtd(nullptr), _last_temperature(0.0f),
      _initialized(false), _in_error_state(false), _last_update_time(0) {}

void PT100Sensor::begin() {
    if (_initialized)
        return; // Prevent re-initialization

    _rtd = new Adafruit_MAX31865(PIN_MAX31865_CS);
    _rtd->begin(MAX31865_3WIRE);
    _initialized = true;

    // Check initial reading to see if sensor is connected
    delay(100); // Allow sensor to stabilize
    float temp_c = _rtd->temperature(RNOMINAL, RREF);
    bool has_error = (temp_c < -100.0f || temp_c > 500.0f);

    if (has_error) {
        _in_error_state = true;
        _logger.registerTextLine("temp", "PT100:", "ERROR");
        _logger.log("PT100 sensor error");
    } else {
        _logger.registerLine("temp", "PT100:", "C", temp_c);
        _logger.log("PT100 initialized.");
    }
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

    // Check for error conditions (invalid reading or sensor fault)
    bool has_error = (temp_c < -100.0f || temp_c > 500.0f);

    if (has_error) {
        if (!_in_error_state) {
            // Entering error state - log once and update display
            _in_error_state = true;
            _logger.log("PT100 sensor error");
            _logger.updateLineText("temp", "ERROR");
        }

        // Clear any faults to prevent accumulation
        uint8_t fault = _rtd->readFault();
        if (fault) {
            _rtd->clearFault();
        }
        return;
    }

    // Valid reading - exit error state if we were in it
    if (_in_error_state) {
        _in_error_state = false;
        _logger.log("PT100 sensor recovered");
        // Re-register as numeric line
        _logger.registerLine("temp", "PT100:", "C", temp_c);
    }

    _logger.updateLine("temp", temp_c);
    _last_temperature = temp_c;
}
