#include "PT100.h"

PT100Sensor::PT100Sensor(Logger &logger, const char *label)
    : _logger(logger), _rtd(PIN_MAX31865_CS), _label(label),
      _last_temperature(0.0f), _initialized(false), _in_error_state(false),
      _last_update_time(0) {}

void PT100Sensor::begin() {
    if (_initialized)
        return; // prevent re-initialization

    _rtd.begin(MAX31865_3WIRE);

    // Format label once and cache it
    Logger::formatLabel(_formatted_label, sizeof(_formatted_label), _label);

    // Check initial reading to see if sensor is connected
    delay(100); // allow sensor to stabilize
    float temp_c = _rtd.temperature(RNOMINAL, RREF);
    bool has_error = (temp_c < PT100_ERROR_MIN_C || temp_c > PT100_ERROR_MAX_C);

    if (has_error) {
        _in_error_state = true;
        _logger.registerTextLine(_label, _formatted_label, "ERROR");
        _logger.log("PT100 sensor error");
    } else {
        _logger.registerLine(_label, _formatted_label, "C", temp_c);
        _logger.log("PT100 initialized.");
        _last_temperature = temp_c;
    }

    _initialized = true;
}

void PT100Sensor::update() {
    if (!_initialized)
        return;

    unsigned long current_time = millis();
    if (current_time - _last_update_time < PT100_UPDATE_INTERVAL_MS) {
        return;
    }
    _last_update_time = current_time;

    float temp_c = _rtd.temperature(RNOMINAL, RREF);

    // Check for error conditions (invalid reading or sensor fault)
    bool has_error = (temp_c < PT100_ERROR_MIN_C || temp_c > PT100_ERROR_MAX_C);

    if (has_error) {
        if (!_in_error_state) {
            // Entering error state - log once and update display
            _in_error_state = true;
            _logger.log("PT100 sensor error");
            _logger.updateLineText(_label, "ERROR");
        }

        // Clear any faults to prevent accumulation
        uint8_t fault = _rtd.readFault();
        if (fault) {
            _rtd.clearFault();
        }
        return;
    }

    // Valid reading - exit error state if we were in it
    if (_in_error_state) {
        _in_error_state = false;
        _logger.log("PT100 sensor recovered");
        // Re-register as numeric line (use cached formatted label)
        _logger.registerLine(_label, _formatted_label, "C", temp_c);
    }

    _logger.updateLine(_label, temp_c);
    _last_temperature = temp_c;
}
