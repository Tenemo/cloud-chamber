#include "DS18B20.h"
#include "config.h"

// Static member definitions for shared bus coordination
unsigned long DS18B20Sensor::_shared_conversion_start_time = 0;
bool DS18B20Sensor::_shared_conversion_pending = false;

DS18B20Sensor::DS18B20Sensor(Logger &logger, DallasTemperature &sensors,
                             const uint8_t *address, const char *label)
    : _logger(logger), _sensors(sensors), _label(label), _id(label),
      _last_temperature(0.0f), _initialized(false), _in_error_state(false),
      _last_update_time(0) {
    memcpy(_address, address, 8);
}

void DS18B20Sensor::begin() {
    if (_initialized)
        return; // prevent re-initialization

    // Set resolution to 12-bit for maximum precision
    _sensors.setResolution(_address, 12);

    // Request initial temperature using specific address
    _sensors.requestTemperaturesByAddress(_address);
    delay(750); // wait for 12-bit conversion
    float temp_c = _sensors.getTempC(_address);

    if (temp_c == DEVICE_DISCONNECTED_C || temp_c == TEMP_ERROR_VALUE) {
        _in_error_state = true;
        char labelBuf[16];
        snprintf(labelBuf, sizeof(labelBuf), "%s:", _label);
        _logger.registerTextLine(_id, labelBuf, "ERROR");
        _logger.log("DS18B20 not found");
    } else {
        char labelBuf[16];
        snprintf(labelBuf, sizeof(labelBuf), "%s:", _label);
        _logger.registerLine(_id, labelBuf, "C", temp_c);
        _logger.log("DS18B20 initialized.");
        _last_temperature = temp_c;
    }

    _initialized = true;
}

void DS18B20Sensor::update() {
    if (!_initialized)
        return;

    unsigned long current_time = millis();

    // Check if this sensor needs an update
    if (current_time - _last_update_time < DS18B20_UPDATE_INTERVAL_MS) {
        return;
    }

    // If no conversion is pending, start one for ALL sensors on the bus
    if (!_shared_conversion_pending) {
        _sensors.requestTemperatures(); // Request all sensors at once
        _shared_conversion_start_time = current_time;
        _shared_conversion_pending = true;
        return;
    }

    // Check if conversion is complete
    if (current_time - _shared_conversion_start_time < CONVERSION_DELAY_MS) {
        return;
    }

    // Conversion complete - read this sensor's temperature
    _last_update_time = current_time;

    float temp_c = _sensors.getTempC(_address);

    // Check for error conditions
    bool has_error =
        (temp_c == DEVICE_DISCONNECTED_C || temp_c == TEMP_ERROR_VALUE);

    if (has_error) {
        if (!_in_error_state) {
            _in_error_state = true;
            _logger.log("DS18B20 sensor error");
            _logger.updateLineText(_id, "ERROR");
        }
        return;
    }

    // Valid reading - exit error state if we were in it
    if (_in_error_state) {
        _in_error_state = false;
        _logger.log("DS18B20 recovered");
        char labelBuf[16];
        snprintf(labelBuf, sizeof(labelBuf), "%s:", _label);
        _logger.registerLine(_id, labelBuf, "C", temp_c);
    }

    _logger.updateLine(_id, temp_c);
    _last_temperature = temp_c;

    // Clear shared pending flag so next update cycle can request new conversion
    _shared_conversion_pending = false;
}
