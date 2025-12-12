#include "DS18B20.h"
#include "config.h"

// Static member definitions for shared bus coordination
unsigned long DS18B20Sensor::_shared_conversion_start_time = 0;
bool DS18B20Sensor::_shared_conversion_pending = false;
uint32_t DS18B20Sensor::_shared_conversion_id = 0;

DS18B20Sensor::DS18B20Sensor(Logger &logger, DallasTemperature &sensors,
                             const uint8_t *address, const char *label)
    : _logger(logger), _sensors(sensors), _label(label),
      _last_temperature(0.0f), _initialized(false), _ever_connected(false),
      _in_error_state(false), _reconnect_pending(false),
      _reconnect_start_time(0), _last_update_time(0),
      _last_read_conversion_id(0) {
    memcpy(_address, address, 8);
}

void DS18B20Sensor::begin() {
    if (_initialized)
        return; // prevent re-initialization

    // Format label once and cache it
    Logger::formatLabel(_formatted_label, sizeof(_formatted_label), _label);

    // Set resolution to 12-bit for maximum precision
    _sensors.setResolution(_address, 12);

    // Request initial temperature using specific address
    _sensors.requestTemperaturesByAddress(_address);
    delay(DS18B20_CONVERSION_TIME_MS); // wait for 12-bit conversion
    float temp_c = _sensors.getTempC(_address);

    if (temp_c == DEVICE_DISCONNECTED_C || temp_c == TEMP_ERROR_VALUE) {
        // Sensor not found - don't register display line, just log
        _logger.log("DS18B20 not found");
    } else {
        _ever_connected = true;
        _logger.registerLine(_label, _formatted_label, "C", temp_c);
        _logger.log("DS18B20 initialized.");
        _last_temperature = temp_c;
    }

    _initialized = true;
}

void DS18B20Sensor::update() {
    if (!_initialized)
        return;

    // Skip update if sensor was never successfully connected
    if (!_ever_connected) {
        unsigned long current_time = millis();

        // If reconnection conversion is pending, check if it's complete
        if (_reconnect_pending) {
            if (current_time - _reconnect_start_time <
                DS18B20_CONVERSION_TIME_MS) {
                return; // Still converting, don't block
            }
            // Conversion complete, read result
            _reconnect_pending = false;
            float temp_c = _sensors.getTempC(_address);

            if (temp_c != DEVICE_DISCONNECTED_C && temp_c != TEMP_ERROR_VALUE) {
                _ever_connected = true;
                _logger.registerLine(_label, _formatted_label, "C", temp_c);
                _logger.log("DS18B20 connected.");
                _last_temperature = temp_c;
            }
            _last_update_time = current_time;
            return;
        }

        // Periodically try to detect the sensor (non-blocking)
        if (current_time - _last_update_time < DS18B20_UPDATE_INTERVAL_MS) {
            return;
        }

        // Start a reconnection conversion
        _sensors.requestTemperaturesByAddress(_address);
        _reconnect_start_time = current_time;
        _reconnect_pending = true;
        return;
    }

    unsigned long current_time = millis();

    // If no conversion is pending or enough time has passed, start a new
    // conversion
    if (!_shared_conversion_pending) {
        // Check if enough time has passed since last conversion
        if (current_time - _shared_conversion_start_time <
            DS18B20_UPDATE_INTERVAL_MS) {
            // Not time for a new conversion yet, but we can still read
            // (conversion data is still valid)
        } else {
            // Time for a new conversion - request all sensors at once
            _sensors.requestTemperatures();
            _shared_conversion_start_time = current_time;
            _shared_conversion_pending = true;
            _shared_conversion_id++; // New conversion cycle
            return;
        }
    }

    // If conversion is pending, wait for it to complete
    if (_shared_conversion_pending) {
        if (current_time - _shared_conversion_start_time <
            DS18B20_CONVERSION_TIME_MS) {
            return; // Still converting
        }
        // Conversion complete - clear the flag
        _shared_conversion_pending = false;
    }

    // Check if this sensor has already read from THIS conversion cycle
    // Using conversion ID prevents reading stale data from previous cycles
    if (_last_read_conversion_id >= _shared_conversion_id) {
        return; // Already read this conversion cycle
    }

    // Read this sensor's temperature
    _last_update_time = current_time;
    _last_read_conversion_id = _shared_conversion_id; // Mark this cycle as read

    float temp_c = _sensors.getTempC(_address);

    // Check for error conditions
    bool has_error =
        (temp_c == DEVICE_DISCONNECTED_C || temp_c == TEMP_ERROR_VALUE);

    if (has_error) {
        if (!_in_error_state) {
            _in_error_state = true;
            _logger.log("DS18B20 sensor error");
            _logger.updateLineText(_label, "ERROR");
        }
        return;
    }

    // Valid reading - exit error state if we were in it
    if (_in_error_state) {
        _in_error_state = false;
        _logger.log("DS18B20 recovered");
        // Re-register as numeric line (use cached formatted label)
        _logger.registerLine(_label, _formatted_label, "C", temp_c);
    }

    _logger.updateLine(_label, temp_c);
    _last_temperature = temp_c;
}
