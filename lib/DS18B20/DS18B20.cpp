#include "DS18B20.h"
#include "config.h"

DS18B20Sensor::DS18B20Sensor(Logger &logger)
    : _logger(logger), _oneWire(nullptr), _sensors(nullptr),
      _last_temperature(0.0f), _initialized(false), _in_error_state(false),
      _last_update_time(0), _conversion_start_time(0),
      _conversion_pending(false) {}

void DS18B20Sensor::begin() {
    if (_initialized)
        return; // Prevent re-initialization

    _oneWire = new OneWire(PIN_DS18B20);
    _sensors = new DallasTemperature(_oneWire);
    _sensors->begin();

    // Set resolution to 12-bit for maximum precision
    _sensors->setResolution(12);

    // Check if sensor is connected
    int deviceCount = _sensors->getDeviceCount();
    if (deviceCount == 0) {
        _in_error_state = true;
        _logger.registerTextLine("ds18b20", "DS18B20:", "ERROR");
        _logger.log("DS18B20 not found");
    } else {
        // Request initial temperature
        _sensors->requestTemperatures();
        delay(100);
        float temp_c = _sensors->getTempCByIndex(0);

        if (temp_c == TEMP_ERROR_VALUE) {
            _in_error_state = true;
            _logger.registerTextLine("ds18b20", "DS18B20:", "ERROR");
            _logger.log("DS18B20 read error");
        } else {
            _logger.registerLine("ds18b20", "DS18B20:", "C", temp_c);
            _logger.log("DS18B20 initialized.");
            _last_temperature = temp_c;
        }
    }

    _initialized = true;
}

void DS18B20Sensor::update() {
    if (!_initialized || !_sensors)
        return;

    unsigned long current_time = millis();

    // Non-blocking temperature reading using async conversion
    if (!_conversion_pending) {
        if (current_time - _last_update_time < SENSOR_UPDATE_INTERVAL_MS) {
            return;
        }
        // Start async temperature conversion
        _sensors->requestTemperatures();
        _conversion_start_time = current_time;
        _conversion_pending = true;
        return;
    }

    // Check if conversion is complete
    if (current_time - _conversion_start_time < CONVERSION_DELAY_MS) {
        return;
    }

    _conversion_pending = false;
    _last_update_time = current_time;

    float temp_c = _sensors->getTempCByIndex(0);

    // Check for error conditions
    bool has_error = (temp_c == TEMP_ERROR_VALUE);

    if (has_error) {
        if (!_in_error_state) {
            _in_error_state = true;
            _logger.log("DS18B20 sensor error");
            _logger.updateLineText("ds18b20", "ERROR");
        }
        return;
    }

    // Valid reading - exit error state if we were in it
    if (_in_error_state) {
        _in_error_state = false;
        _logger.log("DS18B20 recovered");
        _logger.registerLine("ds18b20", "DS18B20:", "C", temp_c);
    }

    _logger.updateLine("ds18b20", temp_c);
    _last_temperature = temp_c;
}
