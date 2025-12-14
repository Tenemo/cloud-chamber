/**
 * @file TemperatureSensors.cpp
 * @brief Implementation of unified temperature sensor manager
 */

#include "TemperatureSensors.h"
#include "config.h"

// Periodic comprehensive temperature log interval (serial only)
static constexpr unsigned long TEMP_LOG_INTERVAL_MS = 10000; // 10 seconds

TemperatureSensors::TemperatureSensors(Logger &logger)
    : _logger(logger), _oneWire(PIN_DS18B20), _dallasSensors(&_oneWire),
      _pt100(logger, "Cold plate"),
      _hotPlate(logger, _dallasSensors, DS18B20_1_ADDRESS, "Hot plate"),
      _glassTop(logger, _dallasSensors, DS18B20_2_ADDRESS, "External"),
      _internal(logger, _dallasSensors, DS18B20_3_ADDRESS, "Internal"),
      _last_temp_log_time(0) {}

void TemperatureSensors::begin() {
    _dallasSensors.begin();
    _dallasSensors.setWaitForConversion(false);

    _internal.begin();
    _glassTop.begin();

    _hotPlate.begin();
    _pt100.begin();
    _logger.registerLine("deltaT", "\\deltaT", "C", 0.0f);

    _logger.log("Sensors initialized.");
}

void TemperatureSensors::update() {
    _internal.update();
    _glassTop.update();
    _hotPlate.update();
    _pt100.update();

    // Update ΔT display (hot - cold)
    _logger.updateLine("deltaT", getDeltaT());

    // Periodic comprehensive temperature log (serial only)
    unsigned long now = millis();
    if (now - _last_temp_log_time >= TEMP_LOG_INTERVAL_MS) {
        _last_temp_log_time = now;

        float cold = getColdPlateTemperature();
        float hot = getHotPlateTemperature();
        float delta = hot - cold;

        unsigned long total_secs = now / 1000;
        unsigned int hours = (total_secs / 3600) % 24;
        unsigned int mins = (total_secs / 60) % 60;
        unsigned int secs = total_secs % 60;

        _logger.logf(true, "[%02u:%02u:%02u] Cold:%.1fC Hot:%.1fC dT:%.1fC",
                     hours, mins, secs, cold, hot, delta);
    }
}
