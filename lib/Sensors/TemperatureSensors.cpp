/**
 * @file TemperatureSensors.cpp
 * @brief Implementation of unified temperature sensor manager
 */

#include "TemperatureSensors.h"
#include "config.h"

TemperatureSensors::TemperatureSensors(Logger &logger)
    : _logger(logger), _oneWire(PIN_DS18B20), _dallasSensors(&_oneWire),
      _pt100(logger, "Cold plate"),
      _hotPlate(logger, _dallasSensors, DS18B20_1_ADDRESS, "Hot plate") {}

void TemperatureSensors::begin() {
    _dallasSensors.begin();
    _dallasSensors.setWaitForConversion(false);
    _hotPlate.begin();
    _pt100.begin();

    // Register ΔT display line (shows temperature difference hot - cold)
    // \delta is a special marker that triggers custom delta character rendering
    _logger.registerLine("deltaT", "\\deltaT", "C", 0.0f);

    _logger.log("Sensors initialized.");
}

void TemperatureSensors::update() {
    _pt100.update();
    _hotPlate.update();

    // Update ΔT display (hot - cold)
    _logger.updateLine("deltaT", getDeltaT());
}
