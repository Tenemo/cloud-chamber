/**
 * @file TemperatureSensors.cpp
 * @brief Implementation of unified temperature sensor manager
 */

#include "TemperatureSensors.h"
#include "config.h"

TemperatureSensors::TemperatureSensors(Logger &logger)
    : _logger(logger), _oneWire(PIN_DS18B20), _dallasSensors(&_oneWire),
      _pt100(logger, "Cold plate"),
      _hotPlate(logger, _dallasSensors, DS18B20_1_ADDRESS, "Hot plate"),
      _glassTop(logger, _dallasSensors, DS18B20_2_ADDRESS, "Glass top"),
      _internal(logger, _dallasSensors, DS18B20_3_ADDRESS, "Internal") {}

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
}
