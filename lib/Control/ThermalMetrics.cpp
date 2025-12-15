/**
 * @file ThermalMetrics.cpp
 * @brief Implementation of unified thermal tracking (history buffer only)
 */

#include "ThermalMetrics.h"
#include "DualPowerSupply.h"
#include "SafetyMonitor.h"
#include "TemperatureSensors.h"
#include "ThermalOptimizer.h"
#include <cmath>

namespace {
constexpr const char *LINE_STATE = "TC_STATE";
constexpr const char *LINE_RATE = "TC_RATE";
constexpr const char *LINE_CURRENT = "TC_I";
constexpr const char *LINE_VOLTAGE = "TC_V";
constexpr const char *LINE_POWER = "TC_P";
} // namespace

ThermalMetrics::ThermalMetrics(Logger &logger)
    : _logger(logger), _history(), _last_avg_voltage(0.0f),
      _last_avg_power(0.0f), _last_temp_log_time(0) {}

void ThermalMetrics::begin() {
    // Nothing to initialize - history buffer is ready
}

void ThermalMetrics::update() {
    // Nothing to persist - history buffer updates happen in recordSample
}

// =============================================================================
// History Buffer Implementation
// =============================================================================

void ThermalMetrics::recordSample(const ThermalSample &sample) {
    _history.push(sample);
}

void ThermalMetrics::recordSample(TemperatureSensors &sensors,
                                  DualPowerSupply &dps) {
    ThermalSample sample;
    sample.cold_plate_temp = sensors.getColdPlateTemperature();
    sample.hot_plate_temp = sensors.getHotPlateTemperature();
    sample.set_current = dps.getTargetCurrent();
    sample.voltage[0] = dps.getOutputVoltage(0);
    sample.voltage[1] = dps.getOutputVoltage(1);
    sample.current[0] = dps.getOutputCurrent(0);
    sample.current[1] = dps.getOutputCurrent(1);
    sample.power[0] = dps.getOutputPower(0);
    sample.power[1] = dps.getOutputPower(1);
    sample.timestamp = millis();

    recordSample(sample);

    // Cache averages for display/logging
    float voltage_sum = 0.0f;
    float power_sum = 0.0f;
    int connected = 0;
    for (size_t ch = 0; ch < 2; ch++) {
        if (dps.isConnected(ch)) {
            voltage_sum += sample.voltage[ch];
            power_sum += sample.power[ch];
            connected++;
        }
    }
    _last_avg_voltage = (connected > 0) ? (voltage_sum / connected) : 0.0f;
    _last_avg_power = (connected > 0) ? (power_sum / connected) : 0.0f;

    dps.checkAndLogImbalance(CHANNEL_CURRENT_IMBALANCE_A,
                             CHANNEL_POWER_IMBALANCE_W,
                             Timing::IMBALANCE_LOG_INTERVAL_MS);

    // Periodic comprehensive temperature log (serial only)
    unsigned long now = millis();
    if (now - _last_temp_log_time >= TEMP_LOG_INTERVAL_MS) {
        _last_temp_log_time = now;

        float cold = sample.cold_plate_temp;
        float hot = sample.hot_plate_temp;
        float delta = hot - cold;
        float rate = getColdPlateRate();
        float set_current = sample.set_current;
        float avg_voltage = _last_avg_voltage;
        float avg_power = _last_avg_power;

        // Format rate string (handle insufficient history)
        char rate_str[16];
        if (rate <= RATE_INSUFFICIENT_HISTORY + 1.0f) {
            snprintf(rate_str, sizeof(rate_str), "--- %s", UNIT_RATE);
        } else {
            snprintf(rate_str, sizeof(rate_str), "%.2f %s", rate, UNIT_RATE);
        }

        // Use same labels as display, two spaces between values
        // Timestamp is added by Logger automatically
        _logger.logf(true,
                     "%s: %.1f%s  %s: %.1f%s  %s: %.1f%s  %s: %s  %s: %.2f%s  "
                     "%s: %.2f%s  %s: %.1f%s",
                     LABEL_COLD_PLATE, cold, UNIT_TEMP, LABEL_HOT_PLATE, hot,
                     UNIT_TEMP, LABEL_DELTA_T, delta, UNIT_TEMP, LABEL_RATE,
                     rate_str, LABEL_CURRENT, set_current, UNIT_CURRENT,
                     "V", avg_voltage, "V", "P", avg_power, UNIT_POWER);
    }
}

ThermalSnapshot ThermalMetrics::buildSnapshot(TemperatureSensors &sensors,
                                              DualPowerSupply &dps,
                                              SafetyMonitor &safety) const {
    return ThermalSnapshot{
        sensors.getColdPlateTemperature(),
        sensors.getHotPlateTemperature(),
        getColdPlateRate(),
        getHotPlateRate(),
        dps.getTargetCurrent(),
        safety.isHotSideWarning(),
        safety.isHotSideAlarm(),
        isHotSideStable(HOT_SIDE_STABLE_RATE_THRESHOLD_C_PER_MIN),
        millis()};
}

bool ThermalMetrics::hasMinimumHistory(size_t min_samples) const {
    return _history.size() >= min_samples;
}

float ThermalMetrics::calculateSlopeKPerMin(bool use_hot_plate,
                                            size_t window_samples) const {
    if (_history.size() < window_samples) {
        return RATE_INSUFFICIENT_HISTORY;
    }

    // Linear regression: y = mx + b
    // We compute slope m using least squares
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    size_t n = window_samples;

    for (size_t i = 0; i < n; i++) {
        const ThermalSample *s = _history.getFromNewest(n - 1 - i);
        if (!s)
            return RATE_INSUFFICIENT_HISTORY;
        float x = static_cast<float>(i);
        float y = use_hot_plate ? s->hot_plate_temp : s->cold_plate_temp;

        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }

    float denom = n * sum_xx - sum_x * sum_x;
    if (fabs(denom) < 0.0001f) {
        return 0.0f;
    }

    // Slope in degrees per sample
    float slope = (n * sum_xy - sum_x * sum_y) / denom;

    // Convert to K/min based on actual sample interval
    // slope is in degrees/sample, multiply by samples/minute
    float samples_per_minute = 60000.0f / HISTORY_SAMPLE_INTERVAL_MS;
    return slope * samples_per_minute;
}

float ThermalMetrics::getColdPlateRate() const {
    return calculateSlopeKPerMin(false, COOLING_RATE_WINDOW_SAMPLES);
}

float ThermalMetrics::getHotPlateRate() const {
    return calculateSlopeKPerMin(true, HOT_SIDE_RATE_SAMPLE_WINDOW_SAMPLES);
}

bool ThermalMetrics::isHotSideStable(float max_rate_k_per_min,
                                     size_t min_samples) const {
    if (!hasMinimumHistory(min_samples))
        return false;

    float rate = getHotPlateRate();
    if (rate == RATE_INSUFFICIENT_HISTORY)
        return false;

    return fabs(rate) <= max_rate_k_per_min;
}

// =============================================================================
// Display Integration
// =============================================================================

void ThermalMetrics::registerDisplayLines() {
    _logger.registerTextLine(LINE_STATE, "State:", "INIT");

    // Build label with colon for display
    char rate_label[16];
    snprintf(rate_label, sizeof(rate_label), "%s:", LABEL_RATE);
    char rate_default[16];
    snprintf(rate_default, sizeof(rate_default), "--- %s", UNIT_RATE);
    _logger.registerTextLine(LINE_RATE, rate_label, rate_default);

    char current_label[16];
    snprintf(current_label, sizeof(current_label), "%s:", LABEL_CURRENT);
    char current_default[16];
    snprintf(current_default, sizeof(current_default), "0.0 %s", UNIT_CURRENT);
    _logger.registerTextLine(LINE_CURRENT, current_label, current_default);

    char voltage_label[12];
    snprintf(voltage_label, sizeof(voltage_label), "V:");
    char voltage_default[12];
    snprintf(voltage_default, sizeof(voltage_default), "0.0 V");
    _logger.registerTextLine(LINE_VOLTAGE, voltage_label, voltage_default);

    char power_label[12];
    snprintf(power_label, sizeof(power_label), "P:");
    char power_default[12];
    snprintf(power_default, sizeof(power_default), "0.0 W");
    _logger.registerTextLine(LINE_POWER, power_label, power_default);
}

void ThermalMetrics::updateDisplay(const char *state_string,
                                   float target_current) {
    _logger.updateLineText(LINE_STATE, state_string);

    // Format rate as text to avoid mixed updateLine/updateLineText issues
    float rate = getColdPlateRate();
    char rate_buf[16];
    if (rate <= RATE_INSUFFICIENT_HISTORY + 1.0f) {
        snprintf(rate_buf, sizeof(rate_buf), "--- %s", UNIT_RATE);
    } else {
        snprintf(rate_buf, sizeof(rate_buf), "%.2f %s", rate, UNIT_RATE);
    }
    _logger.updateLineText(LINE_RATE, rate_buf);

    // Format current as text
    char current_buf[16];
    snprintf(current_buf, sizeof(current_buf), "%.2f %s", target_current,
             UNIT_CURRENT);
    _logger.updateLineText(LINE_CURRENT, current_buf);

    // Voltage and power averages
    char voltage_buf[16];
    snprintf(voltage_buf, sizeof(voltage_buf), "%.2f V", _last_avg_voltage);
    _logger.updateLineText(LINE_VOLTAGE, voltage_buf);

    char power_buf[16];
    snprintf(power_buf, sizeof(power_buf), "%.1f %s", _last_avg_power,
             UNIT_POWER);
    _logger.updateLineText(LINE_POWER, power_buf);
}
