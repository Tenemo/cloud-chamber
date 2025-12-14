/**
 * @file ThermalMetrics.cpp
 * @brief Implementation of unified thermal tracking (history + NVS persistence)
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
} // namespace

ThermalMetrics::ThermalMetrics(Logger &logger)
    : _logger(logger), _head(0), _count(0), _all_time_min_temp(100.0f),
      _all_time_optimal_current(0.0f), _total_runtime_seconds(0),
      _session_count(0), _session_min_temp(100.0f), _session_start_time(0),
      _last_metrics_save_time(0), _last_runtime_save_time(0),
      _last_temp_log_time(0) {}

void ThermalMetrics::begin() {
    _session_start_time = millis();
    _last_runtime_save_time = _session_start_time;
    loadFromNvs();
}

void ThermalMetrics::update() {
    unsigned long now = millis();

    // Update runtime counter
    if (now - _last_runtime_save_time >= NVS_RUNTIME_SAVE_INTERVAL_MS) {
        updateRuntime();
    }

    // Periodic metrics save
    if (now - _last_metrics_save_time >= NVS_METRICS_SAVE_INTERVAL_MS) {
        saveToNvs(false);
    }
}

// =============================================================================
// History Buffer Implementation
// =============================================================================

void ThermalMetrics::recordSample(const ThermalSample &sample) {
    _buffer[_head] = sample;
    _head = (_head + 1) % HISTORY_BUFFER_SIZE;
    if (_count < HISTORY_BUFFER_SIZE) {
        _count++;
    }

    updateSessionMin(sample.cold_plate_temp);
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

        unsigned long total_secs = now / 1000;
        unsigned int hours = (total_secs / 3600) % 24;
        unsigned int mins = (total_secs / 60) % 60;
        unsigned int secs = total_secs % 60;

        // Format rate string (handle insufficient history)
        char rate_str[16];
        if (rate <= RATE_INSUFFICIENT_HISTORY + 1.0f) {
            snprintf(rate_str, sizeof(rate_str), "--- %s", UNIT_RATE);
        } else {
            snprintf(rate_str, sizeof(rate_str), "%.2f %s", rate, UNIT_RATE);
        }

        // Use same labels as display, two spaces between values
        _logger.logf(true,
                     "[%02u:%02u:%02u]  %s: %.1f%s  %s: %.1f%s  "
                     "%s: %.1f%s  %s: %s  %s: %.2f%s",
                     hours, mins, secs, LABEL_COLD_PLATE, cold, UNIT_TEMP,
                     LABEL_HOT_PLATE, hot, UNIT_TEMP, LABEL_DELTA_T, delta,
                     UNIT_TEMP, LABEL_RATE, rate_str, LABEL_CURRENT,
                     set_current, UNIT_CURRENT);
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
    return _count >= min_samples;
}

const ThermalSample *ThermalMetrics::getSample(size_t samples_ago) const {
    if (samples_ago >= _count) {
        return nullptr;
    }
    size_t idx =
        (_head + HISTORY_BUFFER_SIZE - 1 - samples_ago) % HISTORY_BUFFER_SIZE;
    return &_buffer[idx];
}

float ThermalMetrics::calculateSlopeKPerMin(bool use_hot_plate,
                                            size_t window_samples) const {
    if (_count < window_samples) {
        return RATE_INSUFFICIENT_HISTORY;
    }

    // Linear regression: y = mx + b
    // We compute slope m using least squares
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    size_t n = window_samples;

    for (size_t i = 0; i < n; i++) {
        size_t idx =
            (_head + HISTORY_BUFFER_SIZE - n + i) % HISTORY_BUFFER_SIZE;
        float x = static_cast<float>(i);
        float y = use_hot_plate ? _buffer[idx].hot_plate_temp
                                : _buffer[idx].cold_plate_temp;

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
// NVS Persistence Implementation
// =============================================================================

void ThermalMetrics::recordNewMinimum(float temp, float current) {
    // Update session minimum (RAM-only, instant updates for display)
    updateSessionMin(temp);

    // Check if this beats the all-time record
    if (temp < _all_time_min_temp) {
        _all_time_min_temp = temp;
        _all_time_optimal_current = current;

        // Throttle NVS writes to protect flash (max once per
        // NVS_METRICS_SAVE_INTERVAL_MS) The values are already updated in RAM
        // for display purposes
        unsigned long now = millis();
        if (now - _last_metrics_save_time >= NVS_METRICS_SAVE_INTERVAL_MS) {
            _logger.logf(true, "NVS: New best=%.1fC@%.2fA", temp, current);
            saveToNvs(true);
        }
        // If throttled, the new best will be saved on next periodic save or
        // shutdown
    }
}

bool ThermalMetrics::checkNvsSpace() {
    nvs_stats_t nvs_stats;
    esp_err_t err = nvs_get_stats(NULL, &nvs_stats);

    if (err != ESP_OK) {
        _logger.log("NVS: Stats unavailable", true);
        return true; // Allow writes if we can't check
    }

    if (nvs_stats.free_entries < NVS_MIN_FREE_ENTRIES) {
        static unsigned long last_warning = 0;
        unsigned long now = millis();
        if (now - last_warning > 60000) {
            _logger.logf("NVS: Low space! %d free",
                         (int)nvs_stats.free_entries);
            last_warning = now;
        }
        return false;
    }

    return true;
}

void ThermalMetrics::loadFromNvs() {
    if (!_prefs.begin(NVS_NAMESPACE, true)) { // Read-only
        _logger.log("TC: NVS init (first run)");
        _prefs.end();

        // First run - initialize with defaults
        _all_time_min_temp = 100.0f;
        _all_time_optimal_current = 0.0f;
        _total_runtime_seconds = 0;
        _session_count = 1;

#if NVS_WRITES_ENABLED
        if (!checkNvsSpace()) {
            _logger.log("NVS: No space for init!");
            return;
        }

        // Write initial values
        if (_prefs.begin(NVS_NAMESPACE, false)) {
            bool ok = true;
            ok &= _prefs.putFloat(KEY_MIN_TEMP, _all_time_min_temp) > 0;
            ok &=
                _prefs.putFloat(KEY_OPT_CURRENT, _all_time_optimal_current) > 0;
            ok &= _prefs.putULong(KEY_RUNTIME, _total_runtime_seconds) > 0;
            ok &= _prefs.putULong(KEY_SESSIONS, _session_count) > 0;
            _prefs.end();

            if (!ok) {
                _logger.log("NVS: Init write failed!");
            }
        }
#endif
        return;
    }

    // Load existing values
    _all_time_min_temp = _prefs.getFloat(KEY_MIN_TEMP, 100.0f);
    _all_time_optimal_current = _prefs.getFloat(KEY_OPT_CURRENT, 0.0f);
    _total_runtime_seconds = _prefs.getULong(KEY_RUNTIME, 0);
    _session_count = _prefs.getULong(KEY_SESSIONS, 0);
    _prefs.end();

    // Increment session count
    _session_count++;

#if NVS_WRITES_ENABLED
    if (checkNvsSpace() && _prefs.begin(NVS_NAMESPACE, false)) {
        _prefs.putULong(KEY_SESSIONS, _session_count);
        _prefs.end();
    }
#endif

    // Log loaded values
    _logger.logf("TC: Best=%.1fC@%.2fA", _all_time_min_temp,
                 _all_time_optimal_current);

    unsigned long hours = _total_runtime_seconds / 3600;
    unsigned long mins = (_total_runtime_seconds % 3600) / 60;
    _logger.logf("TC: Runtime=%luh%lum #%lu", hours, mins, _session_count);
}

void ThermalMetrics::saveToNvs(bool force) {
#if !NVS_WRITES_ENABLED
    (void)force; // Suppress unused parameter warning
    return;
#else
    unsigned long now = millis();

    if (!force &&
        now - _last_metrics_save_time < NVS_METRICS_SAVE_INTERVAL_MS) {
        return;
    }

    if (!checkNvsSpace()) {
        return;
    }

    if (!_prefs.begin(NVS_NAMESPACE, false)) {
        _logger.log("NVS: Failed to open for save", true);
        return;
    }

    // Only write changed values
    float stored_min = _prefs.getFloat(KEY_MIN_TEMP, 100.0f);
    if (fabs(_all_time_min_temp - stored_min) > 0.01f) {
        _prefs.putFloat(KEY_MIN_TEMP, _all_time_min_temp);
        _prefs.putFloat(KEY_OPT_CURRENT, _all_time_optimal_current);
    }

    _prefs.end();
    _last_metrics_save_time = now;
#endif
}

void ThermalMetrics::updateRuntime() {
    unsigned long now = millis();

    unsigned long elapsed_ms = now - _last_runtime_save_time;
    if (_last_runtime_save_time == 0) {
        elapsed_ms = now - _session_start_time;
    }

    _total_runtime_seconds += elapsed_ms / 1000;
    _last_runtime_save_time = now;

#if NVS_WRITES_ENABLED
    if (!checkNvsSpace()) {
        return;
    }

    if (_prefs.begin(NVS_NAMESPACE, false)) {
        _prefs.putULong(KEY_RUNTIME, _total_runtime_seconds);
        _prefs.end();
    }
#endif
}

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
}
