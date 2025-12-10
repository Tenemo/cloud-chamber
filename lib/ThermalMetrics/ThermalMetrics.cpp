/**
 * @file ThermalMetrics.cpp
 * @brief Implementation of unified thermal tracking (history + NVS persistence)
 */

#include "ThermalMetrics.h"
#include <cmath>

ThermalMetrics::ThermalMetrics(Logger &logger)
    : _logger(logger), _head(0), _count(0), _all_time_min_temp(100.0f),
      _all_time_optimal_current(0.0f), _total_runtime_seconds(0),
      _session_count(0), _session_start_time(0), _last_metrics_save_time(0),
      _last_runtime_save_time(0) {}

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

void ThermalMetrics::clearHistory() {
    _head = 0;
    _count = 0;
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
    if (temp < _all_time_min_temp) {
        _all_time_min_temp = temp;
        _all_time_optimal_current = current;

        _logger.logf(true, "NVS: New best=%.1fC@%.1fA", temp, current);

        saveToNvs(true); // Force save for new records
    }
}

void ThermalMetrics::forceSave() { saveToNvs(true); }

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

    if (checkNvsSpace() && _prefs.begin(NVS_NAMESPACE, false)) {
        _prefs.putULong(KEY_SESSIONS, _session_count);
        _prefs.end();
    }

    // Log loaded values
    _logger.logf("TC: Best=%.1fC@%.1fA", _all_time_min_temp,
                 _all_time_optimal_current);

    unsigned long hours = _total_runtime_seconds / 3600;
    unsigned long mins = (_total_runtime_seconds % 3600) / 60;
    _logger.logf("TC: Runtime=%luh%lum #%lu", hours, mins, _session_count);
}

void ThermalMetrics::saveToNvs(bool force) {
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
}

void ThermalMetrics::updateRuntime() {
    unsigned long now = millis();

    unsigned long elapsed_ms = now - _last_runtime_save_time;
    if (_last_runtime_save_time == 0) {
        elapsed_ms = now - _session_start_time;
    }

    _total_runtime_seconds += elapsed_ms / 1000;
    _last_runtime_save_time = now;

    if (!checkNvsSpace()) {
        return;
    }

    if (_prefs.begin(NVS_NAMESPACE, false)) {
        _prefs.putULong(KEY_RUNTIME, _total_runtime_seconds);
        _prefs.end();
    }
}
