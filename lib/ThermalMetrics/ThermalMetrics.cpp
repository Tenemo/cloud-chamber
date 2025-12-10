/**
 * @file ThermalMetrics.cpp
 * @brief Implementation of NVS metrics persistence
 */

#include "ThermalMetrics.h"
#include <cmath>

ThermalMetrics::ThermalMetrics(Logger &logger)
    : _logger(logger), _all_time_min_temp(100.0f),
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

void ThermalMetrics::recordNewMinimum(float temp, float current) {
    if (temp < _all_time_min_temp) {
        _all_time_min_temp = temp;
        _all_time_optimal_current = current;

        char buf[48];
        snprintf(buf, sizeof(buf), "NVS: New best=%.1fC@%.1fA", temp, current);
        _logger.log(buf, true);

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
            char buf[48];
            snprintf(buf, sizeof(buf), "NVS: Low space! %d free",
                     (int)nvs_stats.free_entries);
            _logger.log(buf);
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
    char buf[64];
    snprintf(buf, sizeof(buf), "TC: Best=%.1fC@%.1fA", _all_time_min_temp,
             _all_time_optimal_current);
    _logger.log(buf);

    unsigned long hours = _total_runtime_seconds / 3600;
    unsigned long mins = (_total_runtime_seconds % 3600) / 60;
    snprintf(buf, sizeof(buf), "TC: Runtime=%luh%lum #%lu", hours, mins,
             _session_count);
    _logger.log(buf);
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
