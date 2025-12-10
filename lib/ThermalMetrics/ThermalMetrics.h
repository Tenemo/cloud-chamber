/**
 * @file ThermalMetrics.h
 * @brief NVS persistence for thermal control metrics
 *
 * Manages persistent storage of key metrics for long-term diagnostics:
 * - All-time minimum temperature achieved
 * - Optimal current that achieved that temperature
 * - Total accumulated runtime
 * - Session (boot) count
 */

#ifndef THERMAL_METRICS_H
#define THERMAL_METRICS_H

#include "Logger.h"
#include "config.h"
#include <Arduino.h>
#include <Preferences.h>
#include <nvs.h>

/**
 * @brief NVS-backed metrics persistence
 *
 * Provides rate-limited writes to NVS to avoid flash wear while
 * maintaining important diagnostic data across power cycles.
 */
class ThermalMetrics {
  public:
    ThermalMetrics(Logger &logger);

    /**
     * @brief Initialize and load metrics from NVS
     * Call once during setup.
     */
    void begin();

    /**
     * @brief Periodic update - saves metrics if interval elapsed
     * Call frequently from main loop.
     */
    void update();

    /**
     * @brief Record a new minimum temperature achievement
     * @param temp The new minimum temperature
     * @param current The current that achieved it
     *
     * Forces an immediate NVS save if this is a new all-time best.
     */
    void recordNewMinimum(float temp, float current);

    /**
     * @brief Force save all metrics to NVS
     */
    void forceSave();

    // Getters
    float getAllTimeMinTemp() const { return _all_time_min_temp; }
    float getAllTimeOptimalCurrent() const { return _all_time_optimal_current; }
    unsigned long getTotalRuntimeSeconds() const {
        return _total_runtime_seconds;
    }
    uint32_t getSessionCount() const { return _session_count; }

  private:
    Logger &_logger;
    Preferences _prefs;

    // Persisted values
    float _all_time_min_temp;
    float _all_time_optimal_current;
    unsigned long _total_runtime_seconds;
    uint32_t _session_count;

    // Timing
    unsigned long _session_start_time;
    unsigned long _last_metrics_save_time;
    unsigned long _last_runtime_save_time;

    // NVS namespace and keys
    static constexpr const char *NVS_NAMESPACE = "tc_metrics";
    static constexpr const char *KEY_MIN_TEMP = "min_t";
    static constexpr const char *KEY_OPT_CURRENT = "opt_i";
    static constexpr const char *KEY_RUNTIME = "runtime";
    static constexpr const char *KEY_SESSIONS = "sessions";

    // NVS space management
    static constexpr size_t NVS_MIN_FREE_ENTRIES = 10;

    /**
     * @brief Check if NVS has enough free space for writes
     */
    bool checkNvsSpace();

    /**
     * @brief Load all metrics from NVS
     */
    void loadFromNvs();

    /**
     * @brief Save metrics to NVS (respects rate limiting unless forced)
     */
    void saveToNvs(bool force);

    /**
     * @brief Update and save runtime counter
     */
    void updateRuntime();
};

#endif // THERMAL_METRICS_H
