/**
 * @file ThermalMetrics.h
 * @brief Thermal history buffer, trend analysis, and NVS persistence
 *
 * This class handles data tracking and persistence:
 * 1. **Runtime History**: Circular buffer of temperature samples for trend
 *    analysis and cooling rate calculations (volatile, lost on reset)
 * 2. **Trend Analysis**: Linear regression for cooling/heating rate
 * 3. **Persistent Metrics**: NVS storage for long-term tracking of best
 *    temperatures, optimal currents, and total runtime across sessions
 *
 * HISTORY BUFFER:
 * ---------------
 * - 300 samples at 1-second intervals = 5 minutes of history
 * - Linear regression for cooling rate calculation (K/min)
 * - Separate windows for cold plate (30 samples) and hot plate (60 samples)
 *
 * NVS PERSISTENCE:
 * ----------------
 * - All-time minimum temperature and corresponding current
 * - Total runtime across all sessions (hours)
 * - Session count
 * - Periodic auto-save with space checking
 *
 * NOTE: Optimization logic (hill-climbing, step sizing) is in ThermalOptimizer.
 */

#ifndef THERMAL_METRICS_H
#define THERMAL_METRICS_H

#include "Logger.h"
#include "ThermalConstants.h"
#include <Preferences.h>
#include <nvs.h>

// Forward declarations
class TemperatureSensors;
class DualPowerSupply;
class SafetyMonitor;

// Special return value when insufficient history for rate calculation
constexpr float RATE_INSUFFICIENT_HISTORY = -999.0f;

/**
 * @brief Single sample with temperatures and power data
 *
 * Records both plate temperatures plus DPS current/power for complete
 * historical analysis. This allows the optimizer to correlate temperature
 * trends with power consumption and detect efficiency changes.
 */
struct ThermalSample {
    float cold_plate_temp;   // Cold plate (PT100) in Celsius
    float hot_plate_temp;    // Hot plate (DS18B20) in Celsius
    float set_current;       // Commanded current per channel (A)
    float voltage[2];        // Output voltage per channel (V)
    float current[2];        // Actual output current per channel (A)
    float power[2];          // Output power per channel (W)
    unsigned long timestamp; // millis() when sample was taken
};

// Forward declaration from ThermalOptimizer.h
struct ThermalSnapshot;

/**
 * @brief Thermal history and metrics tracker
 */
class ThermalMetrics {
  public:
    explicit ThermalMetrics(Logger &logger);

    /**
     * @brief Initialize metrics system (load from NVS, increment session count)
     */
    void begin();

    /**
     * @brief Periodic update (save runtime, check metrics interval)
     */
    void update();

    // =========================================================================
    // History Buffer Interface
    // =========================================================================

    /**
     * @brief Add a temperature sample to the circular buffer
     */
    void recordSample(const ThermalSample &sample);

    /**
     * @brief Record sample from sensors and PSU (convenience overload)
     *
     * Builds a ThermalSample from the hardware references and records it.
     * Also checks for channel imbalance.
     *
     * @param sensors Temperature sensors reference
     * @param dps Dual power supply reference
     */
    void recordSample(TemperatureSensors &sensors, DualPowerSupply &dps);

    /**
     * @brief Build a snapshot of current thermal state for optimizer
     *
     * @param sensors Temperature sensors reference
     * @param dps Dual power supply reference
     * @param safety Safety monitor reference
     * @return ThermalSnapshot with current state
     */
    ThermalSnapshot buildSnapshot(TemperatureSensors &sensors,
                                  DualPowerSupply &dps,
                                  SafetyMonitor &safety) const;

    /**
     * @brief Check if minimum sample count is available
     */
    bool hasMinimumHistory(size_t min_samples) const;

    /**
     * @brief Get a sample from history (0 = most recent)
     * @return Pointer to sample, or nullptr if out of range
     */
    const ThermalSample *getSample(size_t samples_ago) const;

    /**
     * @brief Get cold plate cooling rate in K/min (negative = cooling)
     * @return Rate, or RATE_INSUFFICIENT_HISTORY if not enough data
     */
    float getColdPlateRate() const;

    /**
     * @brief Get hot plate temperature rate in K/min
     * @return Rate, or RATE_INSUFFICIENT_HISTORY if not enough data
     */
    float getHotPlateRate() const;

    /**
     * @brief Check if hot side temperature is stable
     *
     * Hot side is considered stable when its rate of change is below
     * the threshold and we have enough history to make that determination.
     *
     * @param max_rate_k_per_min Maximum acceptable rate (K/min)
     * @param min_samples Minimum history required for valid check
     * @return true if stable, false if unstable or insufficient history
     */
    bool isHotSideStable(float max_rate_k_per_min,
                         size_t min_samples = 60) const;

    // =========================================================================
    // NVS Persistence Interface
    // =========================================================================

    /**
     * @brief Record a new minimum temperature (saves immediately if better)
     */
    void recordNewMinimum(float temp, float current);

    // =========================================================================
    // Accessors
    // =========================================================================

    float getAllTimeMinTemp() const { return _all_time_min_temp; }
    float getAllTimeOptimalCurrent() const { return _all_time_optimal_current; }
    unsigned long getTotalRuntimeSeconds() const {
        return _total_runtime_seconds;
    }
    unsigned long getSessionCount() const { return _session_count; }

    /**
     * @brief Get session minimum temperature (reset each boot)
     *
     * This tracks the coldest temperature achieved during the current session,
     * distinct from the all-time minimum which persists to NVS.
     */
    float getSessionMinTemp() const { return _session_min_temp; }

    /**
     * @brief Update session minimum if new temp is lower
     */
    void updateSessionMin(float temp) {
        if (temp < _session_min_temp) {
            _session_min_temp = temp;
        }
    }

    /**
     * @brief Check if it's time to perform an adjustment
     * @param last_adjustment_time Time of last adjustment
     * @param interval_ms Required interval between adjustments
     * @return true if enough time has passed
     */
    bool isTimeForAdjustment(unsigned long last_adjustment_time,
                             unsigned long interval_ms) const {
        return (millis() - last_adjustment_time) >= interval_ms;
    }

    /**
     * @brief Register display lines for thermal metrics
     */
    void registerDisplayLines();

    /**
     * @brief Update display with current state and PSU info
     *
     * @param state_string Current state string (from stateToString)
     * @param target_current Current target current from DPS
     */
    void updateDisplay(const char *state_string, float target_current);

  private:
    Logger &_logger;

    // -------------------------------------------------------------------------
    // History buffer (circular)
    // -------------------------------------------------------------------------
    ThermalSample _buffer[HISTORY_BUFFER_SIZE];
    size_t _head;  // Next write position
    size_t _count; // Number of valid samples (up to HISTORY_BUFFER_SIZE)

    /**
     * @brief Calculate temperature slope using linear regression
     * @param use_hot_plate If true, use hot plate temps; else cold plate
     * @param window_samples Number of samples to use for calculation
     * @return Slope in K/min, or RATE_INSUFFICIENT_HISTORY
     */
    float calculateSlopeKPerMin(bool use_hot_plate,
                                size_t window_samples) const;

    // -------------------------------------------------------------------------
    // NVS persistence state
    // -------------------------------------------------------------------------
    Preferences _prefs;
    float _all_time_min_temp;
    float _all_time_optimal_current;
    unsigned long _total_runtime_seconds;
    unsigned long _session_count;

    // Session-only tracking (not persisted)
    float _session_min_temp;

    // Timing for periodic saves
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

    // NVS helpers
    bool checkNvsSpace();
    void loadFromNvs();
    void saveToNvs(bool force);
    void updateRuntime();
};

#endif // THERMAL_METRICS_H
