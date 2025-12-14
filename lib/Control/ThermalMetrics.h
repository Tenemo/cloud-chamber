/**
 * @file ThermalMetrics.h
 * @brief Thermal history buffer and trend analysis
 *
 * This class handles data tracking:
 * 1. **Runtime History**: Circular buffer of temperature samples for trend
 *    analysis and cooling rate calculations (volatile, lost on reset)
 * 2. **Trend Analysis**: Linear regression for cooling/heating rate
 *
 * HISTORY BUFFER:
 * ---------------
 * - 300 samples at 1-second intervals = 5 minutes of history
 * - Linear regression for cooling rate calculation (K/min)
 * - Separate windows for cold plate (30 samples) and hot plate (60 samples)
 *
 * NOTE: Optimization logic (hill-climbing, step sizing) is in ThermalOptimizer.
 */

#ifndef THERMAL_METRICS_H
#define THERMAL_METRICS_H

#include "Logger.h"
#include "ThermalConstants.h"

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
     * @brief Initialize metrics system
     */
    void begin();

    /**
     * @brief Periodic update (no-op, kept for API compatibility)
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

    // Timing for periodic logs
    unsigned long _last_temp_log_time;

    // Periodic temperature log interval
    static constexpr unsigned long TEMP_LOG_INTERVAL_MS = 10000; // 10 seconds
};

#endif // THERMAL_METRICS_H
