/**
 * @file ThermalMetrics.h
 * @brief Unified thermal data tracking: history buffer, trend analysis,
 *        optimization math, and NVS persistence
 *
 * This class combines several related concerns:
 * 1. **Runtime History**: Circular buffer of temperature samples for trend
 *    analysis and cooling rate calculations (volatile, lost on reset)
 * 2. **Optimization Math**: Evaluation of current changes and step size
 *    recommendations based on thermal trends
 * 3. **Persistent Metrics**: NVS storage for long-term tracking of best
 *    temperatures, optimal currents, and total runtime across sessions
 *
 * HISTORY BUFFER:
 * ---------------
 * - 300 samples at 1-second intervals = 5 minutes of history
 * - Linear regression for cooling rate calculation (K/min)
 * - Separate windows for cold plate (60 samples) and hot plate (10 samples)
 *
 * OPTIMIZATION:
 * -------------
 * - OptimizerState struct tracks hill-climber state
 * - evaluateEffect() determines if a current change improved cooling
 * - recommendStepSize() selects adaptive step based on cooling rate
 *
 * NVS PERSISTENCE:
 * ----------------
 * - All-time minimum temperature and corresponding current
 * - Total runtime across all sessions (hours)
 * - Session count
 * - Periodic auto-save with space checking
 */

#ifndef THERMAL_METRICS_H
#define THERMAL_METRICS_H

#include "Logger.h"
#include "ThermalConstants.h"
#include <Arduino.h>
#include <Preferences.h>
#include <nvs.h>

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

/**
 * @brief Result of evaluating a current change
 */
enum class EvaluationResult {
    WAITING,  // Not enough time/data yet
    IMPROVED, // Temperature improved
    DEGRADED, // Temperature worsened or rate degraded
    UNCHANGED // No significant change
};

/**
 * @brief Action to take after evaluation
 *
 * Returned by processEvaluation() to tell the controller what to do.
 * This moves the evaluation logic into ThermalMetrics while keeping
 * the actual PSU control in ThermalController.
 */
struct EvaluationAction {
    EvaluationResult result;
    float revert_current; // Current to revert to (valid if result == DEGRADED)
    bool should_transition; // True if controller should transition to
                            // STEADY_STATE
    bool converged;         // True if optimizer has converged
};

/**
 * @brief Hill-climber optimizer state for current optimization
 *
 * Groups all variables related to finding the optimal TEC current.
 * Simplified to track just session best and step evaluation state.
 */
struct OptimizerState {
    // Session best tracking (the known-good point)
    float optimal_current = 0.0f;   // Best current found so far this session
    float temp_at_optimal = 100.0f; // Temperature achieved at optimal

    // Baseline before last step (for local revert, not jump to global best)
    float baseline_current = 0.0f; // Current before the last step
    float baseline_temp = 100.0f;  // Cold plate temp at baseline
    float baseline_rate = 0.0f;    // Cooling rate at baseline

    // Step evaluation state
    bool awaiting_evaluation = false;  // Evaluation in progress
    unsigned long eval_start_time = 0; // When evaluation started

    // Adaptive step control
    int8_t probe_direction = 1;      // +1 = increase current, -1 = decrease
    float current_step = 0.5f;       // Current step size (adaptive)
    uint8_t consecutive_bounces = 0; // Direction flips - shrink step after 2+
    bool converged = false;          // True when both directions fail

    void reset() {
        optimal_current = 0.0f;
        temp_at_optimal = 100.0f;
        baseline_current = 0.0f;
        baseline_temp = 100.0f;
        baseline_rate = 0.0f;
        awaiting_evaluation = false;
        eval_start_time = 0;
        probe_direction = 1;
        current_step = 0.5f;
        consecutive_bounces = 0;
        converged = false;
    }

    /**
     * @brief Update session best tracking with new optimal point
     *
     * Use this when adopting known-good values (e.g., hot reset detection)
     * or when an evaluation confirms improvement.
     */
    void updateBest(float current, float temp) {
        optimal_current = current;
        temp_at_optimal = temp;
    }
};

/**
 * @brief Unified thermal data tracker
 *
 * Handles both real-time trend analysis (circular buffer) and
 * long-term persistence (NVS) in a single cohesive class.
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

    /**
     * @brief Force immediate save to NVS
     */
    void forceSave();

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

    // =========================================================================
    // Optimizer State & Analysis
    // =========================================================================

    /**
     * @brief Get mutable reference to optimizer state
     *
     * The controller uses this to read/write optimizer state.
     * ThermalMetrics owns the state but controller drives the logic flow.
     */
    OptimizerState &optimizer() { return _optimizer; }
    const OptimizerState &optimizer() const { return _optimizer; }

    /**
     * @brief Evaluate the effect of a current change
     *
     * Checks if enough time has passed, waits for hot-side stabilization,
     * then compares current temperature/rate against baseline.
     *
     * @param cold_temp Current cold plate temperature
     * @return WAITING if not ready, IMPROVED/DEGRADED/UNCHANGED once evaluated
     */
    EvaluationResult evaluateEffect(float cold_temp);

    /**
     * @brief Process a pending evaluation and decide what action to take
     *
     * This consolidates the evaluation logic that was in ThermalController.
     * It evaluates the current change, updates optimizer state, and returns
     * the action the controller should take.
     *
     * @param cold_temp Current cold plate temperature
     * @param current Current current setpoint
     * @param allow_transition If true, can return should_transition=true
     * @param logger Logger for status messages
     * @return EvaluationAction describing what to do
     */
    EvaluationAction processEvaluation(float cold_temp, float current,
                                       bool allow_transition, Logger &logger);

    /**
     * @brief Recommend adaptive step size based on cooling rate and conditions
     *
     * Uses larger steps when far from optimum (fast cooling rate) and
     * smaller steps when near optimum (slow cooling rate or bouncing).
     *
     * @param is_hot_side_warning True if hot side is in warning zone
     * @return Step size in amps (COARSE_STEP_A, MEDIUM_STEP_A, or FINE_STEP_A)
     */
    float recommendStepSize(bool is_hot_side_warning) const;

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

  private:
    Logger &_logger;

    // -------------------------------------------------------------------------
    // Optimizer state
    // -------------------------------------------------------------------------
    OptimizerState _optimizer;

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
