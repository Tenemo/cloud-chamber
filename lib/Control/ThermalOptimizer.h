/**
 * @file ThermalOptimizer.h
 * @brief Hill-climbing current optimizer for TEC control
 *
 * This class encapsulates the optimal current search algorithm:
 * - Decides when and how to adjust TEC current
 * - Evaluates whether current changes improved cooling
 * - Manages step sizes and probe directions
 * - Tracks convergence and bounce detection
 *
 * The optimizer is purely about policy - it doesn't directly control hardware.
 * ThermalController calls update() with a snapshot of system state, and the
 * optimizer returns a decision (change current or not, to what value, etc.).
 *
 * ALGORITHM:
 * ----------
 * During RAMP_UP: Monotonically increase current until:
 *   - Maximum current reached
 *   - Hot side limiting kicks in
 *   - Cooling stalls or degrades
 *
 * During STEADY_STATE: Bidirectional probing to find/maintain optimum:
 *   - Alternate between increase and decrease attempts
 *   - Converge when both directions show degradation
 *   - Periodically re-probe to adapt to changing conditions
 */

#ifndef THERMAL_OPTIMIZER_H
#define THERMAL_OPTIMIZER_H

#include "Logger.h"
#include "ThermalConstants.h"

/**
 * @brief Control phase for optimizer decisions
 */
enum class ThermalControlPhase {
    RAMP_UP,     // Monotonic upward search
    STEADY_STATE // Bidirectional fine-tuning
};

/**
 * @brief Result of evaluating a current change
 */
enum class ThermalEvaluationResult {
    WAITING,  // Not enough time/data yet
    IMPROVED, // Temperature improved
    DEGRADED, // Temperature worsened or rate degraded
    UNCHANGED // No significant change
};

/**
 * @brief Snapshot of thermal system state for optimizer decisions
 *
 * The controller populates this with current sensor readings and
 * passes it to the optimizer. This decouples the optimizer from
 * direct hardware access.
 */
struct ThermalSnapshot {
    float cold_temp;        // Cold plate temperature (°C)
    float hot_temp;         // Hot plate temperature (°C)
    float cooling_rate;     // Cold plate rate (K/min, negative = cooling)
    float hot_side_rate;    // Hot plate rate (K/min)
    float current_setpoint; // Current per channel (A)
    bool hot_in_warning;    // Hot side in warning zone
    bool hot_in_alarm;      // Hot side in alarm zone
    bool hot_side_stable;   // Hot side rate below threshold
    unsigned long now;      // Current millis() timestamp
};

/**
 * @brief Decision returned by optimizer
 *
 * Tells the controller what action to take, if any.
 */
struct ThermalOptimizationDecision {
    bool change_current;      // If true, set current to new_current
    float new_current;        // Target current (valid if change_current)
    bool evaluation_complete; // True if we just processed a pending evaluation
    bool should_transition;   // True if controller should exit RAMP_UP
    bool converged;           // True if optimizer has converged at optimum
    const char *log_message;  // Optional log message (nullptr if none)
};

/**
 * @brief Hill-climbing optimizer for TEC current control
 *
 * Encapsulates all the logic for finding and maintaining optimal TEC current.
 */
class ThermalOptimizer {
  public:
    explicit ThermalOptimizer(Logger &logger);

    /**
     * @brief Reset optimizer to initial state
     *
     * Call when entering a new control session (e.g., on boot or after
     * manual override).
     */
    void reset();

    /**
     * @brief Initialize with a known-good starting point
     *
     * Use this for hot reset detection when DPS is already running.
     * Starts a stabilization period before optimization begins.
     *
     * @param current Current current setpoint
     * @param temp Current cold plate temperature
     */
    void seedWithCurrent(float current, float temp);

    /**
     * @brief Main update - decide whether to adjust current
     *
     * Call this from ThermalController::handleRampUp/handleSteadyState.
     * The optimizer examines the snapshot and decides whether to change
     * current, based on evaluation results and timing.
     *
     * @param snapshot Current system state
     * @param phase Control phase (RAMP_UP or STEADY_STATE)
     * @param last_adjustment_time Time of last current adjustment
     * @param adjustment_interval_ms Required interval between adjustments
     * @param psus_ready True if PSUs are settled and ready for commands
     * @return Decision describing what action to take
     */
    ThermalOptimizationDecision update(const ThermalSnapshot &snapshot,
                                       ThermalControlPhase phase,
                                       unsigned long last_adjustment_time,
                                       unsigned long adjustment_interval_ms,
                                       bool psus_ready);

    /**
     * @brief Check if ramp-up should terminate
     *
     * @param snapshot Current system state
     * @param has_enough_history True if history buffer has sufficient samples
     * @return true if any termination condition is met
     */
    bool shouldExitRamp(const ThermalSnapshot &snapshot,
                        bool has_enough_history) const;

    /**
     * @brief Get current optimal current (best found so far)
     */
    float getOptimalCurrent() const { return _optimal_current; }

    /**
     * @brief Get temperature at optimal current
     */
    float getTempAtOptimal() const { return _temp_at_optimal; }

    /**
     * @brief Get current probe direction (+1 or -1)
     */
    int8_t getProbeDirection() const { return _probe_direction; }

    /**
     * @brief Set probe direction for next probe
     */
    void setProbeDirection(int8_t dir) { _probe_direction = dir; }

    /**
     * @brief Check if optimizer has converged
     */
    bool isConverged() const { return _converged; }

    /**
     * @brief Check if in stabilization period (e.g., after hot reset)
     */
    bool isInStabilization() const {
        return _stabilization_until > 0 && millis() < _stabilization_until;
    }

    /**
     * @brief Check if an evaluation is pending
     */
    bool isEvaluationPending() const { return _awaiting_evaluation; }

    /**
     * @brief Clear converged state (for periodic re-probing)
     */
    void clearConverged() {
        _converged = false;
        _consecutive_bounces = 0;
        _consecutive_degraded = 0;
    }

    /**
     * @brief Clear any pending step evaluation (used when changing phases)
     */
    void clearPendingEvaluation() {
        _awaiting_evaluation = false;
        _eval_start_time = 0;
    }

    /**
     * @brief Update best tracking with new optimal point
     */
    void updateBest(float current, float temp) {
        _optimal_current = current;
        _temp_at_optimal = temp;
    }

  private:
    Logger &_logger;

    // Session best tracking (the known-good point)
    float _optimal_current;
    float _temp_at_optimal;

    // Baseline before last step (for local revert)
    float _baseline_current;
    float _baseline_temp;
    float _baseline_rate;

    // Step evaluation state
    bool _awaiting_evaluation;
    unsigned long _eval_start_time;

    // Adaptive step control
    int8_t _probe_direction;
    uint8_t _consecutive_bounces;
    uint8_t _consecutive_degraded; // Track consecutive degraded evals
    bool _converged;

    // Hot reset stabilization
    unsigned long _stabilization_until;

    // Formatting buffer for log messages
    mutable char _log_buffer[64];

    /**
     * @brief Evaluate effect during RAMP_UP (temperature-focused)
     *
     * During ramp, only declare degradation if temperature actually rises.
     * Rate-based degradation is disabled since rate naturally decreases
     * as cold plate gets colder (basic thermodynamics).
     *
     * @param snapshot Current system state
     * @return WAITING if not ready, or evaluation result
     */
    ThermalEvaluationResult evaluateRampEffect(const ThermalSnapshot &snapshot);

    /**
     * @brief Evaluate effect during STEADY_STATE (rate-focused)
     *
     * In steady state, rate-based fine-tuning is appropriate since we're
     * near equilibrium and looking for marginal improvements.
     *
     * @param snapshot Current system state
     * @return WAITING if not ready, or evaluation result
     */
    ThermalEvaluationResult
    evaluateSteadyEffect(const ThermalSnapshot &snapshot);

    /**
     * @brief Process a pending evaluation and update internal state
     *
     * @param snapshot Current system state
     * @param phase Control phase
     * @return Decision with evaluation results
     */
    ThermalOptimizationDecision
    processEvaluation(const ThermalSnapshot &snapshot,
                      ThermalControlPhase phase);

    /**
     * @brief Calculate adaptive step size for RAMP_UP (current-based)
     *
     * During ramp-up, step size is based on current setpoint:
     * - Below 6A: coarse steps (0.5A) for fast approach
     * - 6-10A: medium steps (0.25A)
     * - Above 10A: fine steps (0.1A) for careful approach
     *
     * @param current_setpoint Current setpoint
     * @param is_hot_side_warning True if hot side in warning zone
     * @return Step size in amps
     */
    float calculateRampStepSize(float current_setpoint,
                                bool is_hot_side_warning) const;

    /**
     * @brief Calculate adaptive step size for STEADY_STATE (rate-based)
     *
     * During steady-state probing, step size is based on cooling rate:
     * - Fast rate: system not near equilibrium, use coarse steps
     * - Slow rate: near equilibrium, use fine steps
     *
     * @param cooling_rate Current cooling rate magnitude
     * @param is_hot_side_warning True if hot side in warning zone
     * @return Step size in amps
     */
    float calculateSteadyStepSize(float cooling_rate,
                                  bool is_hot_side_warning) const;

    /**
     * @brief Attempt a current adjustment in RAMP_UP mode
     *
     * @param snapshot Current system state
     * @return Decision with new current, or no change if conditions not met
     */
    ThermalOptimizationDecision
    attemptRampStep(const ThermalSnapshot &snapshot);

    /**
     * @brief Attempt a current probe in STEADY_STATE mode
     *
     * @param snapshot Current system state
     * @return Decision with new current, or no change if conditions not met
     */
    ThermalOptimizationDecision
    attemptSteadyProbe(const ThermalSnapshot &snapshot);
};

#endif // THERMAL_OPTIMIZER_H
