/**
 * @file ThermalController.h
 * @brief Smart thermal control system for cloud chamber TEC cooling
 *
 * ARCHITECTURE:
 * -------------
 * The controller delegates responsibilities to specialized modules:
 * - DualPowerSupply: Symmetric control of two DPS5015 units
 * - ThermalMetrics: Unified history buffer, trend analysis, and NVS persistence
 * - SafetyMonitor: Centralized safety checks
 * - CrashLog: SPIFFS persistence for crash diagnostics
 *
 * DESIGN PRINCIPLES:
 * ------------------
 * 1. Safety checks run once per update(), not in each state handler
 * 2. State handlers contain only state-specific logic
 * 3. All PSU control goes through DualPowerSupply wrapper
 * 4. Constants that rarely change are in ThermalConstants.h
 */

#ifndef THERMAL_CONTROLLER_H
#define THERMAL_CONTROLLER_H

#include "CrashLog.h"
#include "DS18B20.h"
#include "DualPowerSupply.h"
#include "Logger.h"
#include "PT100.h"
#include "SafetyMonitor.h"
#include "ThermalConstants.h"
#include "ThermalMetrics.h"
#include <Arduino.h>

// Controller state machine states
enum class ThermalState {
    INITIALIZING,    // Waiting for hardware to come online
    SELF_TEST,       // Running DPS self-test
    STARTUP,         // Soft-start with low current
    RAMP_UP,         // Gradually increasing current to maximum
    STEADY_STATE,    // Maintaining maximum safe operation
    MANUAL_OVERRIDE, // Human has taken control
    THERMAL_FAULT,   // Critical temperature exceeded
    SENSOR_FAULT,    // Critical sensor failure
    DPS_DISCONNECTED // Lost communication with PSU(s)
};

// Result of evaluating a current change
enum class EvaluationResult { WAITING, IMPROVED, DEGRADED, UNCHANGED };

/**
 * @brief Hill-climber optimizer state for current optimization
 *
 * Groups all variables related to finding the optimal TEC current.
 * This makes save/restore of optimizer state cleaner and reduces
 * clutter in the main controller class.
 *
 * Key concepts:
 * - optimal_current/temp_at_optimal: Session best (updated when truly better)
 * - baseline_current/temp: Point before the last step (for local revert)
 * - probe_direction: +1 = increasing current, -1 = decreasing
 * - current_step: Adaptive step size (coarse→medium→fine as we approach
 * optimum)
 * - consecutive_bounces: Tracks direction flips to shrink step size
 * - converged: True when both directions fail to improve (stop probing)
 */
struct OptimizerState {
    // Session best tracking
    float optimal_current = 0.0f;   // Best current found so far this session
    float temp_at_optimal = 100.0f; // Temperature achieved at optimal

    // Baseline before last step (for local revert, not jump to global best)
    float baseline_current = 0.0f; // Current before the last step
    float baseline_temp = 100.0f;  // Cold plate temp at baseline
    float baseline_rate = 0.0f;    // Cooling rate at baseline

    // Step evaluation state
    bool awaiting_evaluation = false;  // Evaluation in progress
    unsigned long eval_start_time = 0; // When evaluation started

    // Global best tracking during ramp (may differ from step-by-step optimal)
    float best_temp_during_ramp = 100.0f; // Coldest temp seen during this ramp
    float current_at_best_temp = 0.0f;    // Current that achieved best temp

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
        best_temp_during_ramp = 100.0f;
        current_at_best_temp = 0.0f;
        probe_direction = 1;
        current_step = 0.5f;
        consecutive_bounces = 0;
        converged = false;
    }
};

class ThermalController {
  public:
    ThermalController(Logger &logger, PT100Sensor &coldPlate,
                      DS18B20Sensor &hotPlate, DPS5015 &psu0, DPS5015 &psu1);

    void begin();
    void update();

    // State accessors
    ThermalState getState() const { return _state; }
    const char *getStateString() const;
    float getCoolingRate() const;
    float getTargetCurrent() const { return _dps.getTargetCurrent(); }
    unsigned long getSteadyStateUptime() const;

  private:
    // Hardware references
    Logger &_logger;
    PT100Sensor &_cold_plate;
    DS18B20Sensor &_hot_plate;

    // Delegate modules
    DualPowerSupply _dps;
    ThermalMetrics _metrics;
    SafetyMonitor _safety;

    // State machine
    ThermalState _state;
    ThermalState _previous_state;
    unsigned long _state_entry_time;
    unsigned long _last_sample_time;

    // Hill-climber optimizer state
    OptimizerState _optimizer;

    // Tracking
    // Session minimum temperature (reset each boot). Used for display/logging.
    // Distinct from ThermalMetrics::_all_time_min_temp which persists to NVS.
    float _min_cold_temp_achieved;
    unsigned long _last_adjustment_time;
    unsigned long _steady_state_start_time;
    unsigned long _ramp_start_time;
    unsigned long _sensor_fault_time;

    // =========================================================================
    // State handlers (contain only state-specific logic)
    // =========================================================================
    void handleInitializing();
    void handleSelfTest();
    void handleStartup();
    void handleRampUp();
    void handleSteadyState();
    void handleManualOverride();
    void handleThermalFault();
    void handleSensorFault();
    void handleDpsDisconnected();

    // =========================================================================
    // State transitions
    // =========================================================================
    void transitionTo(ThermalState newState);
    void enterThermalFault(const char *reason);

    // =========================================================================
    // Safety and control
    // =========================================================================

    /**
     * @brief Run all safety checks at top of update loop
     * @return true if ok to continue, false if transitioned to fault state
     */
    bool runSafetyChecks();

    /**
     * @brief Check if we're allowed to send PSU commands
     *
     * Centralizes the "am I allowed to command the DPS?" decision.
     * Returns false when in states that should not modify PSU settings:
     * - MANUAL_OVERRIDE: Human has taken control
     * - THERMAL_FAULT: Emergency shutdown in progress
     * - DPS_DISCONNECTED: No PSU communication
     * - Shutdown in progress: Emergency ramp-down active
     *
     * @return true if PSU commands may be issued
     */
    bool canControlPower() const;

    /**
     * @brief Evaluate the result of a current change
     */
    EvaluationResult evaluateCurrentChange();

    /**
     * @brief Process pending evaluation if one is active
     * @param allow_transition If true, may trigger state transition on bounce
     * @return true if evaluation was processed (IMPROVED, DEGRADED, or
     * UNCHANGED) false if no evaluation pending or still WAITING
     */
    bool processEvaluationIfPending(bool allow_transition = false);

    /**
     * @brief Choose adaptive step size based on cooling rate and hot-side temp
     *
     * Uses larger steps when far from optimum (fast cooling rate) and
     * smaller steps when near optimum (slow cooling rate or hot side warm).
     *
     * @return Step size in amps (COARSE_STEP_A, MEDIUM_STEP_A, or FINE_STEP_A)
     */
    float chooseStepSize() const;

    // =========================================================================
    // History and display
    // =========================================================================
    void recordSample();
    void registerDisplayLines();
    void updateDisplay();
};

#endif // THERMAL_CONTROLLER_H
