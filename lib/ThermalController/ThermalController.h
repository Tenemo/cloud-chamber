/**
 * @file ThermalController.h
 * @brief Smart thermal control system for cloud chamber TEC cooling
 *
 * This module coordinates two DPS5015 power supplies controlling TEC pairs
 * to achieve and maintain optimal cold plate temperature for cloud chamber
 * operation.
 *
 * ARCHITECTURE:
 * -------------
 * The controller delegates responsibilities to specialized modules:
 * - ThermalHistory: Circular buffer and trend analysis
 * - ThermalMetrics: NVS persistence for long-term tracking
 * - SafetyMonitor: Centralized safety checks with consistent error handling
 * - CrashLog: SPIFFS persistence for crash diagnostics
 *
 * FEATURES:
 * ---------
 * - Hierarchical state machine with safety-first design
 * - Symmetric dual-channel current control (both channels always together)
 * - Automatic optimization to find minimum cold plate temperature
 * - Manual override detection (respects human intervention)
 * - Thermal fault protection with emergency shutdown
 * - Hot reset recovery (adopts running current to avoid thermal shock)
 */

#ifndef THERMAL_CONTROLLER_H
#define THERMAL_CONTROLLER_H

#include "CrashLog.h"
#include "DPS5015.h"
#include "DS18B20.h"
#include "Logger.h"
#include "PT100.h"
#include "SafetyMonitor.h"
#include "ThermalHistory.h"
#include "ThermalMetrics.h"
#include "config.h"
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

/**
 * @brief Result of evaluating a current change
 */
enum class EvaluationResult {
    WAITING,  // Still waiting for stabilization
    IMPROVED, // Temperature improved
    DEGRADED, // Temperature or rate degraded
    UNCHANGED // No significant change
};

class ThermalController {
  public:
    ThermalController(Logger &logger, PT100Sensor &coldPlate,
                      DS18B20Sensor &hotPlate, DS18B20Sensor *ambientSensors,
                      size_t numAmbient, DPS5015 *psus);

    void begin();
    void update();

    // State accessors
    ThermalState getState() const { return _state; }
    const char *getStateString() const;
    float getCoolingRate() const;
    float getTargetCurrent(size_t channel) const;
    unsigned long getSteadyStateUptime() const;

  private:
    // Hardware references
    Logger &_logger;
    PT100Sensor &_cold_plate;
    DS18B20Sensor &_hot_plate;
    DS18B20Sensor *_ambient_sensors;
    size_t _num_ambient;
    DPS5015 *_psus;

    // Delegate modules
    ThermalHistory _history;
    ThermalMetrics _metrics;
    SafetyMonitor _safety;

    // State machine
    ThermalState _state;
    ThermalState _previous_state;
    unsigned long _state_entry_time;
    unsigned long _last_sample_time;

    // Current setpoints
    float _target_current[2];

    // Optimal current tracking
    float _optimal_current;
    float _temp_at_optimal;
    float _temp_before_last_increase;
    float _rate_before_last_increase;
    bool _awaiting_evaluation;
    unsigned long _evaluation_start_time;

    // Tracking
    float _min_cold_temp_achieved;
    unsigned long _last_adjustment_time;
    unsigned long _steady_state_start_time;
    unsigned long _ramp_start_time;
    unsigned long _sensor_fault_time;

    // Emergency shutdown state
    bool _shutdown_in_progress;
    float _shutdown_current;
    unsigned long _last_shutdown_step_time;

    // Self-test state
    int _selftest_phase;
    unsigned long _selftest_phase_start;
    bool _selftest_passed[2];

    // Imbalance logging
    unsigned long _last_imbalance_log_time;

    // Display line IDs
    static constexpr const char *LINE_STATE = "TC_STATE";
    static constexpr const char *LINE_RATE = "TC_RATE";
    static constexpr const char *LINE_I1_SET = "TC_I1";
    static constexpr const char *LINE_I2_SET = "TC_I2";

    // =========================================================================
    // State handlers
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
    // Safety check wrapper
    // =========================================================================

    /**
     * @brief Run all safety checks and handle transitions
     * @param include_plausibility Include PT100 plausibility check
     * @param include_cross_check Include cross-sensor validation
     * @return true if all checks passed, false if transitioned to fault state
     */
    bool runSafetyChecks(bool include_plausibility = true,
                         bool include_cross_check = true);

    // =========================================================================
    // Control actions
    // =========================================================================

    /**
     * @brief Set current on both channels symmetrically
     * @param current Target current (will be clamped to valid range)
     */
    void setAllCurrents(float current);

    /**
     * @brief Start non-blocking emergency shutdown
     */
    void startEmergencyShutdown();

    /**
     * @brief Continue emergency shutdown ramp-down
     */
    void updateEmergencyShutdown();

    // =========================================================================
    // Evaluation logic (extracted from handleRampUp/handleSteadyState)
    // =========================================================================

    /**
     * @brief Evaluate the result of a current change
     * @return EvaluationResult indicating what to do next
     */
    EvaluationResult evaluateCurrentChange();

    /**
     * @brief Check if plausibility checks should be skipped
     */
    bool shouldSkipPlausibilityCheck() const;

    /**
     * @brief Check if cross-sensor validation should be skipped
     */
    bool shouldSkipCrossCheck() const;

    // =========================================================================
    // History and analysis
    // =========================================================================

    /**
     * @brief Record a sample to the history buffer
     */
    void recordSample();

    /**
     * @brief Get average ambient temperature
     */
    float getAmbientTemperature() const;

    /**
     * @brief Check for channel imbalance and log if significant
     */
    void checkChannelImbalance();

    // =========================================================================
    // Display
    // =========================================================================
    void registerDisplayLines();
    void updateDisplay();
};

#endif // THERMAL_CONTROLLER_H
