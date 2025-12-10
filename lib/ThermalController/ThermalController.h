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
#include "ThermalTypes.h"
#include <Arduino.h>

// ThermalState enum is defined in SafetyMonitor.h (shared for context-aware
// checks) EvaluationResult and OptimizerState are defined in ThermalMetrics.h

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
    ThermalState
        _state_before_fault; // State to return to after sensor fault recovery
    unsigned long _state_entry_time;
    unsigned long _last_sample_time;

    // Timing tracking
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

    /**
     * @brief Check if ramp-up should terminate
     *
     * Returns true if any termination condition is met:
     * - At maximum current
     * - Hot side in alarm zone
     * - Cooling has stalled
     */
    bool shouldExitRamp(float current) const;

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
     * @brief Try to complete a pending evaluation
     *
     * If an evaluation is in progress and ready, processes it and
     * takes appropriate action (accept improvement or revert).
     *
     * @param may_transition If true, may trigger state transition on bounce
     * @return true if evaluation completed (result available),
     *         false if no evaluation pending or still waiting
     */
    bool tryCompleteEvaluation(bool may_transition = false);

    // =========================================================================
    // History and display
    // =========================================================================
    void recordSample();
    void registerDisplayLines();
    void updateDisplay();
};

#endif // THERMAL_CONTROLLER_H
