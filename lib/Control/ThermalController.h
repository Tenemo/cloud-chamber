/**
 * @file ThermalController.h
 * @brief Smart thermal control system for cloud chamber TEC cooling
 *
 * ARCHITECTURE:
 * -------------
 * The controller delegates responsibilities to specialized modules:
 * - DualPowerSupply: Symmetric control of two DPS5015 units
 * - ThermalMetrics: History buffer, trend analysis, session tracking
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
#include "DualPowerSupply.h"
#include "SafetyMonitor.h"
#include "TemperatureSensors.h"
#include "ThermalConstants.h"
#include "ThermalMetrics.h"
#include "ThermalOptimizer.h"
#include "ThermalTypes.h"
#include <cstdint>

// ThermalState enum is defined in ThermalTypes.h (shared for context-aware
// checks)

class ThermalController {
  public:
    ThermalController(Logger &logger, TemperatureSensors &sensors);

    void begin();
    void update();

    // State accessors
    ThermalState getState() const { return _state; }
    const char *getStateString() const { return stateToString(_state); }
    float getCoolingRate() const;
    float getTargetCurrent() const { return _dps.getTargetCurrent(); }
    unsigned long getSteadyStateUptime() const;

  private:
    // Hardware references
    Logger &_logger;
    TemperatureSensors &_sensors;

    // Delegate modules (DualPowerSupply is owned, others use references)
    DualPowerSupply _dps;
    ThermalMetrics _metrics;
    ThermalOptimizer _optimizer;
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
    Counter _consecutive_stall_detects;

    // Hot reset recovery tracking
    bool _hot_reset_active;   // True if we detected hot reset and haven't
                              // completed normal startup
    float _hot_reset_current; // Adopted current from hot reset detection

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
    void transitionTo(ThermalState newState, const char *reason = nullptr);
    void enterThermalFault(const char *reason);

    // =========================================================================
    // Safety and control
    // =========================================================================

    /**
     * @brief Run all safety checks at top of update loop
     */
    void runSafetyChecks();

    /**
     * @brief Check if controller is in an operational state
     *
     * Returns true for states where safety checks should run:
     * STARTUP, RAMP_UP, STEADY_STATE, MANUAL_OVERRIDE, SENSOR_FAULT
     *
     * Returns false for:
     * - INITIALIZING: Hardware not ready yet, sensors may not be valid
     * - SELF_TEST: Running DPS verification, not operational
     * - THERMAL_FAULT: Already in fault state, shutdown in progress
     * - DPS_DISCONNECTED: No PSU communication, can't check or control
     *
     * @return true if in operational state requiring safety monitoring
     */
    bool isOperationalState() const;

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
     * @brief Apply optimizer decision (log message, set current if requested)
     *
     * Common pattern used by both handleRampUp() and handleSteadyState().
     * Logs any decision message and applies current changes to DPS.
     *
     * @param decision The optimization decision from ThermalOptimizer
     * @param snapshot_now Current timestamp from snapshot
     */
    void applyOptimizationDecision(const ThermalOptimizationDecision &decision,
                                   unsigned long snapshot_now);

    // =========================================================================
    // History and display
    // =========================================================================
    void registerDisplayLines();
    void updateDisplay();
};

#endif // THERMAL_CONTROLLER_H
