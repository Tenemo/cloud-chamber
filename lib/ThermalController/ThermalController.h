/**
 * @file ThermalController.h
 * @brief Smart thermal control system for cloud chamber TEC cooling
 *
 * ARCHITECTURE:
 * -------------
 * The controller delegates responsibilities to specialized modules:
 * - DualPowerSupply: Symmetric control of two DPS5015 units
 * - ThermalHistory: Circular buffer and trend analysis
 * - ThermalMetrics: NVS persistence for long-term tracking
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

// Result of evaluating a current change
enum class EvaluationResult { WAITING, IMPROVED, DEGRADED, UNCHANGED };

class ThermalController {
  public:
    ThermalController(Logger &logger, PT100Sensor &coldPlate,
                      DS18B20Sensor &hotPlate, DS18B20Sensor *ambientSensors,
                      size_t numAmbient, DPS5015 &psu0, DPS5015 &psu1);

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
    DS18B20Sensor *_ambient_sensors;
    size_t _num_ambient;

    // Delegate modules
    DualPowerSupply _dps;
    ThermalHistory _history;
    ThermalMetrics _metrics;
    SafetyMonitor _safety;

    // State machine
    ThermalState _state;
    ThermalState _previous_state;
    unsigned long _state_entry_time;
    unsigned long _last_sample_time;

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
    unsigned long _last_imbalance_log_time;

    // Self-test state
    int _selftest_phase;
    unsigned long _selftest_phase_start;
    bool _selftest_passed[2];

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
     * @brief Evaluate the result of a current change
     */
    EvaluationResult evaluateCurrentChange();

    // =========================================================================
    // History and display
    // =========================================================================
    void recordSample();
    float getAmbientTemperature() const;
    void checkChannelImbalance();
    void registerDisplayLines();
    void updateDisplay();
};

#endif // THERMAL_CONTROLLER_H
