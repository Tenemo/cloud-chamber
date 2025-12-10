/**
 * @file ThermalController.h
 * @brief Smart thermal control system for cloud chamber TEC cooling
 *
 * This module coordinates two DPS5015 power supplies controlling TEC pairs
 * to achieve and maintain optimal cold plate temperature for cloud chamber
 * operation.
 *
 * FEATURES:
 * ---------
 * - Hierarchical state machine with safety-first design
 * - Independent dual-channel current control
 * - Automatic optimization to find minimum cold plate temperature
 * - Manual override detection (respects human intervention)
 * - Thermal fault protection with emergency shutdown
 * - 5-minute history buffer for trend analysis
 *
 * USAGE:
 * ------
 *   ThermalController controller(logger, coldPlateSensor, hotPlateSensor,
 *                                 ambientSensors, numAmbient, psus);
 *   controller.begin();
 *   // In loop:
 *   controller.update();
 */

#ifndef THERMAL_CONTROLLER_H
#define THERMAL_CONTROLLER_H

#include "DPS5015.h"
#include "DS18B20.h"
#include "Logger.h"
#include "PT100.h"
#include "config.h"
#include <Arduino.h>

// Controller state machine states
enum class ThermalState {
    INITIALIZING,    // Waiting for hardware to come online
    STARTUP,         // Soft-start with low current
    RAMP_UP,         // Gradually increasing current to maximum
    STEADY_STATE,    // Maintaining maximum safe operation
    MANUAL_OVERRIDE, // Human has taken control
    THERMAL_FAULT,   // Critical temperature exceeded
    SENSOR_FAULT,    // Critical sensor failure
    DPS_DISCONNECTED // Lost communication with PSU(s)
};

// History sample record
struct ThermalSample {
    unsigned long timestamp_ms;
    float cold_plate_temp;
    float hot_plate_temp;
    float ambient_temp;
    float current_setpoint_ch1;
    float current_setpoint_ch2;
    float actual_current_ch1;
    float actual_current_ch2;
    float power_ch1;
    float power_ch2;
};

// Trend classification for analysis
enum class ThermalTrend {
    COOLING,     // Sustained negative temperature slope
    WARMING,     // Sustained positive slope
    STABLE,      // Near-zero slope with low variance
    OSCILLATING, // Slope changes sign frequently
    ANOMALOUS    // High variance without clear trend
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
    float getCoolingRate() const { return _cooling_rate; }
    float getTargetCurrent(size_t channel) const;
    unsigned long getSteadyStateUptime() const;

    // Manual control (for testing/debugging)
    void requestManualOverride();

  private:
    // References to hardware
    Logger &_logger;
    PT100Sensor &_cold_plate;
    DS18B20Sensor &_hot_plate;
    DS18B20Sensor *_ambient_sensors;
    size_t _num_ambient;
    DPS5015 *_psus;

    // State machine
    ThermalState _state;
    ThermalState _previous_state; // For recovery after sensor fault
    unsigned long _state_entry_time;
    unsigned long _last_update_time;

    // Current setpoints (commanded to each channel)
    float _target_current[2];

    // Optimal current tracking - detects when more power makes things worse
    float _optimal_current;           // Best current found so far
    float _temp_at_optimal;           // Temperature at optimal current
    float _temp_before_last_increase; // Temperature before last current bump
    float _rate_before_last_increase; // Cooling rate before last current bump
    bool _awaiting_evaluation; // True when waiting to evaluate last change
    unsigned long _evaluation_start_time;

    // Steady state tracking
    float _min_cold_temp_achieved;
    unsigned long _last_adjustment_time;

    // Cooling rate calculation
    float _cooling_rate; // K/min, negative = cooling

    // Steady state tracking
    unsigned long _steady_state_start_time;

    // History buffer (circular)
    ThermalSample _history[HISTORY_BUFFER_SIZE];
    size_t _history_head;
    size_t _history_count;
    unsigned long _last_sample_time;

    // Fault tracking
    unsigned long _sensor_fault_time;
    bool _dps_was_connected[2];

    // Emergency shutdown state (non-blocking)
    bool _shutdown_in_progress;
    float _shutdown_current;
    unsigned long _last_shutdown_step_time;

    // Temperature hysteresis tracking
    bool _hot_side_in_warning;
    bool _hot_side_in_alarm;

    // Channel imbalance tracking
    unsigned long _last_imbalance_log_time;

    // Display line IDs
    static constexpr const char *LINE_STATE = "TC_STATE";
    static constexpr const char *LINE_RATE = "TC_RATE";
    static constexpr const char *LINE_I1_SET = "TC_I1";
    static constexpr const char *LINE_I2_SET = "TC_I2";

    // State handlers
    void handleInitializing();
    void handleStartup();
    void handleRampUp();
    void handleSteadyState();
    void handleManualOverride();
    void handleThermalFault();
    void handleSensorFault();
    void handleDpsDisconnected();

    // State transitions
    void transitionTo(ThermalState newState);
    void enterThermalFault(const char *reason);

    // Safety checks
    bool checkForManualOverride();
    bool checkThermalLimits();
    bool checkSensorHealth();
    bool checkDpsConnection();

    // Control actions
    void setChannelCurrent(size_t channel, float current);
    void setAllCurrents(float current);
    void startEmergencyShutdown();
    void updateEmergencyShutdown();
    void emergencyShutdown(); // Legacy blocking version for init only
    void rampDownCurrent(float targetCurrent, float ratePerSecond);

    // History and analysis
    void recordSample();
    float calculateCoolingRate() const;
    ThermalTrend analyzeTrend() const;
    float getAmbientTemperature() const;
    void checkChannelImbalance();

    // Display updates
    void registerDisplayLines();
    void updateDisplay();
};

#endif // THERMAL_CONTROLLER_H
