/**
 * @file ThermalTypes.h
 * @brief Shared type definitions for the thermal control system
 *
 * This header contains types that are shared across multiple thermal
 * control modules (ThermalController, SafetyMonitor, etc.) to avoid
 * circular dependencies and clarify ownership.
 */

#ifndef THERMAL_TYPES_H
#define THERMAL_TYPES_H

/**
 * @brief State machine states for ThermalController
 *
 * Defined in a shared header so that SafetyMonitor and other modules
 * can reference these states for context-aware behavior without
 * creating a dependency on ThermalController itself.
 */
enum class ThermalState {
    INITIALIZING,    // Waiting for hardware to come online
    SELF_TEST,       // Verifying DPS communication and output control
    STARTUP,         // Soft-start with low current
    RAMP_UP,         // Gradually increasing current to find optimum
    STEADY_STATE,    // Maintaining optimal operation
    MANUAL_OVERRIDE, // Human has taken control
    THERMAL_FAULT,   // Critical temperature exceeded - shutdown
    SENSOR_FAULT,    // Sensor failure - reduced power
    DPS_DISCONNECTED // Lost communication with PSU(s)
};

/**
 * @brief Convert ThermalState to display string
 */
inline const char *stateToString(ThermalState state) {
    switch (state) {
    case ThermalState::INITIALIZING:
        return "INIT";
    case ThermalState::SELF_TEST:
        return "TEST";
    case ThermalState::STARTUP:
        return "START";
    case ThermalState::RAMP_UP:
        return "RAMP";
    case ThermalState::STEADY_STATE:
        return "STEADY";
    case ThermalState::MANUAL_OVERRIDE:
        return "MANUAL";
    case ThermalState::THERMAL_FAULT:
        return "FAULT";
    case ThermalState::SENSOR_FAULT:
        return "SENS_F";
    case ThermalState::DPS_DISCONNECTED:
        return "DSCNCT";
    default:
        return "???";
    }
}

#endif // THERMAL_TYPES_H
