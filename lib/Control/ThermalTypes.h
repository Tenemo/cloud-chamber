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

#endif // THERMAL_TYPES_H
