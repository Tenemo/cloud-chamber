#ifndef TEC_CONTROLLER_H
#define TEC_CONTROLLER_H

#include "Logger.h"
#include "config.h"
#include <Arduino.h>

/**
 * @brief Main TEC control system managing current regulation and state machine
 *
 * Encapsulates the control loop, state management, safety checks, and PI
 * control for thermoelectric cooler (TEC) modules driven via PWM.
 */
class TECController {
  public:
    /**
     * @brief Construct controller (uses constants from config.h)
     *
     * @param logger Reference to logger instance
     */
    TECController(Logger &logger);

    /**
     * @brief Initialize hardware (PWM, enables, sensors)
     */
    void begin();

    /**
     * @brief Calibrate current sensors
     *
     * @return true if calibration successful
     */
    bool calibrateSensors();

    /**
     * @brief Start power detection sequence
     */
    void startPowerDetection();

    /**
     * @brief Main control loop update (call periodically)
     *
     * @param control_interval_ms Time since last update in milliseconds
     * @param display_interval_ms Display update interval
     */
    void update(unsigned long control_interval_ms,
                unsigned long display_interval_ms);

    /**
     * @brief Get current system state
     */
    SystemState getState() const { return _state; }

    /**
     * @brief Get calibration offsets for display
     */
    void getCalibrationOffsets(float &offset1, float &offset2) const;

    /**
     * @brief Emergency shutdown
     */
    void emergencyShutdown();

  private:
    // Helper methods
    void setPwmDuty(float duty);
    void readCurrents(float &tec1, float &tec2, float &total);
    bool handleWaitingForPower(float total_current, float &current_duty);
    void handleSoftStart(unsigned long current_time, float &target_current);
    void computeControl(float target_current, float total_current,
                        float tec1_current, float tec2_current, float dt,
                        float &output_duty);
    bool checkOvercurrent(float total_current);
    void handleOvercurrentError();

    // PI control methods
    float computePI(float error, float dt);
    void resetPI();

    // Sensor management methods
    bool calibrateSensor(int pin, float &offset_voltage);
    float readSensorCurrent(int pin, float offset_voltage,
                            float &filtered_current);
    float readAverageVoltage(int pin, int num_samples);
    static float clampSmallCurrent(float current, float threshold = 0.1f);

    // Component references
    Logger &_logger;

    // State
    SystemState _state;
    unsigned long _soft_start_begin_time;

    // PI controller state
    float _pi_integral;
    float _pi_last_output;

    // Sensor state (TEC1 and TEC2)
    float _tec1_offset_voltage;
    float _tec2_offset_voltage;
    float _tec1_filtered_current;
    float _tec2_filtered_current;
    bool _sensors_calibrated;
    int _adc_max_value;

    // PWM channel
    static constexpr int PWM_CHANNEL = 0;
    static constexpr int PWM_FREQ_HZ = 25000;
    static constexpr int PWM_RES_BITS = 10;
};

#endif // TEC_CONTROLLER_H
