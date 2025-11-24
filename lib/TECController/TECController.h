#ifndef TEC_CONTROLLER_H
#define TEC_CONTROLLER_H

#include "CurrentSensor.h"
#include "Logger.h"
#include <Arduino.h>

/**
 * @brief Main TEC control system managing current regulation and state machine
 *
 * Encapsulates the control loop, state management, safety checks, and PI
 * control for thermoelectric cooler (TEC) modules driven via PWM.
 */
class TECController {
  public:
    struct Config {
        // Hardware pins
        int pin_acs1;
        int pin_acs2;
        int pin_rpwm;
        int pin_l_en;
        int pin_r_en;

        // Sensor parameters
        float adc_ref_v;
        float acs_sensitivity;
        float filter_alpha;
        int adc_samples;

        // Control parameters
        float target_current_per_tec;
        float max_duty;
        float min_duty;

        // PI gains
        float kp;
        float ki;
        float integral_max;

        // Detection and startup
        float detection_duty;
        float detection_threshold;
        unsigned long soft_start_duration_ms;

        // Safety
        float overcurrent_multiplier;
    };

    /**
     * @brief Construct controller with configuration
     *
     * @param config Configuration parameters
     * @param logger Reference to logger instance
     */
    TECController(const Config &config, Logger &logger);

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

    // Configuration
    Config _config;

    // Component instances
    CurrentSensor _tec1_sensor;
    CurrentSensor _tec2_sensor;
    Logger &_logger;

    // State
    SystemState _state;
    unsigned long _soft_start_begin_time;

    // PI controller state
    float _pi_integral;
    float _pi_last_output;

    // PWM channel
    static constexpr int PWM_CHANNEL = 0;
    static constexpr int PWM_FREQ_HZ = 25000;
    static constexpr int PWM_RES_BITS = 10;
};

#endif // TEC_CONTROLLER_H
