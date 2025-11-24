#ifndef TEC_CONTROLLER_H
#define TEC_CONTROLLER_H

#include "Logger.h"
#include "config.h"
#include <Arduino.h>

class TECController {
  public:
    TECController(Logger &logger);

    void begin();
    bool calibrateSensors();
    void startPowerDetection();
    void update();

    SystemState getState() const { return _state; }
    void getCalibrationOffsets(float &offset1, float &offset2) const;
    void emergencyShutdown();

  private:
    void setPwmDuty(float duty);
    void readCurrents(float &tec1, float &tec2, float &total);
    bool handleWaitingForPower(float total_current, float &current_duty);
    void handleSoftStart(unsigned long current_time, float &target_current);
    void computeControl(float target_current, float total_current,
                        float tec1_current, float tec2_current, float dt,
                        float &output_duty);
    bool checkOvercurrent(float total_current);
    void handleOvercurrentError();

    float computePI(float error, float dt);
    void resetPI();

    bool calibrateSensor(int pin, float &offset_voltage);
    float readSensorCurrent(int pin, float offset_voltage,
                            float &filtered_current);
    float readAverageVoltage(int pin, int num_samples);
    static float clampSmallCurrent(float current, float threshold = 0.1f);

    Logger &_logger;

    SystemState _state;
    unsigned long _soft_start_begin_time;

    float _pi_integral;
    float _pi_last_output;

    float _tec1_offset_voltage;
    float _tec2_offset_voltage;
    float _tec1_filtered_current;
    float _tec2_filtered_current;
    bool _sensors_calibrated;
    int _adc_max_value;

    static constexpr int PWM_CHANNEL = 0;
    static constexpr int PWM_FREQ_HZ = 25000;
    static constexpr int PWM_RES_BITS = 10;
};

#endif // TEC_CONTROLLER_H
