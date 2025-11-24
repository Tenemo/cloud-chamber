#ifndef CURRENT_LOGGER_H
#define CURRENT_LOGGER_H

#include <Arduino.h>

enum SystemState {
    STATE_INIT,
    STATE_CALIBRATING,
    STATE_SOFT_START,
    STATE_RUNNING,
    STATE_ERROR
};

class CurrentLogger {
  public:
    CurrentLogger();

    void logInitialization();
    void logCalibrationStart();
    void logCalibrationResults(float acs1_offset_V, float acs2_offset_V);
    void logSoftStartBegin(float target_current_per_tec, float max_duty);
    void logSoftStartComplete();
    void logControlActive();

    void logMeasurements(float target_current, float total_current,
                         float tec1_current, float tec2_current, float duty,
                         float error);

    void logWarningImbalance(float imbalance);
    void logErrorOvercurrent();

  private:
    // Helper methods for consistent formatting
    void printSeparator();
};

#endif // CURRENT_LOGGER_H
