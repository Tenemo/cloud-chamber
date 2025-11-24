#ifndef CURRENT_LOGGER_H
#define CURRENT_LOGGER_H

#include <Arduino.h>

class Display; // Forward declaration

enum SystemState {
    STATE_INIT,
    STATE_CALIBRATING,
    STATE_WAITING_FOR_POWER,
    STATE_SOFT_START,
    STATE_RUNNING,
    STATE_ERROR
};

struct CurrentMeasurements {
    float tec1_current;
    float tec2_current;
    float total_current;
    float duty;
    float error;
};

class CurrentLogger {
  public:
    CurrentLogger();

    void logInitialization();
    void logCalibrationStart();
    void logCalibrationResults(float acs1_offset_V, float acs2_offset_V);
    void logWaitingForPower();
    void logPowerDetected(float detected_current);
    void logSoftStartBegin(float target_current_per_tec, float max_duty);
    void logSoftStartComplete();
    void logControlActive();

    void logMeasurements(float target_current, float total_current,
                         float tec1_current, float tec2_current, float duty,
                         float error);

    void logWarningImbalance(float imbalance);
    void logErrorOvercurrent();

    // Process raw current measurements with clamping
    static float clampSmallCurrent(float current, float threshold = 0.1f);

    // Check if measurement values have changed significantly
    bool shouldLogMeasurements(const CurrentMeasurements &measurements,
                               float change_threshold = 0.01f);

    // Display update methods
    void setDisplay(Display *disp, int line_height, int value_x,
                    int value_width, int y_status, int y_tec1, int y_tec2,
                    int y_total, int y_duty, int y_target);
    void initializeDisplay(float target_current_per_tec);
    void updateDisplay(float tec1_current, float tec2_current, float duty,
                       SystemState state, float target_current_per_tec,
                       unsigned long display_interval_ms);

  private:
    // Helper methods for consistent formatting
    void printSeparator();
    void clearValueArea(int y);

    // Previous values for serial logging change detection
    float prev_tec1_current;
    float prev_tec2_current;
    float prev_total_current;
    float prev_duty;

    // Display management
    Display *display;
    bool display_initialized;
    unsigned long last_display_update;

    // Display layout configuration
    int line_height;
    int value_x;
    int value_width;
    int y_status;
    int y_tec1;
    int y_tec2;
    int y_total;
    int y_duty;
    int y_target;

    // Previous display values for memoization
    float prev_display_I1;
    float prev_display_I2;
    float prev_display_total;
    float prev_display_duty;
    float prev_display_target;
    SystemState prev_display_state;
};

#endif // CURRENT_LOGGER_H
