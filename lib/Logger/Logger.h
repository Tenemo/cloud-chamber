#ifndef LOGGER_H
#define LOGGER_H

#include "DFRobot_GDL.h"
#include <Arduino.h>

enum SystemState {
    STATE_INIT,
    STATE_CALIBRATING,
    STATE_WAITING_FOR_POWER,
    STATE_SOFT_START,
    STATE_RUNNING,
    STATE_ERROR
};

struct DisplayLayout {
    int line_height;
    int value_x;
    int value_width;
    int y_status;
    int y_tec1;
    int y_tec2;
    int y_total;
    int y_duty;
    int y_target;
};

// Helper function to convert state to display text
const char *getStateText(SystemState state);

struct CurrentMeasurements {
    float tec1_current;
    float tec2_current;
    float total_current;
    float duty;
    float error;
};

class Logger {
  public:
    Logger();

    void logInitialization();
    void logCalibrationStart();
    void logCalibrationResults(float acs1_offset_V, float acs2_offset_V);
    void logWaitingForPower();
    void logPowerDetected(float detected_current);
    void logSoftStartBegin();
    void logSoftStartComplete();
    void logControlActive();

    void logMeasurements(float target_current, float total_current,
                         float tec1_current, float tec2_current, float duty,
                         float error);

    void logWarningImbalance(float imbalance);
    void logErrorOvercurrent();

    // Display initialization and control
    void initializeDisplay();
    void showStartupSequence(bool cal_success, float offset1, float offset2);

    // Check if measurement values have changed significantly
    bool shouldLogMeasurements(const CurrentMeasurements &measurements,
                               float change_threshold = 0.01f);
    void initializeDisplayLayout();
    void updateDisplay(float tec1_current, float tec2_current, float duty,
                       SystemState state);

  private:
    // Helper methods for consistent formatting
    void printSeparator();
    void clearValueArea(int y);
    void printLine(const char *text, int x, int y, uint8_t textSize = 1);
    void clearDisplay();
    void fillBox(int x, int y, int w, int h, uint16_t color);

    // Previous values for serial logging change detection
    float prev_tec1_current;
    float prev_tec2_current;
    float prev_total_current;
    float prev_duty;

    // Display management
    DFRobot_ST7735_128x160_HW_SPI *_screen;
    int8_t _backlight;
    bool display_initialized;
    unsigned long last_display_update;

    // Display layout configuration
    DisplayLayout _layout;

    // Previous display values for memoization
    float prev_display_I1;
    float prev_display_I2;
    float prev_display_total;
    float prev_display_duty;
    float prev_display_target;
    SystemState prev_display_state;
};

#endif // LOGGER_H
