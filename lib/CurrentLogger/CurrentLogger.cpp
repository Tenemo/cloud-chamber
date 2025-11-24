#include "CurrentLogger.h"
#include "Display.h"

CurrentLogger::CurrentLogger()
    : prev_tec1_current(-999.0f), prev_tec2_current(-999.0f),
      prev_total_current(-999.0f), prev_duty(-999.0f), display(nullptr),
      display_initialized(false), last_display_update(0), line_height(0),
      value_x(0), value_width(0), y_status(0), y_tec1(0), y_tec2(0), y_total(0),
      y_duty(0), y_target(0), prev_display_I1(-999.0f),
      prev_display_I2(-999.0f), prev_display_total(-999.0f),
      prev_display_duty(-999.0f), prev_display_target(-999.0f),
      prev_display_state(STATE_INIT) {}

void CurrentLogger::printSeparator() { Serial.println(); }

float CurrentLogger::clampSmallCurrent(float current, float threshold) {
    return (current < threshold) ? 0.0f : current;
}

bool CurrentLogger::shouldLogMeasurements(
    const CurrentMeasurements &measurements, float change_threshold) {
    bool should_log = false;

    // Check if any value has changed significantly
    if (abs(measurements.tec1_current - prev_tec1_current) > change_threshold ||
        abs(measurements.tec2_current - prev_tec2_current) > change_threshold ||
        abs(measurements.total_current - prev_total_current) >
            change_threshold ||
        abs(measurements.duty - prev_duty) > change_threshold) {
        should_log = true;
    }

    // Update previous values
    prev_tec1_current = measurements.tec1_current;
    prev_tec2_current = measurements.tec2_current;
    prev_total_current = measurements.total_current;
    prev_duty = measurements.duty;

    return should_log;
}

void CurrentLogger::setDisplay(Display *disp, int line_h, int val_x,
                               int val_width, int y_stat, int y_t1, int y_t2,
                               int y_tot, int y_dut, int y_targ) {
    display = disp;
    line_height = line_h;
    value_x = val_x;
    value_width = val_width;
    y_status = y_stat;
    y_tec1 = y_t1;
    y_tec2 = y_t2;
    y_total = y_tot;
    y_duty = y_dut;
    y_target = y_targ;
}

void CurrentLogger::clearValueArea(int y) {
    if (display) {
        display->fillBox(value_x, y, value_width, line_height, 0x0000);
    }
}

void CurrentLogger::initializeDisplay(float target_current_per_tec) {
    if (!display)
        return;

    display->clear();
    display->printLine("Status:", 0, y_status, 1);
    display->printLine("TEC1:", 0, y_tec1, 1);
    display->printLine("TEC2:", 0, y_tec2, 1);
    display->printLine("Total:", 0, y_total, 1);
    display->printLine("Duty:", 0, y_duty, 1);
    display->printLine("Target:", 0, y_target, 1);
    display_initialized = true;

    // Reset previous values to force first update
    prev_display_I1 = -999.0f;
    prev_display_I2 = -999.0f;
    prev_display_total = -999.0f;
    prev_display_duty = -999.0f;
    prev_display_target = -999.0f;
    prev_display_state = STATE_INIT;
}

void CurrentLogger::updateDisplay(float I1, float I2, float duty,
                                  SystemState state,
                                  float target_current_per_tec,
                                  unsigned long display_interval_ms) {
    if (!display)
        return;

    unsigned long current_time = millis();

    if (current_time - last_display_update < display_interval_ms) {
        return;
    }
    last_display_update = current_time;

    if (!display_initialized) {
        initializeDisplay(target_current_per_tec);
    }

    char buf[32];
    float total = I1 + I2;
    float target = target_current_per_tec * 2.0f;

    // Update status only if changed
    if (state != prev_display_state) {
        clearValueArea(y_status);
        const char *status_text;
        if (state == STATE_WAITING_FOR_POWER) {
            status_text = "WAIT PWR";
        } else if (state == STATE_SOFT_START) {
            status_text = "SOFT START";
        } else {
            status_text = "RUNNING";
        }
        snprintf(buf, sizeof(buf), "%s", status_text);
        display->printLine(buf, value_x, y_status, 1);
        prev_display_state = state;
    }

    // Update TEC1 current only if changed significantly
    if (abs(I1 - prev_display_I1) > 0.01f) {
        clearValueArea(y_tec1);
        snprintf(buf, sizeof(buf), "%.2f A", I1);
        display->printLine(buf, value_x, y_tec1, 1);
        prev_display_I1 = I1;
    }

    // Update TEC2 current only if changed significantly
    if (abs(I2 - prev_display_I2) > 0.01f) {
        clearValueArea(y_tec2);
        snprintf(buf, sizeof(buf), "%.2f A", I2);
        display->printLine(buf, value_x, y_tec2, 1);
        prev_display_I2 = I2;
    }

    // Update total current only if changed significantly
    if (abs(total - prev_display_total) > 0.01f) {
        clearValueArea(y_total);
        snprintf(buf, sizeof(buf), "%.2f A", total);
        display->printLine(buf, value_x, y_total, 1);
        prev_display_total = total;
    }

    // Update duty cycle only if changed significantly
    if (abs(duty - prev_display_duty) > 0.001f) {
        clearValueArea(y_duty);
        snprintf(buf, sizeof(buf), "%.1f%%", duty * 100.0f);
        display->printLine(buf, value_x, y_duty, 1);
        prev_display_duty = duty;
    }

    // Target is constant, only update on first draw
    if (abs(target - prev_display_target) > 0.01f) {
        clearValueArea(y_target);
        snprintf(buf, sizeof(buf), "%.2f A", target);
        display->printLine(buf, value_x, y_target, 1);
        prev_display_target = target;
    }
}

void CurrentLogger::logInitialization() {
    Serial.println("\n=== TEC Controller Initializing ===");
}

void CurrentLogger::logCalibrationStart() {
    Serial.println("Calibrating current sensors...");
}

void CurrentLogger::logCalibrationResults(float acs1_offset_V,
                                          float acs2_offset_V) {
    Serial.print("ACS1 zero offset: ");
    Serial.print(acs1_offset_V, 4);
    Serial.println(" V");
    Serial.print("ACS2 zero offset: ");
    Serial.print(acs2_offset_V, 4);
    Serial.println(" V");
}

void CurrentLogger::logWaitingForPower() {
    Serial.println("Waiting for power supply detection (5% duty test)...");
}

void CurrentLogger::logPowerDetected(float detected_current) {
    Serial.print("Power detected! Initial current: ");
    Serial.print(detected_current, 2);
    Serial.println(" A");
}

void CurrentLogger::logSoftStartBegin(float target_current_per_tec,
                                      float max_duty) {
    Serial.println("Starting soft-start sequence...");
    Serial.print("Target current per TEC: ");
    Serial.print(target_current_per_tec, 2);
    Serial.println(" A");
    Serial.print("Maximum duty cycle: ");
    Serial.print(max_duty * 100.0f, 1);
    Serial.println("%");
}

void CurrentLogger::logSoftStartComplete() {
    Serial.println("Soft-start complete - full power enabled");
}

void CurrentLogger::logControlActive() {
    Serial.println("=== TEC Controller Active ===\n");
}

void CurrentLogger::logMeasurements(float target_current, float total_current,
                                    float tec1_current, float tec2_current,
                                    float duty, float error) {
    Serial.print("Target: ");
    Serial.print(target_current, 2);
    Serial.print("A | Measured: ");
    Serial.print(total_current, 2);
    Serial.print("A | TEC1: ");
    Serial.print(tec1_current, 2);
    Serial.print("A | TEC2: ");
    Serial.print(tec2_current, 2);
    Serial.print("A | Duty: ");
    Serial.print(duty * 100.0f, 1);
    Serial.print("% | Error: ");
    Serial.print(error, 3);
    Serial.println("A");
}

void CurrentLogger::logWarningImbalance(float imbalance) {
    Serial.print("WARNING: Branch current imbalance detected: ");
    Serial.print(imbalance, 2);
    Serial.println("A");
}

void CurrentLogger::logErrorOvercurrent() {
    Serial.println("ERROR: Overcurrent detected - shutting down");
}
