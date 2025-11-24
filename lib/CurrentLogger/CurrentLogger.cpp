#include "CurrentLogger.h"

CurrentLogger::CurrentLogger() {}

void CurrentLogger::printSeparator() { Serial.println(); }

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
