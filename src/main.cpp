#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "CurrentLogger.h"
#include "Display.h"
#include "config.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

// Current sensor & H-bridge pins
constexpr int PIN_ACS1 = A1; // GPIO5  - CJMCU #1 (TEC1 branch)
constexpr int PIN_ACS2 = A2; // GPIO6  - CJMCU #2 (TEC2 branch)
constexpr int PIN_RPWM = A0; // GPIO4  - BTS7960 RPWM
constexpr int PIN_L_EN = A4; // GPIO10 - BTS7960 L_EN
constexpr int PIN_R_EN = D9; // GPIO0  - BTS7960 R_EN

// PWM configuration
constexpr int PWM_CHANNEL = 0;
constexpr int PWM_FREQ_HZ = 25000; // 25 kHz
constexpr int PWM_RES_BITS = 10;   // 10-bit resolution (0-1023)

// ACS758 sensor parameters (powered from 3.3V)
constexpr float ADC_REF_V = 3.3f;
constexpr int ADC_MAX = 4095;
constexpr float ACS_SENS = 0.026f; // 26 mV/A at 3.3V supply

// TEC control parameters
constexpr float TARGET_CURRENT_PER_TEC = 4.0f; // Amperes per TEC
constexpr float MAX_DUTY_TEST = 0.60f;         // 60% duty limit
constexpr float MIN_DUTY = 0.0f;
constexpr float MAX_CURRENT_PER_TEC = 4.0f; // Hard limit per TEC
constexpr float CURRENT_TOLERANCE = 0.05f;  // ±50mA acceptable deviation

// PI controller gains (tune these based on system response)
constexpr float KP = 0.02f;          // Proportional gain
constexpr float KI = 0.001f;         // Integral gain
constexpr float INTEGRAL_MAX = 0.2f; // Anti-windup limit

// Power detection parameters
constexpr float DETECTION_DUTY = 0.25f; // 25% duty for power detection
constexpr float DETECTION_CURRENT_THRESHOLD =
    0.2f; // 200mA minimum to detect power

// Soft-start parameters
constexpr unsigned long SOFT_START_DURATION_MS = 5000; // 5 seconds ramp-up
bool soft_start_complete = false;
unsigned long soft_start_begin_time = 0;

// System state tracking
SystemState current_state = STATE_INIT;

// Logger instance
CurrentLogger logger;

// Zero-current offsets (calibrated during setup)
float acs1_offset_V = 0.0f;
float acs2_offset_V = 0.0f;

// Filtered current measurements
float acs1_filtered_A = 0.0f;
float acs2_filtered_A = 0.0f;
constexpr float FILTER_ALPHA = 0.2f;

// PI controller state
float error_integral = 0.0f;
float current_duty = 0.0f;

// Safety monitoring
unsigned long last_control_update = 0;
constexpr unsigned long CONTROL_INTERVAL_MS = 100; // 10Hz control loop
constexpr unsigned long DISPLAY_INTERVAL_MS = 250; // 4Hz display update

// Display layout constants
constexpr int LINE_HEIGHT = 12;
constexpr int VALUE_X = 60;
constexpr int VALUE_WIDTH = 70;
constexpr int Y_STATUS = 0;
constexpr int Y_TEC1 = LINE_HEIGHT * 2;
constexpr int Y_TEC2 = LINE_HEIGHT * 3;
constexpr int Y_TOTAL = LINE_HEIGHT * 4;
constexpr int Y_DUTY = LINE_HEIGHT * 5;
constexpr int Y_TARGET = LINE_HEIGHT * 6;

Display display(TFT_DC, TFT_CS, TFT_RST, LCD_BL);

float readCurrentA(int adcPin, float offsetV) {
    long rawSum = 0;
    const int SAMPLES = 1000;

    for (int i = 0; i < SAMPLES; i++) {
        rawSum += analogRead(adcPin);
    }

    float rawAvg = (float)rawSum / SAMPLES;
    float volts = rawAvg * (ADC_REF_V / ADC_MAX);
    float delta = volts - offsetV;
    float amps = delta / ACS_SENS;

    return amps;
}

void setPwmDuty(float duty01) {
    duty01 = constrain(duty01, MIN_DUTY, MAX_DUTY_TEST);
    uint32_t maxVal = (1u << PWM_RES_BITS) - 1u;
    uint32_t val = uint32_t(duty01 * maxVal + 0.5f);
    ledcWrite(PWM_CHANNEL, val);
}

void calibrateSensors() {
    current_state = STATE_CALIBRATING;

    logger.logCalibrationStart();
    display.clear();
    display.printLine("Calibrating sensors...", 0, 20, 1);
    display.printLine("Please wait", 0, 40, 1);

    delay(100);

    const int N = 100;
    uint32_t sum1 = 0, sum2 = 0;

    for (int i = 0; i < N; ++i) {
        sum1 += analogRead(PIN_ACS1);
        sum2 += analogRead(PIN_ACS2);
        delay(5);
    }

    float adc1_off = sum1 / float(N);
    float adc2_off = sum2 / float(N);
    acs1_offset_V = adc1_off * (ADC_REF_V / ADC_MAX);
    acs2_offset_V = adc2_off * (ADC_REF_V / ADC_MAX);

    logger.logCalibrationResults(acs1_offset_V, acs2_offset_V);

    char buf1[32], buf2[32];
    snprintf(buf1, sizeof(buf1), "ACS1: %.3fV", acs1_offset_V);
    snprintf(buf2, sizeof(buf2), "ACS2: %.3fV", acs2_offset_V);
    display.printLine(buf1, 0, 60, 1);
    display.printLine(buf2, 0, 75, 1);
    display.printLine("Calibration complete", 0, 95, 1);

    delay(2000);
}

void setup() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_task_wdt_init(60, true);
    Serial.begin(115200);
    logger.logInitialization();

    current_state = STATE_INIT;

    display.begin();
    display.clear();
    display.printLine("TEC Controller", 0, 0, 1);
    display.printLine("Initializing...", 0, 15, 1);

    // Configure logger with display
    logger.setDisplay(&display, LINE_HEIGHT, VALUE_X, VALUE_WIDTH, Y_STATUS,
                      Y_TEC1, Y_TEC2, Y_TOTAL, Y_DUTY, Y_TARGET);

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    pinMode(PIN_L_EN, OUTPUT);
    pinMode(PIN_R_EN, OUTPUT);
    digitalWrite(PIN_L_EN, LOW);
    digitalWrite(PIN_R_EN, LOW);

    ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_RPWM, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0);

    analogReadResolution(12);

    delay(1000);

    calibrateSensors();

    logger.logWaitingForPower();

    digitalWrite(PIN_L_EN, HIGH);
    digitalWrite(PIN_R_EN, HIGH);
    digitalWrite(STATUS_LED_PIN, HIGH);

    // Start with detection duty to check for power
    setPwmDuty(DETECTION_DUTY);
    current_state = STATE_WAITING_FOR_POWER;
    last_control_update = millis();

    logger.logControlActive();
}

void loop() {
    unsigned long current_time = millis();

    if (current_time - last_control_update < CONTROL_INTERVAL_MS) {
        return;
    }
    last_control_update = current_time;

    float I1_raw = readCurrentA(PIN_ACS1, acs1_offset_V);
    float I2_raw = readCurrentA(PIN_ACS2, acs2_offset_V);

    acs1_filtered_A =
        FILTER_ALPHA * I1_raw + (1.0f - FILTER_ALPHA) * acs1_filtered_A;
    acs2_filtered_A =
        FILTER_ALPHA * I2_raw + (1.0f - FILTER_ALPHA) * acs2_filtered_A;

    float I1_display = CurrentLogger::clampSmallCurrent(acs1_filtered_A);
    float I2_display = CurrentLogger::clampSmallCurrent(acs2_filtered_A);
    float I_total = I1_display + I2_display;

    float target_total_current = TARGET_CURRENT_PER_TEC * 2.0f;

    // Handle state machine
    if (current_state == STATE_WAITING_FOR_POWER) {
        // Keep detection duty and wait for current
        if (I_total >= DETECTION_CURRENT_THRESHOLD) {
            logger.logPowerDetected(I_total);
            logger.logSoftStartBegin(TARGET_CURRENT_PER_TEC, MAX_DUTY_TEST);
            current_state = STATE_SOFT_START;
            soft_start_begin_time = current_time;
        } else {
            // Stay at detection duty, don't run PI controller
            setPwmDuty(DETECTION_DUTY);
            current_duty = DETECTION_DUTY;
            logger.updateDisplay(I1_display, I2_display, current_duty,
                                 current_state, TARGET_CURRENT_PER_TEC,
                                 DISPLAY_INTERVAL_MS);
            return;
        }
    }

    if (current_state == STATE_SOFT_START) {
        unsigned long elapsed = current_time - soft_start_begin_time;
        if (elapsed < SOFT_START_DURATION_MS) {
            float ramp_fraction = (float)elapsed / SOFT_START_DURATION_MS;
            target_total_current *= ramp_fraction;
        } else {
            soft_start_complete = true;
            current_state = STATE_RUNNING;
            logger.logSoftStartComplete();
        }
    }

    // Check if any individual TEC exceeds its limit
    bool tec_limit_exceeded = (I1_display > MAX_CURRENT_PER_TEC) ||
                              (I2_display > MAX_CURRENT_PER_TEC);

    float error = target_total_current - I_total;

    // If any TEC exceeds limit, force duty reduction
    if (tec_limit_exceeded) {
        // Force negative control output to reduce duty
        float control_output = -0.01f; // Aggressive reduction
        current_duty =
            constrain(current_duty + control_output, MIN_DUTY, MAX_DUTY_TEST);
        // Clear integral to prevent windup
        error_integral = 0.0f;
    } else {
        // Normal PI control
        error_integral += error * (CONTROL_INTERVAL_MS / 1000.0f);
        error_integral = constrain(error_integral, -INTEGRAL_MAX, INTEGRAL_MAX);

        float control_output = KP * error + KI * error_integral;
        current_duty =
            constrain(current_duty + control_output, MIN_DUTY, MAX_DUTY_TEST);
    }

    setPwmDuty(current_duty);

    logger.updateDisplay(I1_display, I2_display, current_duty, current_state,
                         TARGET_CURRENT_PER_TEC, DISPLAY_INTERVAL_MS);

    CurrentMeasurements measurements = {I1_display, I2_display, I_total,
                                        current_duty, error};
    if (logger.shouldLogMeasurements(measurements)) {
        logger.logMeasurements(target_total_current, I_total, I1_display,
                               I2_display, current_duty, error);
    }

    float current_imbalance = abs(I1_display - I2_display);
    if (current_imbalance > 0.5f && I_total > 1.0f) {
        logger.logWarningImbalance(current_imbalance);
    }

    if (I_total > (TARGET_CURRENT_PER_TEC * 2.5f)) {
        logger.logErrorOvercurrent();
        current_state = STATE_ERROR;

        setPwmDuty(0.0f);
        digitalWrite(PIN_L_EN, LOW);
        digitalWrite(PIN_R_EN, LOW);
        digitalWrite(STATUS_LED_PIN, LOW);

        display.clear();
        display.printLine("ERROR: OVERCURRENT", 0, 30, 1);
        display.printLine("System shutdown", 0, 45, 1);

        while (true) {
            // Continue reading and logging current values with voltages
            long rawSum1 = 0, rawSum2 = 0;
            const int SAMPLES = 1000;

            for (int i = 0; i < SAMPLES; i++) {
                rawSum1 += analogRead(PIN_ACS1);
                rawSum2 += analogRead(PIN_ACS2);
            }

            float rawAvg1 = (float)rawSum1 / SAMPLES;
            float rawAvg2 = (float)rawSum2 / SAMPLES;
            float volts1 = rawAvg1 * (ADC_REF_V / ADC_MAX);
            float volts2 = rawAvg2 * (ADC_REF_V / ADC_MAX);

            float I1_error = (volts1 - acs1_offset_V) / ACS_SENS;
            float I2_error = (volts2 - acs2_offset_V) / ACS_SENS;
            float I1_clamped = CurrentLogger::clampSmallCurrent(I1_error);
            float I2_clamped = CurrentLogger::clampSmallCurrent(I2_error);
            float I_total_error = I1_clamped + I2_clamped;

            Serial.print("OVERCURRENT - TEC1: ");
            Serial.print(I1_clamped, 2);
            Serial.print("A (");
            Serial.print(volts1, 3);
            Serial.print("V) | TEC2: ");
            Serial.print(I2_clamped, 2);
            Serial.print("A (");
            Serial.print(volts2, 3);
            Serial.print("V) | Total: ");
            Serial.print(I_total_error, 2);
            Serial.println("A");

            delay(1000);
        }
    }
}
