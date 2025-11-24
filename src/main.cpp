#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

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
constexpr float TARGET_CURRENT_PER_TEC = 2.0f; // Amperes per TEC
constexpr float MAX_DUTY_TEST = 0.40f;         // 40% duty limit for testing
constexpr float MIN_DUTY = 0.0f;
constexpr float CURRENT_TOLERANCE = 0.05f; // ±50mA acceptable deviation

// PI controller gains (tune these based on system response)
constexpr float KP = 0.02f;          // Proportional gain
constexpr float KI = 0.001f;         // Integral gain
constexpr float INTEGRAL_MAX = 0.2f; // Anti-windup limit

// Soft-start parameters
constexpr unsigned long SOFT_START_DURATION_MS = 5000; // 5 seconds ramp-up
bool soft_start_complete = false;
unsigned long soft_start_begin_time = 0;

// System state tracking
enum SystemState {
    STATE_INIT,
    STATE_CALIBRATING,
    STATE_SOFT_START,
    STATE_RUNNING,
    STATE_ERROR
};
SystemState current_state = STATE_INIT;

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
unsigned long last_display_update = 0;
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

// Previous display values to detect changes
float prev_I1 = -999.0f;
float prev_I2 = -999.0f;
float prev_total = -999.0f;
float prev_duty = -999.0f;
float prev_target = -999.0f;
SystemState prev_state = STATE_INIT;

Display display(TFT_DC, TFT_CS, TFT_RST, LCD_BL);
bool display_initialized = false;

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

void initializeDisplayLayout() {
    display.clear();
    display.printLine("Status:", 0, Y_STATUS, 1);
    display.printLine("TEC1:", 0, Y_TEC1, 1);
    display.printLine("TEC2:", 0, Y_TEC2, 1);
    display.printLine("Total:", 0, Y_TOTAL, 1);
    display.printLine("Duty:", 0, Y_DUTY, 1);
    display.printLine("Target:", 0, Y_TARGET, 1);
    display_initialized = true;

    // Reset previous values to force first update
    prev_I1 = -999.0f;
    prev_I2 = -999.0f;
    prev_total = -999.0f;
    prev_duty = -999.0f;
    prev_target = -999.0f;
    prev_state = STATE_INIT;
}

void clearValueArea(int y) {
    display.fillBox(VALUE_X, y, VALUE_WIDTH, LINE_HEIGHT, 0x0000);
}

void updateDisplayValues(float I1, float I2, float duty, SystemState state) {
    unsigned long current_time = millis();

    if (current_time - last_display_update < DISPLAY_INTERVAL_MS) {
        return;
    }
    last_display_update = current_time;

    if (!display_initialized) {
        initializeDisplayLayout();
    }

    char buf[32];
    float total = I1 + I2;
    float target = TARGET_CURRENT_PER_TEC * 2.0f;

    // Update status only if changed
    if (state != prev_state) {
        clearValueArea(Y_STATUS);
        const char *status_text =
            (state == STATE_SOFT_START) ? "SOFT START" : "RUNNING";
        snprintf(buf, sizeof(buf), "%s", status_text);
        display.printLine(buf, VALUE_X, Y_STATUS, 1);
        prev_state = state;
    }

    // Update TEC1 current only if changed significantly
    if (abs(I1 - prev_I1) > 0.01f) {
        clearValueArea(Y_TEC1);
        snprintf(buf, sizeof(buf), "%.2f A", I1);
        display.printLine(buf, VALUE_X, Y_TEC1, 1);
        prev_I1 = I1;
    }

    // Update TEC2 current only if changed significantly
    if (abs(I2 - prev_I2) > 0.01f) {
        clearValueArea(Y_TEC2);
        snprintf(buf, sizeof(buf), "%.2f A", I2);
        display.printLine(buf, VALUE_X, Y_TEC2, 1);
        prev_I2 = I2;
    }

    // Update total current only if changed significantly
    if (abs(total - prev_total) > 0.01f) {
        clearValueArea(Y_TOTAL);
        snprintf(buf, sizeof(buf), "%.2f A", total);
        display.printLine(buf, VALUE_X, Y_TOTAL, 1);
        prev_total = total;
    }

    // Update duty cycle only if changed significantly
    if (abs(duty - prev_duty) > 0.001f) {
        clearValueArea(Y_DUTY);
        snprintf(buf, sizeof(buf), "%.1f%%", duty * 100.0f);
        display.printLine(buf, VALUE_X, Y_DUTY, 1);
        prev_duty = duty;
    }

    // Target is constant, only update on first draw
    if (abs(target - prev_target) > 0.01f) {
        clearValueArea(Y_TARGET);
        snprintf(buf, sizeof(buf), "%.2f A", target);
        display.printLine(buf, VALUE_X, Y_TARGET, 1);
        prev_target = target;
    }
}

void calibrateSensors() {
    current_state = STATE_CALIBRATING;

    Serial.println("Calibrating current sensors...");
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

    Serial.print("ACS1 zero offset: ");
    Serial.print(acs1_offset_V, 4);
    Serial.println(" V");
    Serial.print("ACS2 zero offset: ");
    Serial.print(acs2_offset_V, 4);
    Serial.println(" V");

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
    Serial.println("\n=== TEC Controller Initializing ===");

    current_state = STATE_INIT;

    display.begin();
    display.clear();
    display.printLine("TEC Controller", 0, 0, 1);
    display.printLine("Initializing...", 0, 15, 1);

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

    Serial.println("Starting soft-start sequence...");
    Serial.print("Target current per TEC: ");
    Serial.print(TARGET_CURRENT_PER_TEC, 2);
    Serial.println(" A");
    Serial.print("Maximum duty cycle: ");
    Serial.print(MAX_DUTY_TEST * 100.0f, 1);
    Serial.println("%");

    digitalWrite(PIN_L_EN, HIGH);
    digitalWrite(PIN_R_EN, HIGH);
    digitalWrite(STATUS_LED_PIN, HIGH);

    current_state = STATE_SOFT_START;
    soft_start_begin_time = millis();
    last_control_update = millis();
    last_display_update = millis();

    display_initialized = false;

    Serial.println("=== TEC Controller Active ===\n");
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

    float I1_display = (acs1_filtered_A < 0.1f) ? 0.0f : acs1_filtered_A;
    float I2_display = (acs2_filtered_A < 0.1f) ? 0.0f : acs2_filtered_A;
    float I_total = I1_display + I2_display;

    float target_total_current = TARGET_CURRENT_PER_TEC * 2.0f;

    if (current_state == STATE_SOFT_START) {
        unsigned long elapsed = current_time - soft_start_begin_time;
        if (elapsed < SOFT_START_DURATION_MS) {
            float ramp_fraction = (float)elapsed / SOFT_START_DURATION_MS;
            target_total_current *= ramp_fraction;
        } else {
            soft_start_complete = true;
            current_state = STATE_RUNNING;
            Serial.println("Soft-start complete - full power enabled");
        }
    }

    float error = target_total_current - I_total;
    error_integral += error * (CONTROL_INTERVAL_MS / 1000.0f);
    error_integral = constrain(error_integral, -INTEGRAL_MAX, INTEGRAL_MAX);

    float control_output = KP * error + KI * error_integral;
    current_duty =
        constrain(current_duty + control_output, MIN_DUTY, MAX_DUTY_TEST);

    setPwmDuty(current_duty);

    updateDisplayValues(I1_display, I2_display, current_duty, current_state);

    Serial.print("Target: ");
    Serial.print(target_total_current, 2);
    Serial.print("A | Measured: ");
    Serial.print(I_total, 2);
    Serial.print("A | TEC1: ");
    Serial.print(I1_display, 2);
    Serial.print("A | TEC2: ");
    Serial.print(I2_display, 2);
    Serial.print("A | Duty: ");
    Serial.print(current_duty * 100.0f, 1);
    Serial.print("% | Error: ");
    Serial.print(error, 3);
    Serial.println("A");

    float current_imbalance = abs(I1_display - I2_display);
    if (current_imbalance > 0.5f && I_total > 1.0f) {
        Serial.print("WARNING: Branch current imbalance detected: ");
        Serial.print(current_imbalance, 2);
        Serial.println("A");
    }

    if (I_total > (TARGET_CURRENT_PER_TEC * 2.5f)) {
        Serial.println("ERROR: Overcurrent detected - shutting down");
        current_state = STATE_ERROR;

        setPwmDuty(0.0f);
        digitalWrite(PIN_L_EN, LOW);
        digitalWrite(PIN_R_EN, LOW);
        digitalWrite(STATUS_LED_PIN, LOW);

        display.clear();
        display.printLine("ERROR: OVERCURRENT", 0, 30, 1);
        display.printLine("System shutdown", 0, 45, 1);

        while (true) {
            delay(1000);
        }
    }
}
