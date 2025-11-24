#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "Display.h"
#include "config.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

// Current sensor & H-bridge pins
constexpr int PIN_ACS1 = A1; // GPIO5  - CJMCU #1 OU1
constexpr int PIN_ACS2 = A2; // GPIO6  - CJMCU #2 OU1
constexpr int PIN_RPWM = A0; // GPIO4  - BTS7960 RPWM
// constexpr int PIN_LPWM = ; // LPWM is bound to ground
constexpr int PIN_L_EN = A4; // GPIO10 - BTS7960 L_EN
constexpr int PIN_R_EN = D9; // GPIO0  - BTS7960 R_EN

// PWM config
constexpr int PWM_CHANNEL = 0;
constexpr int PWM_FREQ_HZ = 25000; // 25 kHz
constexpr int PWM_RES_BITS = 10;   // 10-bit 0..1023

// ACS758 params (CJMCU-758 powered from 3.3V)
constexpr float ADC_REF_V = 3.3f;  // ESP32 ADC reference
constexpr int ADC_MAX = 4095;      // 12-bit resolution
constexpr float ACS_SENS = 0.026f; // ~26 mV/A at 3.3V (ACS758-050B)

// Will hold zero-current offsets (in volts)
float acs1_offset_V = 0.0f;
float acs2_offset_V = 0.0f;

// Filtered current values for exponential moving average
float acs1_filtered_A = 0.0f;
float acs2_filtered_A = 0.0f;
constexpr float FILTER_ALPHA =
    0.2f; // Filter coefficient (lower = more smoothing)

float readCurrentA(int adcPin, float offsetV) {
    long rawSum = 0;
    // Increase samples to 1000 for better smoothing
    const int SAMPLES = 1000;

    // BURST READ: No delay.
    // This allows us to average out the PWM ripple effectively.
    for (int i = 0; i < SAMPLES; i++) {
        rawSum += analogRead(adcPin);
    }

    float rawAvg = (float)rawSum / SAMPLES;
    float volts = rawAvg * (ADC_REF_V / ADC_MAX);

    // Deadband: If measurement is less than 0.03A (noise floor), ignore it
    float delta = volts - offsetV;
    float amps = delta / ACS_SENS;

    return amps;
}

void setPwmDuty(float duty01) {
    // duty01 in [0.0, 1.0]
    if (duty01 < 0.0f)
        duty01 = 0.0f;
    if (duty01 > 0.95f)
        duty01 = 0.95f;
    uint32_t maxVal = (1u << PWM_RES_BITS) - 1u; // e.g. 1023
    uint32_t val = uint32_t(duty01 * maxVal + 0.5f);
    ledcWrite(PWM_CHANNEL, val);
}

Display display(TFT_DC, TFT_CS, TFT_RST, LCD_BL);

void setup() {
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_task_wdt_init(60, true);
    Serial.begin(115200);
    Serial.println("Initializing...");

    display.printLine("Initializing...", 0, 0, 1);

    pinMode(STATUS_LED_PIN, OUTPUT);

    // H-bridge control pins
    pinMode(PIN_L_EN, OUTPUT);
    pinMode(PIN_R_EN, OUTPUT);
    digitalWrite(PIN_L_EN, LOW); // keep bridge disabled at start
    digitalWrite(PIN_R_EN, LOW);

    // PWM setup on RPWM
    ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_RPWM, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0); // duty = 0

    // ADC setup (12-bit)
    analogReadResolution(12); // 0..4095

    // Zero-current calibration for ACS758s
    delay(100); // let things settle
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
    Serial.print("ACS1 zero offset (V): ");
    Serial.println(acs1_offset_V, 4);
    Serial.print("ACS2 zero offset (V): ");
    Serial.println(acs2_offset_V, 4);

    display.begin();
    display.clear();
    display.printLine("Initialized.", 0, 0, 1);
    Serial.println("Initialized.");

    // Enable H-bridge logic (still 0 duty, so no current)
    digitalWrite(PIN_L_EN, HIGH);
    digitalWrite(PIN_R_EN, HIGH);
    digitalWrite(STATUS_LED_PIN, HIGH);
}

void loop() {
    // Duty sweep for 24V LED strip (18V min = 75% duty, up to 95%)
    static float duty = 0.75f; // Start at 75% (18V)
    static float delta = 0.01f;

    // Set PWM duty
    setPwmDuty(duty);

    // Read raw averaged current values from burst sampling
    float I1_raw = readCurrentA(PIN_ACS1, acs1_offset_V);
    float I2_raw = readCurrentA(PIN_ACS2, acs2_offset_V);

    // Apply exponential moving average filter for temporal smoothing
    acs1_filtered_A =
        FILTER_ALPHA * I1_raw + (1.0f - FILTER_ALPHA) * acs1_filtered_A;
    acs2_filtered_A =
        FILTER_ALPHA * I2_raw + (1.0f - FILTER_ALPHA) * acs2_filtered_A;

    // Apply threshold for display only (don't modify the filter state)
    // We are never measuring negative currents, so clamp to zero
    float I1_display = (acs1_filtered_A < 0.1f) ? 0.0f : acs1_filtered_A;
    float I2_display = (acs2_filtered_A < 0.1f) ? 0.0f : acs2_filtered_A;
    float Itotal = I1_display + I2_display;

    Serial.print("Duty=");
    Serial.print(duty, 3);
    Serial.print(" | I2=");
    Serial.print(I2_display, 3);
    Serial.println(" A");

    delay(200);

    // Update duty for next loop
    duty += delta;
    if (duty > 0.95f) { // Max at 95% (22.8V)
        duty = 0.95f;
        delta = -delta;
    } else if (duty < 0.75f) { // Min at 75% (18V)
        duty = 0.75f;
        delta = -delta;
    }
}
