#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <Arduino.h>

/**
 * @brief Manages ACS758 current sensor reading, calibration, and filtering
 *
 * This class encapsulates the logic for reading current from ACS758 Hall-effect
 * sensors, including zero-current calibration and exponential moving average
 * filtering.
 */
class CurrentSensor {
  public:
    /**
     * @brief Construct a new Current Sensor object
     *
     * @param pin ADC pin number for reading sensor voltage
     * @param adc_ref_voltage ADC reference voltage (typically 3.3V)
     * @param adc_resolution ADC resolution in bits (e.g., 12 for ESP32)
     * @param sensitivity Sensor sensitivity in V/A (e.g., 0.026 for ACS758)
     * @param filter_alpha Exponential filter coefficient (0.0-1.0, higher =
     * faster response)
     */
    CurrentSensor(int pin, float adc_ref_voltage = 3.3f,
                  int adc_resolution = 12, float sensitivity = 0.026f,
                  float filter_alpha = 0.2f);

    /**
     * @brief Calibrate the sensor by measuring zero-current offset
     *
     * @param num_samples Number of samples to average for calibration
     * @return true if calibration successful
     * @return false if calibration failed (e.g., invalid readings)
     */
    bool calibrate(int num_samples = 50);

    /**
     * @brief Read and filter current value
     *
     * @param num_samples Number of samples to average per reading
     * @return float Filtered current in Amperes
     */
    float readCurrent(int num_samples = 100);

    /**
     * @brief Get the filtered current value without taking a new reading
     *
     * @return float Last filtered current in Amperes
     */
    float getFilteredCurrent() const;

    /**
     * @brief Get the calibrated zero-current offset voltage
     *
     * @return float Offset voltage in Volts
     */
    float getOffsetVoltage() const;

    /**
     * @brief Check if sensor is calibrated
     *
     * @return true if calibrate() has been called successfully
     */
    bool isCalibrated() const;

    /**
     * @brief Clamp small currents to zero to reduce noise
     *
     * @param current Current value to clamp
     * @param threshold Threshold below which to return 0.0
     * @return float Clamped current value
     */
    static float clampSmallCurrent(float current, float threshold = 0.1f);

    /**
     * @brief Reset the filter state
     */
    void resetFilter();

  private:
    /**
     * @brief Read raw ADC value averaged over multiple samples
     *
     * @param num_samples Number of samples to average
     * @return float Average ADC voltage in Volts
     */
    float readAverageVoltage(int num_samples);

    int _pin;
    float _adc_ref_voltage;
    int _adc_max_value;
    float _sensitivity;
    float _filter_alpha;
    float _offset_voltage;
    float _filtered_current;
    bool _calibrated;
};

#endif // CURRENT_SENSOR_H
