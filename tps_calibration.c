//tps_calibration.c
#include "tps_calibration.h"
#include "adc_manager.h"
#include "checksum_manager.h"
#include <string.h>
#include <math.h>

/* Private constants */
#define TPS_FILTER_SIZE         5      // Moving average window size
#define TPS_CAL_TIMEOUT_MS      5000   // Max calibration time
#define TPS_VOLTAGE_TOLERANCE   0.1f   // Allowed voltage variation
#define TPS_MIN_RANGE_VOLTAGE   0.3f   // Minimum required range between min/max

/* Private variables */
static struct {
    float min_voltage;          // Voltage at 0% throttle
    float max_voltage;          // Voltage at 100% throttle
    uint8_t cal_state;          // 0=uncalibrated, 1=min calibrated, 2=fully calibrated
    float last_raw_voltage;     // Last ADC reading
    float filter_buffer[TPS_FILTER_SIZE]; // Filter buffer
    uint8_t filter_index;       // Current filter index
} tps_data;

/* Private function prototypes */
static float ApplyMovingAverage(float new_value);
static bool ValidateVoltage(float voltage);

/**
  * @brief  Initialize TPS calibration module
  */
ECU_Status TPS_Calibration_Init(void) {
    memset(&tps_data, 0, sizeof(tps_data));
    tps_data.min_voltage = TPS_VOLTAGE_MIN;
    tps_data.max_voltage = TPS_VOLTAGE_MAX;

    if (fabs(tps_data.max_voltage - tps_data.min_voltage) < TPS_MIN_RANGE_VOLTAGE) {
        return ECU_CALIBRATION_ERROR;
    }

    return ECU_OK;
}

/**
  * @brief  Calibrate TPS with current voltage reading
  * @param  voltage_type 0 = min, 1 = max
  */
ECU_Status TPS_Calibrate(uint8_t voltage_type) {
    float voltage = ADC_ReadVoltage(ADC_CHANNEL_TPS);
    
    if (!ValidateVoltage(voltage)) {
        return ECU_SENSOR_ERROR;
    }

    if (voltage_type == 0) {
        tps_data.min_voltage = voltage;
        tps_data.cal_state = 1;
    } else if (voltage_type == 1) {
        tps_data.max_voltage = voltage;

        // Validate range
        if (fabs(tps_data.max_voltage - tps_data.min_voltage) < TPS_MIN_RANGE_VOLTAGE) {
            return ECU_CALIBRATION_ERROR;
        }

        tps_data.cal_state = 2;
        extern ECU_Core_t* ecu;
        ecu->settings.tps_min = tps_data.min_voltage;
        ecu->settings.tps_max = tps_data.max_voltage;
        SaveSettings(&ecu->settings);
    } else {
        return ECU_PARAM_ERROR; // voltage_type invalid
    }

    return ECU_OK;
}  
float pos = (filtered_voltage - tps_data.min_voltage) / 
            (tps_data.max_voltage - tps_data.min_voltage);

return CLAMP(pos, 0.0f, 1.0f);



float TPS_GetPosition(void) {
    float voltage = ADC_ReadVoltage(ADC_CHANNEL_TPS);
    tps_data.last_raw_voltage = voltage;

    if (!TPS_IsCalibrated()) return 0.0f;

    // Validate voltage range
    if (voltage < tps_data.min_voltage) return 0.0f;
    if (voltage > tps_data.max_voltage) return 1.0f;

    float filtered_voltage = ApplyMovingAverage(voltage);

    // Hindari pembagian dengan 0
    float range = tps_data.max_voltage - tps_data.min_voltage;
    if (range < TPS_MIN_RANGE_VOLTAGE) return 0.0f;

    return (filtered_voltage - tps_data.min_voltage) / range;
}

float TPS_GetRawVoltage(void) {
    return tps_data.last_raw_voltage;
}

bool TPS_IsCalibrated(void) {
    return (tps_data.cal_state == 2);
}

static float ApplyMovingAverage(float new_value) {
    tps_data.filter_buffer[tps_data.filter_index] = new_value;
    tps_data.filter_index = (tps_data.filter_index + 1) % TPS_FILTER_SIZE;

    float sum = 0;
    for (uint8_t i = 0; i < TPS_FILTER_SIZE; i++) {
        sum += tps_data.filter_buffer[i];
    }

    return sum / TPS_FILTER_SIZE;
}

static bool ValidateVoltage(float voltage) {
    if (voltage < 0.1f || voltage > 4.9f) {
        return false;
    }

    static float last_valid_voltage = 0;
    if (fabs(voltage - last_valid_voltage) > 0.5f) {
        return false;
    }

    last_valid_voltage = voltage;
    return true;
}