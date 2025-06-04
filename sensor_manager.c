//sensor_manager.c
#include "sensor_manager.h"
#include "adc_manager.h"
#include "tps_calibration.h"
#include "sensor_filter.h"
#include <string.h>
#include "thresholds.h"
#include "ecu.h"
#include "gpio.h"

typedef enum {
    TPS_CAL_MIN = 0,
    TPS_CAL_MAX = 1
} TPS_CalibrationMode_t;

TPS_Calibrate(TPS_CAL_MIN);


/* Filter instances */
static SensorFilter_t rpm_filter;
static SensorFilter_t tps_filter;

/* Internal calibration & status */
static ManajerSensor_t manajer_sensor;

/* Inisialisasi sistem sensor */
ECU_Status SensorManager_Init(void) {
    memset(&manajer_sensor, 0, sizeof(manajer_sensor));

    // Inisialisasi filter
    SensorFilter_Init(&rpm_filter, JENIS_FILTER_RPM, 5, 0.0f);     // Moving Average
    SensorFilter_Init(&tps_filter, FILTER_MOVING_AVG, TPS_FILTER_WINDOW, 0.0f);

    // Kalibrasi awal TPS
    TPS_Calibration_Init();

    manajer_sensor.sensor_rpm_aktif = true;
    manajer_sensor.sensor_tps_aktif = true;

    return ECU_OK;
}

/* Baca semua data sensor (untuk update berkala) */
ECU_Status SensorManager_BacaSemuaData(SensorData_t* data_sensor) {
    if (!data_sensor) return ECU_PARAM_ERROR;

    float rpm_raw = ADC_ReadVoltage(ADC_CHANNEL_RPM);
    float tps_raw = TPS_GetRawVoltage();
    float temp_coolant = ADC_ReadVoltage(ADC_CHANNEL_CLT);
    float temp_air = ADC_ReadVoltage(ADC_CHANNEL_IAT);
    float o2 = ADC_ReadVoltage(ADC_CHANNEL_O2);
    float map = ADC_ReadVoltage(ADC_CHANNEL_MAP);
    float vbat = ADC_ReadVoltage(ADC_CHANNEL_BAT);

    // Filtering
    float rpm_filtered = SensorFilter_Apply(&rpm_filter, rpm_raw);
    float tps_percent = TPS_GetPosition();  // Sudah filtered dan dikalibrasi

    data_sensor->rpm              = rpm_filtered;
    data_sensor->throttle_position = tps_percent * TO_PERCENT_FACTOR;
    data_sensor->coolant_temp     = temp_coolant * TO_PERCENT_FACTOR; // Konversi ke °C jika linear
    data_sensor->intake_air_temp  = temp_air * TO_PERCENT_FACTOR;
    data_sensor->map_sensor       = map * TO_PERCENT_FACTOR;
    data_sensor->oxygen_sensor    = o2;
    data_sensor->battery_voltage  = vbat;

    return ECU_OK;
}

/* Baca RPM saja */
ECU_Status SensorManager_BacaRPM(float* rpm_out) {
    if (!rpm_out) return ECU_PARAM_ERROR;
    float rpm_raw = ADC_ReadVoltage(ADC_CHANNEL_RPM);
    *rpm_out = SensorFilter_Apply(&rpm_filter, rpm_raw);
    return ECU_OK;
}

/* Baca TPS sebagai persentase (0.0 - 100.0) */
ECU_Status SensorManager_BacaPosisiThrottle(float* posisi_throttle) {
    if (!posisi_throttle) return ECU_PARAM_ERROR;
    *posisi_throttle = TPS_GetPosition() * TO_PERCENT_FACTOR;
    return ECU_OK;
}

/* Fitur kalibrasi */
ECU_Status SensorManager_Kalibrasi(uint8_t jenis_sensor) {
    if (jenis_sensor == 0) {
        // RPM biasanya tidak dikalibrasi manual
        manajer_sensor.faktor_kalibrasi_rpm = 1.0f;
        return ECU_OK;
    } else if (jenis_sensor == 1) {
        float voltage = ADC_ReadVoltage(ADC_CHANNEL_TPS);
        return TPS_Calibrate(0); // atau 1 tergantung tahap
    }
    return ECU_PARAM_ERROR;
}
void FanControl_Update(float coolant_temp) {
    if (coolant_temp >= ecu.settings.coolant_fan_on) {
        GPIO_SetFan(true);
    } else if (coolant_temp <= ecu.settings.coolant_fan_off) {
        GPIO_SetFan(false);
    }
}
void SensorManager_update(void) {
    SensorData_t data;
    SensorManager_BacaSemuaData(&data);

    // Optional: Simpan atau push ke buffer global untuk JSON, dsb
}
