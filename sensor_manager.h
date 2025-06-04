//sensor_manager.h
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "ecu_types.h"

// Konfigurasi khusus Yamaha R15 V3
#define FREKUENSI_PEMBACAAN_RPM   200     // Frekuensi sampling sensor crank (Hz)
#define BATAS_MIN_RPM             200     // RPM minimum valid
#define BATAS_MAX_RPM             12500   // RPM maksimum valid
#define JENIS_FILTER_RPM          FILTER_MA // Moving Average untuk RPM
#define TPS_FILTER_WINDOW            5

typedef struct {
    // Data Kalibrasi
    float faktor_kalibrasi_rpm;  // Faktor koreksi RPM
    float offset_sensor_tps;     // Koreksi TPS
    
    // Status Sensor
    bool sensor_rpm_aktif;
    bool sensor_tps_aktif;
} ManajerSensor_t;

ECU_Status SensorManager_Init(void);

ECU_Status SensorManager_BacaSemuaData(SensorData_t* data_sensor);

ECU_Status SensorManager_BacaRPM(float* rpm);

ECU_Status SensorManager_BacaPosisiThrottle(float* posisi_throttle);

ECU_Status SensorManager_Kalibrasi(uint8_t jenis_sensor);
void SensorManager_update(void);


#endif // SENSOR_MANAGER_H