//sensor calibration.h
#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#include "ecu_types.h"
#include "flash_manager.h"  // Untuk penyimpanan ke flash

// Konfigurasi khusus Yamaha R15 V3
#define NILAI_MIN_TPS      0.5f    // Voltage throttle tertutup (0%)
#define NILAI_MAX_TPS      4.5f    // Voltage throttle terbuka (100%)
#define BATAS_ERROR_TPS    0.2f    // Toleransi kesalahan kalibrasi
#define UKURAN_PAGE_FLASH  256     // Ukuran page flash STM32H7

#pragma pack(push, 1)  // Optimasi packing untuk flash
typedef struct {
    // Kalibrasi TPS
    float tps_voltage_min;   // Voltage saat throttle tertutup
    float tps_voltage_max;   // Voltage saat throttle terbuka
    float tps_scale;         // Faktor skala linier
    uint32_t crc32;
    uint32_t write_count;
    // Kalibrasi MAP (jika ada)
    float map_voltage_min;   // Voltage pada 0 kPa
    float map_voltage_max;   // Voltage pada tekanan maksimal
    // Validasi data
    uint16_t checksum;       // Untuk memverifikasi integritas data
    uint8_t is_calibrated;   // Flag status kalibrasi
} SensorCalibration_t;
#pragma pack(pop)

ECU_Status SensorCalibration_Init(SensorCalibration_t* cal);

ECU_Status SensorCalibration_SimpanKeFlash(SensorCalibration_t* cal, uint8_t sector);

ECU_Status SensorCalibration_MuatDariFlash(SensorCalibration_t* cal, uint8_t sector);

ECU_Status SensorCalibration_KalibrasiNol(SensorCalibration_t* cal, float voltage);


ECU_Status SensorCalibration_KalibrasiSkalaPenuh(SensorCalibration_t* cal, float voltage);

ECU_Status SensorCalibration_Validasi(SensorCalibration_t* cal);

#endif // SENSOR_CALIBRATION_H