//tps_calibration.h
#ifndef TPS_CALIBRATION_H
#define TPS_CALIBRATION_H

#include "stm32h7xx_hal.h"
#include "ecu_types.h"

/* Calibration states */
typedef enum {
    TPS_CAL_NOT_DONE,
    TPS_CAL_IN_PROGRESS,
    TPS_CAL_MIN,
    TPS_CAL_MAX,
    TPS_CAL_DONE,
    TPS_CAL_ERROR,
    TPS_CAL_TIMEOUT
} TPS_CalibrationState_t;

/* TPS calibration structure */
typedef struct {
    float min_voltage;       // Voltage at 0% throttle
    float max_voltage;       // Voltage at 100% throttle
    TPS_CalibrationState_t cal_state;
    float last_voltage;      // Last raw voltage reading
    uint8_t checksum;        // Data integrity check
} TPS_Calibration_t;

/* Public API */
ECU_Status TPS_Calibration_Init(TPS_Calibration_t* cal);
ECU_Status TPS_Calibration_Start(TPS_Calibration_t* cal);
ECU_Status TPS_Calibration_Update(TPS_Calibration_t* cal, float current_voltage, uint8_t calibration_mode);
float TPS_GetPositionFiltered(TPS_Calibration_t* cal, ADC_HandleTypeDef* hadc);
ECU_Status TPS_Calibration_SaveToFlash(TPS_Calibration_t* cal, uint8_t sector);

#endif /* TPS_CALIBRATION_H */