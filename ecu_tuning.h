//ecu_tuning.h
#ifndef ECU_TUNING_H
#define ECU_TUNING_H
#define RPM_BINS 12
#define TPS_BINS 8
#include "ecu_types.h"
typedef struct {
    float fuel_base;
    float fuel_rpm_scale;
    float fuel_load_scale;
    float ignition_base;
    float ignition_rpm_advance;
    uint16_t rpm_threshold;  // Minimal RPM untuk trigger (contoh: 3500)
    float tps_threshold;     // Max TPS untuk decel detection (contoh: 2.0%)
    uint32_t injector_cut_duration_soft; // ms (contoh: 50)
    uint32_t injector_cut_duration_aggressive; // ms (contoh: 100)
    float ignition_delay_aggressive; // Derajat (contoh: 15.0)
} TuningParams_t;

ECU_Status Tuning_Init(TuningParams_t* params);
ECU_Status Tuning_SaveToFlash(TuningParams_t* params);
ECU_Status Tuning_LoadFromFlash(TuningParams_t* params);
ECU_Status Tuning_UpdateFromCAN(TuningParams_t* params, uint8_t* can_data);
ECU_Status Tuning_Apply(TuningParams_t* params, FuelIgnition_t* fuel_ignition);
void ECU_Tuning_SetFuelMap(const float map[RPM_BINS][TPS_BINS]);
void ECU_Tuning_GetFuelMap(float map[RPM_BINS][TPS_BINS]);

void ECU_Tuning_SetIgnitionMap(const float map[RPM_BINS][TPS_BINS]);
void ECU_Tuning_GetIgnitionMap(float map[RPM_BINS][TPS_BINS]);

void ECU_Tuning_LoadDefaultMaps(void);
#endif // ECU_TUNING_H