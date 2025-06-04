//engine_protection.h
#ifndef ENGINE_PROTECTION_H
#define ENGINE_PROTECTION_H

#include "ecu_types.h"

typedef struct {
    bool rev_limiter_active;
    bool overtemp_protection;
    bool low_voltage_cutoff;
    float max_rpm;
    float max_coolant_temp;
    float min_battery_voltage;
} EngineProtection_t;

ECU_Status EngineProtection_Init(EngineProtection_t* prot);
ECU_Status EngineProtection_Update(EngineProtection_t* prot, SensorData_t* sensors, ActuatorOutput_t* actuators);
ECU_Status EngineProtection_SetLimits(EngineProtection_t* prot, float max_rpm, float max_temp, float min_voltage);
ECU_Status EngineProtection_GetState(EngineProtection_t* prot, char* state_str, uint16_t len);

#endif // ENGINE_PROTECTION_H