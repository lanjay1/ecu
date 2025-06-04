//pit_limiter.h
#pragma once
#include "ecu_types.h"

typedef struct {
    bool is_active;
    float speed_limit;      // Dalam km/h (misal: 40km/h untuk pit lane)
    float rpm_limit;        // RPM maksimum saat aktif
    uint8_t throttle_cap;   // Maksimal throttle % (misal: 30%)
    bool enabled;
    bool is_active;
    float rpm_limit;
} PitLimiter_t;

// Public API
void PitLimiter_Init(PitLimiter_t* pl);
void PitLimiter_Enable(PitLimiter_t* pl, bool enable);
ECU_Status PitLimiter_Update(PitLimiter_t* pl, float speed, float rpm, float* throttle, ActuatorOutput_t* actuators);
