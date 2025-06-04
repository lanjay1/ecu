//pit_limiter.c
#include "pit_limiter.h"
#include "tbw_manager.h"

void PitLimiter_Init(PitLimiter_t* pl) {
    pl->is_active = false;
    pl->speed_limit = 40.0f;    // Default 40 km/h
    pl->rpm_limit = 5000.0f;    // Batas RPM
    pl->throttle_cap = 30;      // Maksimal 30% throttle
}

void PitLimiter_Enable(PitLimiter_t* pl, bool enable) {
    pl->is_active = enable;
}

ECU_Status PitLimiter_Update(PitLimiter_t* pl, float speed, float rpm, float* throttle, ActuatorOutput_t* actuators) {
    if (!pl || !throttle || !actuators) return ECU_PARAM_ERROR;

    if (pl->enabled) {
        if (speed > pl->speed_limit) {
            *throttle = PIT_THROTTLE_LIMIT;
        }

        if (rpm > PIT_RPM_LIMIT) {
            actuators->ignition_advance -= 10.0f;
            if (actuators->ignition_advance < MIN_ADVANCE) {
                actuators->ignition_advance = MIN_ADVANCE;
            }
            actuators->fuel_cut = true;
        }
    }

    return ECU_OK;
}
