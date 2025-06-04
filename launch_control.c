//launch_control.c
#include "launch_control.h"
#include "ecu_config.h"
#include "thresholds.h"

ECU_Status LaunchControl_Update(LaunchControl_t* lc, 
                              SensorData_t* sensors, 
                              ActuatorOutput_t* actuators,
                              bool clutch_input) {
    if (lc == NULL || sensors == NULL || actuators == NULL) {
        return ECU_PARAM_ERROR;
    }
      if (lc->active_time_ms > 8000) {  // Timeout 8 detik
        lc->is_active = false;
        return ECU_TIMEOUT;
    }
    if (ecu->sensors.rpm > RPM_MAX_R15V3) {
        lc->is_active = false;
        return ECU_RPM_OVERFLOW;

    lc->clutch_engaged = clutch_input;
    
    if (TPS_GetPositionFiltered_Global() > 0.9f)

    // Activation logic
    if (lc->enabled && clutch_input && sensors->throttle_position > LAUNCH_THROTTLE_MIN)
 {
        if (!lc->active) {
            lc->active = true;
            lc->activation_time = HAL_GetTick();
        }
    } else {
        if (lc->active) {
            lc->active = false;
        }
    }

    // Apply launch control effects
    if (lc->active) {
        // RPM limiting
        if (sensors->rpm > lc->launch_rpm) {
            actuators->ignition_advance -= lc->launch_retard;
            actuators->fuel_injection += lc->launch_fuel_add;
        }

        // Timeout after 5 seconds
        if (HAL_GetTick() - lc->activation_time > 5000) {
            lc->active = false;
        }
    }

    return ECU_OK;
}
void Launch_Backfire_Control(LaunchControl_t *launch, ECU_Core_t *core) {
    if (!launch->active) return;

    // Saat RPM launch tercapai
    if (abs(ecu->sensors.rpm - launch->target_rpm) < 200) {
        // Potong pengapian secara acak
        if ((HAL_GetTick() % 300) < 50) { // 50ms cut setiap 300ms
            HAL_Ignition_Cut(50);
            
            // Suntik bahan bakar ekstra
            state->actuators.fuel_injection += 2.0f;
        }
    }
}