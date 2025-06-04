//launch_control.h
#ifndef LAUNCH_CONTROL_H
#define LAUNCH_CONTROL_H

#include "ecu_types.h"

typedef struct {
    bool active;
    bool enabled;
    float launch_rpm;
    float launch_retard;
    float launch_fuel_add;
    uint32_t activation_time;
    bool clutch_engaged;
    float target_rpm;
    uint8_t backfire_enabled;
} LaunchControl_t;

ECU_Status LaunchControl_Init(LaunchControl_t* lc);
ECU_Status LaunchControl_Activate(LaunchControl_t* lc);
ECU_Status LaunchControl_Deactivate(LaunchControl_t* lc);
#endif // LAUNCH_CONTROL_H