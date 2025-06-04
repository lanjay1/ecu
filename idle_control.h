//idle_control.h
#ifndef IDLE_CONTROL_H
#define IDLE_CONTROL_H

#include "ecu_type.h"
#include "ecu_config.h"
#include "ecu_state.h"

typedef struct {
    bool enabled;
    float target_rpm;
    float base_throttle;   // throttle idle saat mesin normal
    float kp;
    float ki;
    float integral;
    uint32_t last_update_ms;
} IdleControl_t;

void IdleControl_Init(IdleControl_t* idle);
void IdleControl_Update(IdleControl_t* idle, ECU_Core_t* ecu);

#endif // IDLE_CONTROL_H
