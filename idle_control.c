//idle_control.c
#include "idle_control.h"
#include "ecu_state.h"
#include "sensor_manager.h"
#include "tbw_manager.h"   // untuk throttle DBW
#include "utils.h"         // clamp, HAL_GetTick, dsb

#define IDLE_INTEGRAL_LIMIT  10.0f
#define IDLE_THROTTLE_MIN     2.0f
#define IDLE_THROTTLE_MAX     10.0f
#define IDLE_DEADZONE_RPM     30.0f
#define IDLE_LOOP_PERIOD_MS   20

void IdleControl_Init(IdleControl_t* idle) {
    idle->enabled        = true;
    idle->target_rpm     = IDLE_RPM_TARGET;     // dari ecu_config.h
    idle->base_throttle  = 5.0f;                // nilai awal throttle saat idle
    idle->kp             = 0.01f;
    idle->ki             = 0.0005f;
    idle->integral       = 0.0f;
    idle->last_update_ms = 0;
}

void IdleControl_Update(IdleControl_t* idle, ECU_Core_t* ecu) {
    if (!idle || !ecu || !idle->enabled) return;

    float rpm = ecu->sensors.rpm;
    float tps = ecu->sensors.tps; // nilai 0–100

    // Jika throttle dibuka manual → jangan override
    if (tps > 2.5f) {
        idle->integral = 0.0f;
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((now - idle->last_update_ms) < IDLE_LOOP_PERIOD_MS) return;
    idle->last_update_ms = now;

    float error = idle->target_rpm - rpm;

    // Jangan overkompensasi jika sudah dekat
    if (fabsf(error) < IDLE_DEADZONE_RPM) return;

    // PID Integral (simple)
    idle->integral += error * idle->ki;
    idle->integral = CLAMP(idle->integral, -IDLE_INTEGRAL_LIMIT, IDLE_INTEGRAL_LIMIT);

    float throttle_correction = idle->kp * error + idle->integral;
    float target_throttle = idle->base_throttle + throttle_correction;
    target_throttle = CLAMP(target_throttle, IDLE_THROTTLE_MIN, IDLE_THROTTLE_MAX);

    ecu->controls.target_throttle_idle = target_throttle;

    // Kirim ke actuator DBW
    TBW_SetTargetThrottle(target_throttle);
}
