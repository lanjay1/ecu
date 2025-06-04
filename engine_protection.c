//engine_protection.c
#include "engine_protection.h"
#include "actuator_manager.h"
#include "gpio.h" // Tambahan untuk kontrol kipas GPIO
#include "ecu.h"
#include "injector_driver.h"
#include "lean_angle.h"

#define FAN_ON_TEMP_C    95.0f
#define FAN_OFF_TEMP_C   90.0f
#define OVERHEAT_TEMP_C  110.0f
bool fail_safe_triggered = false;

ECU_Status EngineProtection_Update(EngineProtection_t* prot, SensorData_t* sensors, ActuatorOutput_t* actuators) {
    // === Rev limiter ===
    if (prot->rev_limiter_active && sensors->rpm > prot->max_rpm) {
        actuators->fuel_injection = 0; // Cut fuel
        actuators->ignition_advance = 0; // Cut spark
    }

    // === Fan Control ===
    if (sensors->coolant_temp > FAN_ON_TEMP_C) {
        HAL_GPIO_WritePin(FAN_RELAY_GPIO_Port, FAN_RELAY_Pin, GPIO_PIN_SET);
    } else if (sensors->coolant_temp < FAN_OFF_TEMP_C) {
        HAL_GPIO_WritePin(FAN_RELAY_GPIO_Port, FAN_RELAY_Pin, GPIO_PIN_RESET);
    }

    // === Over temperature protection ===
    if (prot->overtemp_protection && sensors->coolant_temp > prot->max_coolant_temp) {
        actuators->fan_control = true;
        // Reduce power if needed
    }
    if (coolant_temp > OVERHEAT_LIMIT) {
    ECU_Log("Engine protection active: Overheat");
    EventLogger_Add(&ecu->logger, LOG_WARNING, PROTECT_OVERHEAT, "Coolant temp too high");
}

if (Sensor_IsDisconnected() || ecu_state.temp_coolant > 120.0f) {
    EngineProtection_TriggerFailSafe("Sensor Error or Overheat");
}

    // === Hard overheat cut ===
    if (sensors->coolant_temp >= OVERHEAT_TEMP_C) {
        prot->overtemp_protection = true;
        actuators->fuel_injection = 0;
        actuators->ignition_advance = 0;
        return ECU_OVERHEAT;
    }

    // === Low voltage protection ===
    if (prot->low_voltage_cutoff && sensors->battery_voltage < prot->min_battery_voltage) {
        // Shutdown non-essential systems
    }

    return ECU_OK;
}
void RPMProtection_Update() {
    if (ecu.runtime.rpm >= ecu.settings.engine_rpm_limit) {
        ecu.runtime.rpm_limited = true;
        InjectorDriver_Disable();
    } else {
        ecu.runtime.rpm_limited = false;
        InjectorDriver_Enable();
    }
}
void check_tip_over_protection(void) {
    if (lean_angle_is_dangerous()) {
        ecu_state.fault_flags |= FAULT_TIP_OVER;
        disable_fuel();
        disable_ignition();
    }
}
void EngineProtection_TriggerFailSafe(const char *reason) {
    fail_safe_triggered = true;
    ecu_state.engine_cut = true;
    ecu_state.limiter_active = true;
    EventLogger_LogError("FAILSAFE: %s", reason);
}

bool EngineProtection_IsFailSafeActive(void) {
    return fail_safe_triggered;
}