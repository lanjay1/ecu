// ecu_settings.c
#include "ecu_settings.h"
#include "map_storage.h"
#include <string.h>
#include "thresholds.h"

#define SETTINGS_FLASH_ADDRESS  0x081FF800  // Pastikan tidak konflik dengan sektor lain

void ECU_Settings_LoadDefaults(ECU_Settings_t* s) {
    s->quickshift_rpm_min = 4500.0f;
    s->quickshift_cut_time_ms = 70.0f;
    s->quickshift_sensitivity = 1.8f;

    s->coolant_fan_on = 95.0f;
    s->coolant_fan_off = 90.0f;

    s->engine_rpm_limit = 11000.0f;
    s->pit_limiter_rpm = 4500.0f;
    s->launch_control_rpm = INJECTOR_MAX_RPM;
    s->launch_delay_ms = 300.0f;

    s->blipper_duration_ms = 80.0f;

    s->idle_loop_enabled = true;
    s->close_throttle_backfire = true;
    s->backfire_enabled = true;    s->launch_control_enable = false;
    s->pit_limiter_enable = false;
    //s->intake_temp = 30.0f;
    s->intake_correction = 100.0f;

}

void ECU_Settings_LoadFromFlash(ECU_Settings_t* s) {
    const ECU_Settings_t* stored = (const ECU_Settings_t*) SETTINGS_FLASH_ADDRESS;
    if (stored->quickshift_rpm_min > 0.0f && stored->quickshift_rpm_min < 20000.0f) {
        memcpy(s, stored, sizeof(ECU_Settings_t));
    } else {
        ECU_Settings_LoadDefaults(s);
    }
}

void ECU_Settings_SaveToFlash(const ECU_Settings_t* s) {
    MapStorage_WriteBlock(SETTINGS_FLASH_ADDRESS, (const uint8_t*)s, sizeof(ECU_Settings_t));
}
