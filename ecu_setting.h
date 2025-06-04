// ecu_settings.h
#ifndef ECU_SETTINGS_H
#define ECU_SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#define ECU_SETTINGS_VERSION 1

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float quickshift_rpm_min;
    float quickshift_cut_time_ms;
    float quickshift_sensitivity;

    float coolant_fan_on;
    float coolant_fan_off;

    float engine_rpm_limit;
    float pit_limiter_rpm;
    float launch_control_rpm;
    float launch_delay_ms;
    float blipper_duration_ms;

    bool idle_loop_enabled;
    bool close_throttle_backfire;
    bool backfire_enabled;
     bool launch_control_enable;
    bool pit_limiter_enable;
    //float intake_temp;         // Optional: jika sensor tersedia
    float intake_correction;   // Untuk AFR vs IAT koreksi

} ECU_Settings_t;

void ECU_Settings_LoadDefaults(ECU_Settings_t* s);
void ECU_Settings_LoadFromFlash(ECU_Settings_t* s);
void ECU_Settings_SaveToFlash(const ECU_Settings_t* s);

#ifdef __cplusplus
}
#endif

#endif // ECU_SETTINGS_H
