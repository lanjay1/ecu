//engine_sound.c
#include "engine_sound.h"
#include "thresholds.h"
static SoundConfig_t sound_config;

void EngineSound_Init(const SoundConfig_t* config) {
    memcpy(&sound_config, config, sizeof(SoundConfig_t));
}

void Update_Engine_Sound_Effects(ECU_State_t *state) {
    // Idle Lope
    if (state->controls.idle_lope_enabled) {
        Apply_Idle_Lope(state);
    }
    
    // Decel Backfire
    if (state->controls.decel_backfire_enabled) {
        Handle_Close_Throttle_Backfire(state);
    }
    
    // Launch Backfire dihandle di launch_control.c
}
// Efek suara khusus R15 V3
// engine_sound.c
void Apply_Idle_Lope(ECU_State_t* state) {
    if(state->rpm < 1200 && sound_config.lope_rpm_range > 0) {
        float result = state->ignition_advance + lope;
if (result > MAX_ADVANCE) result = MAX_ADVANCE;
if (result < MIN_ADVANCE) result = MIN_ADVANCE;
state->ignition_advance = result;

    }
}

void Handle_Close_Throttle_Backfire(ECU_State_t* state) {
    if(state->tps < 0.1f && (rand() % GENERIC_PERCENT_MAX) < sound_config.backfire_probability) {
        // Trigger backfire
    }
}