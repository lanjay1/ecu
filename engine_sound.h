//engine_sound.h
#pragma once
#include "ecu_core.h"
typedef struct {
    uint16_t lope_rpm_range;   // ±100 RPM dari idle
    uint8_t backfire_probability; // 0-100%
} SoundConfig_t;

void EngineSound_Init(const SoundConfig_t* config);

void Update_Engine_Sound_Effects(ECU_State_t *state);