//ecu_tuning.c
#include "ecu_tuning.h"
#include <string.h>

static float fuel_map[RPM_BINS][TPS_BINS];
static float ignition_map[RPM_BINS][TPS_BINS];

void ECU_Tuning_SetFuelMap(const float map[RPM_BINS][TPS_BINS]) {
    memcpy(fuel_map, map, sizeof(fuel_map));
}

void ECU_Tuning_GetFuelMap(float map[RPM_BINS][TPS_BINS]) {
    memcpy(map, fuel_map, sizeof(fuel_map));
}

void ECU_Tuning_SetIgnitionMap(const float map[RPM_BINS][TPS_BINS]) {
    memcpy(ignition_map, map, sizeof(ignition_map));
}

void ECU_Tuning_GetIgnitionMap(float map[RPM_BINS][TPS_BINS]) {
    memcpy(map, ignition_map, sizeof(ignition_map));
}

void ECU_Tuning_LoadDefaultMaps(void) {
    for (int i = 0; i < RPM_BINS; ++i) {
        for (int j = 0; j < TPS_BINS; ++j) {
            fuel_map[i][j] = 10.0f;        // 10 ms injeksi
            ignition_map[i][j] = 15.0f;    // 15 derajat BTDC
        }
    }
}
void SetIgnitionMapValue(uint8_t x, uint8_t y, float value) {
    if (x < RPM_BINS && y < TPS_BINS) {
        ignition_map[x][y] = value;
    }
}

