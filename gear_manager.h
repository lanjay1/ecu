//gear_manager.h
#ifndef GEAR_MANAGER_H
#define GEAR_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    GEAR_NEUTRAL = 0,
    GEAR_1,
    GEAR_2,
    GEAR_3,
    GEAR_4,
    GEAR_5,
    GEAR_6,
    GEAR_INVALID = 255
} GearPosition;

typedef struct {
    uint16_t gear_thresholds[7];  // 0: Neutral, 1-6: Gear 1-6
    uint16_t hysteresis;          // toleransi perbedaan
    uint8_t filter_strength;      // stabilisasi input
} GearConfig;

void GearManager_Init(const GearConfig* config);
void GearManager_Update(uint16_t sensor_value);
GearPosition GearManager_GetCurrentGear(void);
void GearManager_ForceGear(GearPosition gear);
void GearManager_SetDebugMode(bool enabled);
bool GearManager_IsGearValid(GearPosition gear);

#endif // GEAR_MANAGER_H
