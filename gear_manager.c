//gear_manager.c
#include "gear_manager.h"
#include "ecu_log.h"

static GearPosition current_gear = GEAR_NEUTRAL;
static GearPosition filtered_gear = GEAR_NEUTRAL;
static GearConfig gear_config = {
    .gear_thresholds = {100, 300, 500, 700, 900, 1100, 1300},
    .hysteresis = 20,
    .filter_strength = 3
};
static uint8_t stability_counter = 0;
static bool debug_mode = false;

static GearPosition DetectGear(uint16_t sensor_value) {
    uint16_t min_diff = 0xFFFF;
    GearPosition best_match = GEAR_INVALID;

    for (int i = 0; i <= GEAR_6; ++i) {
        uint16_t ref = gear_config.gear_thresholds[i];
        uint16_t diff = (sensor_value > ref) ? (sensor_value - ref) : (ref - sensor_value);

        if (diff < min_diff) {
            min_diff = diff;
            best_match = (GearPosition)i;
        }
    }

    if (min_diff > gear_config.hysteresis) return GEAR_INVALID;
    return best_match;
}

void GearManager_Init(const GearConfig* config) {
    if (config != 0) gear_config = *config;
    current_gear = GEAR_NEUTRAL;
    filtered_gear = GEAR_NEUTRAL;
    stability_counter = 0;
    if (debug_mode) ECU_LOG_INFO("GearManager initialized");
}

void GearManager_Update(uint16_t sensor_value) {
    GearPosition detected = DetectGear(sensor_value);

    if (detected == GEAR_INVALID) {
        stability_counter = 0;
        return;
    }

    if (detected == filtered_gear) {
        if (stability_counter < gear_config.filter_strength)
            stability_counter++;
    } else {
        filtered_gear = detected;
        stability_counter = 0;
    }

    if (stability_counter >= gear_config.filter_strength && current_gear != filtered_gear) {
        ECU_LOG_INFO("Gear changed from %d to %d", current_gear, filtered_gear);
        current_gear = filtered_gear;
    }
}

GearPosition GearManager_GetCurrentGear(void) {
    return current_gear;
}

void GearManager_ForceGear(GearPosition gear) {
    if (gear <= GEAR_6) {
        current_gear = gear;
        ECU_LOG_WARN("Gear forcibly set to %d", gear);
    }
}

void GearManager_SetDebugMode(bool enabled) {
    debug_mode = enabled;
}

bool GearManager_IsGearValid(GearPosition gear) {
    return (gear >= GEAR_NEUTRAL && gear <= GEAR_6);
}
