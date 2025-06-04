//map_storage.h
#ifndef MAP_STORAGE_H
#define MAP_STORAGE_H

#include "ecu_type.h"
#include "ecu_config.h"
#include <stdbool.h>
#include <stdint.h>

#define MAP_STORAGE_VERSION     0x01
#define MAP_STORAGE_MAGIC       0xECU5151U

typedef struct {
    uint32_t magic;
    uint8_t version;
    float fuel_map[RPM_BINS][TPS_BINS];
    float ignition_map[RPM_BINS][TPS_BINS];
    float tps_min;
    float tps_max;
    float idle_rpm_target;
    float fuel_map[MAP_SIZE];
    float ign_map[MAP_SIZE];
    float quickshift_cut_time_ms;
    float launch_control_rpm;
    float pit_limiter_rpm;
    float blipper_duration_ms;
    float coolant_fan_on;
    float coolant_fan_off;
    uint32_t checksum;
} ECU_MapData_t;

bool MapStorage_Load(ECU_MapData_t* map);
bool MapStorage_Save(const ECU_MapData_t* map);
void MapStorage_ApplyToSystem(const ECU_MapData_t* map);
void MapStorage_ReadFromSystem(ECU_MapData_t* map);
void MapStorage_SaveAll(void);
void MapStorage_LoadAll(void);

#endif // MAP_STORAGE_H
