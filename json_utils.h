//json_utils.h
#ifndef JSON_UTILS_H
#define JSON_UTILS_H

#include <stdbool.h>
#include "ecu_type.h"
#include "ecu_tuning.h"

bool ParseIgnFuelFromJSON(const char* json_str, float ign_map[RPM_BINS][TPS_BINS], float fuel_map[RPM_BINS][TPS_BINS]);
void SendMapAsJSON(const ECU_MapData_t* map);

#endif
