// json_utils.c
#include "json_utils.h"
#include "ecu_tuning.h"
#include "cJSON.h"
#include <string.h>
#include "uart_handler.h"

bool ParseIgnFuelFromJSON(const char* json_str, float ign_map[RPM_BINS][TPS_BINS], float fuel_map[RPM_BINS][TPS_BINS]) {
    cJSON* root = cJSON_Parse(json_str);
    if (!root) return false;

    cJSON* ign = cJSON_GetObjectItem(root, "ign");
    cJSON* fuel = cJSON_GetObjectItem(root, "fuel");
    if (!cJSON_IsArray(ign) || !cJSON_IsArray(fuel)) {
        cJSON_Delete(root);
        return false;
    }

    for (int i = 0; i < RPM_BINS; i++) {
        cJSON* row_ign = cJSON_GetArrayItem(ign, i);
        cJSON* row_fuel = cJSON_GetArrayItem(fuel, i);
        if (!cJSON_IsArray(row_ign) || !cJSON_IsArray(row_fuel)) continue;
    for (int j = 0; j < TPS_BINS; j++) {
    cJSON* ign_val = cJSON_GetArrayItem(row_ign, j);
    cJSON* fuel_val = cJSON_GetArrayItem(row_fuel, j);

    if (ign_val && cJSON_IsNumber(ign_val)) {
        ign_map[i][j] = (float)ign_val->valuedouble;
    } else {
        ign_map[i][j] = 0.0f;  // Atau nilai default lain
    }

    if (fuel_val && cJSON_IsNumber(fuel_val)) {
        fuel_map[i][j] = (float)fuel_val->valuedouble;
    } else {
        fuel_map[i][j] = 0.0f;
    }
}

    }

    cJSON_Delete(root);
    return true;
}

void SendMapAsJSON(const ECU_MapData_t* map) {
    cJSON* root = cJSON_CreateObject();
    cJSON* ign = cJSON_CreateArray();
    cJSON* fuel = cJSON_CreateArray();

    for (int i = 0; i < RPM_BINS; i++) {
        cJSON* row_ign = cJSON_CreateArray();
        cJSON* row_fuel = cJSON_CreateArray();
        for (int j = 0; j < TPS_BINS; j++) {
            cJSON_AddItemToArray(row_ign, cJSON_CreateNumber(map->ignition_map[i][j]));
            cJSON_AddItemToArray(row_fuel, cJSON_CreateNumber(map->fuel_map[i][j]));
        }
        cJSON_AddItemToArray(ign, row_ign);
        cJSON_AddItemToArray(fuel, row_fuel);
    }

    cJSON_AddItemToObject(root, "ign", ign);
    cJSON_AddItemToObject(root, "fuel", fuel);

    char* out = cJSON_PrintUnformatted(root);
    UART_SendFrame(0x02, out);  // CMD_GET_MAPS

    cJSON_free(out);
    cJSON_Delete(root);
}
