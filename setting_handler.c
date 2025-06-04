//settings_handler.c
#include "settings_handler.h"
#include "ecu_settings.h"
#include "cJSON.h"
#include "uart_handler.h"
#include <string.h>

extern ECU_Settings_t ecu_settings;

void Handle_Settings_Command(uint8_t cmd, const char* json) {
    if (cmd == 0x10) { // CMD_GET_SETTINGS
        cJSON* root = cJSON_CreateObject();

        cJSON_AddNumberToObject(root, "quickshift_rpm_min", ecu_settings.quickshift_rpm_min);
        cJSON_AddNumberToObject(root, "quickshift_cut_time_ms", ecu_settings.quickshift_cut_time_ms);
        cJSON_AddNumberToObject(root, "quickshift_sensitivity", ecu_settings.quickshift_sensitivity);

        cJSON_AddNumberToObject(root, "coolant_fan_on", ecu_settings.coolant_fan_on);
        cJSON_AddNumberToObject(root, "coolant_fan_off", ecu_settings.coolant_fan_off);

        cJSON_AddNumberToObject(root, "engine_rpm_limit", ecu_settings.engine_rpm_limit);
        cJSON_AddNumberToObject(root, "pit_limiter_rpm", ecu_settings.pit_limiter_rpm);
        cJSON_AddNumberToObject(root, "launch_control_rpm", ecu_settings.launch_control_rpm);
        cJSON_AddNumberToObject(root, "launch_delay_ms", ecu_settings.launch_delay_ms);
        cJSON_AddNumberToObject(root, "blipper_duration_ms", ecu_settings.blipper_duration_ms);

        cJSON_AddBoolToObject(root, "idle_loop_enabled", ecu_settings.idle_loop_enabled);
        cJSON_AddBoolToObject(root, "close_throttle_backfire", ecu_settings.close_throttle_backfire);
        cJSON_AddBoolToObject(root, "backfire_enabled", ecu_settings.backfire_enabled);

        char* json_out = cJSON_PrintUnformatted(root);
        UART_SendFrame(0x10, json_out); // Reply same CMD

        cJSON_free(json_out);
        cJSON_Delete(root);

    } else if (cmd == 0x11) { // CMD_SET_SETTINGS
        cJSON* root = cJSON_Parse(json);
        if (!root) return;

        #define SET_FLOAT(k, f) if (cJSON_GetObjectItem(root, k)) ecu_settings.f = cJSON_GetObjectItem(root, k)->valuedouble;
        #define SET_BOOL(k, f)  if (cJSON_GetObjectItem(root, k)) ecu_settings.f = cJSON_IsTrue(cJSON_GetObjectItem(root, k));

        SET_FLOAT("quickshift_rpm_min", quickshift_rpm_min);
        SET_FLOAT("quickshift_cut_time_ms", quickshift_cut_time_ms);
        SET_FLOAT("quickshift_sensitivity", quickshift_sensitivity);

        SET_FLOAT("coolant_fan_on", coolant_fan_on);
        SET_FLOAT("coolant_fan_off", coolant_fan_off);
        SET_FLOAT("engine_rpm_limit", engine_rpm_limit);
        SET_FLOAT("pit_limiter_rpm", pit_limiter_rpm);
        SET_FLOAT("launch_control_rpm", launch_control_rpm);
        SET_FLOAT("launch_delay_ms", launch_delay_ms);
        SET_FLOAT("blipper_duration_ms", blipper_duration_ms);

        SET_BOOL("idle_loop_enabled", idle_loop_enabled);
        SET_BOOL("close_throttle_backfire", close_throttle_backfire);
        SET_BOOL("backfire_enabled", backfire_enabled);

        cJSON_Delete(root);

    } else if (cmd == 0x12) { // CMD_SAVE_SETTINGS
        ECU_Settings_SaveToFlash(&ecu_settings);
        MapStorage_SaveAll();  // Simpan fuelMap, ignMap, afrMap ke flash eksternal

    }
}
void SaveSettings(ECU_Settings_t* settings) {
    settings->checksum = CalculateChecksum((uint8_t*)settings, sizeof(ECU_Settings_t) - sizeof(uint32_t));
    Flash_WriteBlock(FLASH_SETTINGS_ADDR, (uint8_t*)settings, sizeof(ECU_Settings_t));
}

