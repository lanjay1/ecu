//communication_manager.c
#include "thresholds.h"
#include "communication_manager.h"
#include <string.h>
#include "watchdog_manager.h"
#include "data_logger.h"
#include "ecu_tuning.h"
#include "map_storage.h"
#include "dbw_learning.h"
#include "cJSON.h"
#include "ecu_setting.h"
#include "ecu_core.h"
#include "uart_handler.h" // asumsi Anda punya implementasi ini
#include "json_utils.h"  // ParseIgnFuelFromJSON, SendMapAsJSON
#include "settings_handler.h"  // Tambahan untuk command settings
#include "communication_manager.h"

#define UART_STM32 Serial2
#define BAUDRATE 115200
#define RX_PIN 16
#define TX_PIN 17

#define CAN_TIMEOUT_MS 1000
#define CAN_ALIVE_THRESHOLD 3

#define CMD_SET_PROFILE    0x21
#define CMD_RESET_PROFILE  0x22
#define CMD_SET_MAPS       0x01
#define CMD_GET_MAPS       0x02
#define CMD_SAVE_MAPS      0x03
#define CMD_DBW_LEARN      0x04
#define CMD_GET_SETTINGS   0x10
#define CMD_SET_SETTINGS   0x11
#define CMD_SAVE_SETTINGS 0x19
#define CMD_GET_TPS_STATUS 0x20  

extern ECU_Core_t ecu;
extern DBW_Learning_t dbw_learning;
extern ECU_MapData_t map_shadow; // Jika digunakan
 extern float fuelMap[RPM_BINS][TPS_BINS]; // Jika global
 extern float ignMap[RPM_BINS][TPS_BINS];  // Jika global
 extern float afrMap[RPM_BINS][TPS_BINS];  // Jika global
 extern void WDG_ESP32_Sync(void); // Jika ini fungsi yang benar
 extern bool WDG_IsESP32Alive(void);

static uint32_t last_can_activity = 0;
static uint32_t last_rx_timestamp_ms = 0;
static bool can_connected = false;
static uint32_t can_alive_counter = 0;

static CommProtocol_t current_protocol;

static ECU_Status send_uart(uint8_t* data, uint16_t len) {
    return ECU_OK;
}

static ECU_Status recv_uart(uint8_t* buffer, uint16_t len) {
    return ECU_OK;
}

static ECU_Status send_can(uint32_t id, uint8_t* data, uint8_t len) {
    return ECU_OK;
}

static ECU_Status recv_can(uint32_t* id, uint8_t* data, uint8_t* len) {
    return ECU_OK;
}

ECU_Status CommunicationManager_Init(CommProtocol_t protocol) {
    current_protocol = protocol;
    if (protocol == COMM_CAN_500K) return Comm_Configure_CAN(500000);
    return ECU_OK;
}

ECU_Status CommunicationManager_SendData(uint8_t* data, uint16_t len) {
    if (current_protocol == COMM_UART_115K) return send_uart(data, len);
    else if (current_protocol == COMM_CAN_500K) return Comm_Send_CAN(CAN_ID_ECU_DATA, data, len);
    return ECU_ERROR;
}

ECU_Status CommunicationManager_ReceiveData(uint8_t* buffer, uint16_t len) {
    if (current_protocol == COMM_UART_115K) return recv_uart(buffer, len);
    else if (current_protocol == COMM_CAN_500K) {
        uint32_t id;
        uint8_t rx_len;
        return Comm_Receive_CAN(&id, buffer, &rx_len);
    }
    return ECU_ERROR;
}

ECU_Status Comm_Send_CAN(uint32_t id, uint8_t* data, uint8_t len) {
    return send_can(id, data, len);
}

ECU_Status Comm_Receive_CAN(uint32_t* id, uint8_t* data, uint8_t* len) {
    return recv_can(id, data, len);
}

ECU_Status Comm_Configure_CAN(uint32_t baudrate) {
    return ECU_OK;
}

void CommManager_HealthCheck(void) {
    if ((HAL_GetTick() - last_can_activity) > CAN_TIMEOUT_MS) {
        can_alive_counter++;
        if (can_alive_counter >= CAN_ALIVE_THRESHOLD) {
            Comm_LostConnection(COMM_CAN, &ecu);
        }
    } else {
        can_alive_counter = 0;
    }

    if (!WDG_IsESP32Alive()) {
        Comm_LostConnection(COMM_ESP32, &ecu);
    }
}

void Comm_LostConnection(CommDevice_t device, ECU_Core_t* ecu) {
    switch (device) {
        case COMM_CAN:
            DataLogger_LogError("CAN communication lost");
            ECU_SetOperationalMode(ECU_MODE_LIMP_HOME);
            break;
        case COMM_ESP32:
            DataLogger_LogError("ESP32 heartbeat timeout");
            if (ecu) {
                ecu->controls.telemetry_enabled = false;
            }
            break;
    }
}

void Comm_ProcessCANMessage(void) {
    last_can_activity = HAL_GetTick();
    can_alive_counter = 0;
}

// ==== Handler UART Command JSON ====
void CommunicationManager_HandleUARTCommand(uint8_t cmd, const char* payload_json) {
    switch (cmd) {
        case CMD_SET_MAPS: {
            float ign[RPM_BINS][TPS_BINS];
            float fuel[RPM_BINS][TPS_BINS];
            if (ParseIgnFuelFromJSON(payload_json, ign, fuel)) {
                ECU_Tuning_SetIgnitionMap(ign);
                ECU_Tuning_SetFuelMap(fuel);
            }
            break;
        }
        case CMD_GET_MAPS: {
            ECU_MapData_t map;
            ECU_Tuning_GetIgnitionMap(map.ignition_map);
            ECU_Tuning_GetFuelMap(map.fuel_map);
            SendMapAsJSON(&map);
            break;
        }
        case CMD_SAVE_MAPS: {
            ECU_MapData_t map;
            ECU_Tuning_GetIgnitionMap(map.ignition_map);
            ECU_Tuning_GetFuelMap(map.fuel_map);
            MapStorage_Save(&map);
            break;
        }
        case CMD_SET_PROFILE: {
    int profile = atoi(payload_json);
    if (profile >= 0 && profile < MAP_PROFILE_COUNT) {
        ecu_state.active_profile = profile;
        ecu_config.last_profile = profile;
        ECU_Config_SaveToFlash();
    }
    break;
}

case CMD_RESET_PROFILE: {
    int profile = atoi(payload_json);
    if (profile >= 0 && profile < MAP_PROFILE_COUNT) {
        ECU_Setting_ResetSingleProfile(profile);
    }
    break;
}

        case CMD_DBW_LEARN: {
            DBW_Learning_Start(&dbw_learning);
            break;
        }
        case CMD_GET_SETTINGS:
    Comm_SendSettingsJSON(&ecu.settings);
    break;
case CMD_SET_SETTINGS:
    Comm_UpdateSettingsFromJSON(payload_json, &ecu.settings);
    break;

        case CMD_SAVE_SETTINGS: {
            Handle_Settings_Command(cmd, payload_json);
            break;
    case CMD_GET_TPS_STATUS: {
        float voltage = TPS_GetRawVoltage();
        float min = ecu.settings.tps_min;
        float max = ecu.settings.tps_max;
        float percent = (voltage - min) / (max - min);
        if (percent < 0.0f) percent = 0.0f;
        if (percent > 1.0f) percent = 1.0f;


        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "type", "tps_status");
        cJSON_AddNumberToObject(root, "voltage", voltage);
        cJSON_AddNumberToObject(root, "min", min);
        cJSON_AddNumberToObject(root, "max", max);

        char* json_str = cJSON_PrintUnformatted(root);
        UART_SendFrame(CMD_GET_TPS_STATUS, json_str);
        cJSON_free(json_str);
        cJSON_Delete(root);
        break;
    }

        }
        case WEB_CMD_SET_CUT_DURATION:
    ShiftControl_setCutDuration(payload_json[0]); // asumsikan kirim byte tunggal
    break;

case WEB_CMD_SET_BLIP_DURATION:
    ShiftControl_setBlipDuration(payload[0]);
    break;

        default:
            DataLogger_LogError("Unknown UART command received");
            break;
    }
}
void Comm_SendSettingsJSON(ECU_Settings_t* s) {
    cJSON* root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "qsRpm", s->quickshift_rpm_min);
    cJSON_AddNumberToObject(root, "qsCut", s->quickshift_cut_time_ms);
    cJSON_AddNumberToObject(root, "qsSens", s->quickshift_sensitivity);

    cJSON_AddNumberToObject(root, "fanOn", s->coolant_fan_on);
    cJSON_AddNumberToObject(root, "fanOff", s->coolant_fan_off);

    cJSON_AddNumberToObject(root, "engineLimit", s->engine_rpm_limit);
    cJSON_AddNumberToObject(root, "pitLimit", s->pit_limiter_rpm);
    cJSON_AddNumberToObject(root, "launchRpm", s->launch_control_rpm);
    cJSON_AddNumberToObject(root, "launchDelay", s->launch_delay_ms);

    cJSON_AddBoolToObject(root, "launchEnable", s->launch_control_enable);
    cJSON_AddBoolToObject(root, "pitEnable", s->pit_limiter_enable);

    cJSON_AddNumberToObject(root, "iatTemp", s->intake_temp);
    cJSON_AddNumberToObject(root, "iatCorr", s->intake_correction);

    char* str = cJSON_PrintUnformatted(root);
    Comm_SendJSON(str);
    cJSON_Delete(root);
    free(str);
}
void Comm_UpdateSettingsFromJSON(const char* json, ECU_Settings_t* s) {
    cJSON* root = cJSON_Parse(json);
    if (!root) return;

    // Quickshifter
    s->quickshift_rpm_min       = cJSON_GetFloat(root, "qsRpm", s->quickshift_rpm_min);
    s->quickshift_cut_time_ms   = cJSON_GetFloat(root, "qsCut", s->quickshift_cut_time_ms);
    s->quickshift_sensitivity   = cJSON_GetFloat(root, "qsSens", s->quickshift_sensitivity);

    // Coolant fan control
    s->coolant_fan_on           = cJSON_GetFloat(root, "fanOn", s->coolant_fan_on);
    s->coolant_fan_off          = cJSON_GetFloat(root, "fanOff", s->coolant_fan_off);

    // Engine limits
    s->engine_rpm_limit         = cJSON_GetFloat(root, "engineLimit", s->engine_rpm_limit);
    s->pit_limiter_rpm          = cJSON_GetFloat(root, "pitLimit", s->pit_limiter_rpm);
    s->launch_control_rpm       = cJSON_GetFloat(root, "launchRpm", s->launch_control_rpm);
    s->launch_delay_ms          = cJSON_GetFloat(root, "launchDelay", s->launch_delay_ms);

    // Enable flags
    s->launch_control_enable    = cJSON_GetBool(root, "launchEnable", s->launch_control_enable);
    s->pit_limiter_enable       = cJSON_GetBool(root, "pitEnable", s->pit_limiter_enable);

    // Intake temperature correction
    s->intake_temp              = cJSON_GetFloat(root, "iatTemp", s->intake_temp);
    s->intake_correction        = cJSON_GetFloat(root, "iatCorr", s->intake_correction);

    cJSON_Delete(root);
}
void CommManager_UpdateActivity(void) {
    // Menyinkronkan status bahwa ESP32 aktif
    WDG_ESP32_Sync();

    ecu_state.comm.esp32_alive = true;
}
