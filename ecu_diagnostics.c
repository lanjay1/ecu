//ecu_diagnostics.c
#include "ecu_diagnostics.h"
#include "ecu_config.h"
#include <string.h>

ECU_Status Diagnostics_Init(DiagnosticData_t* diag) {
    if (diag == NULL) return ECU_PARAM_ERROR;
    
    memset(diag, 0, sizeof(DiagnosticData_t));
    return ECU_OK;
}

ECU_Status Diagnostics_Update(DiagnosticData_t* diag, SensorData_t* sensors, ActuatorOutput_t* actuators) {
    // Check sensor validity
    if (sensors->coolant_temp > 120.0f) {
        Diagnostics_AddCode(diag, DIAG_OVERHEAT);
    }
    
    if (sensors->rpm > MAX_RPM * 1.05f) {
        Diagnostics_AddCode(diag, DIAG_OVERREV);
    }
    
    if (sensors->battery_voltage < 9.0f) {
        Diagnostics_AddCode(diag, DIAG_LOW_VOLTAGE);
    }
    
    diag->ecu_uptime++;
    return ECU_OK;
}

static void Diagnostics_AddCode(DiagnosticData_t* diag, DiagnosticCode_t code) {
    if (diag->code_count < 10) {
        diag->active_codes[diag->code_count++] = code;
        diag->error_count++;
    }
}

ECU_Status Diagnostics_GetReport(DiagnosticData_t* diag, char* report, uint16_t len) {
    if (diag == NULL || report == NULL) return ECU_PARAM_ERROR;
    
    char temp[256] = {0};
    snprintf(temp, sizeof(temp), "Uptime: %lu, Errors: %u\nCodes:", diag->ecu_uptime, diag->error_count);
    
    for (uint8_t i = 0; i < diag->code_count; i++) {
        strcat(temp, " ");
        switch(diag->active_codes[i]) {
            case DIAG_OK: strcat(temp, "OK"); break;
            case DIAG_SENSOR_FAILURE: strcat(temp, "SENSOR_FAIL"); break;
            // Tambahkan case lainnya
        }
    }
    
    strncpy(report, temp, len);
    return ECU_OK;
}
ECU_Status Diagnostics_ClearCodes(DiagnosticData_t* diag) {
    if (!diag) return ECU_PARAM_ERROR;
    diag->code_count = 0;
    diag->error_count = 0;
    memset(diag->active_codes, 0, sizeof(diag->active_codes));
    return ECU_OK;
}
