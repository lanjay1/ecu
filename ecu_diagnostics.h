//ecu_diagnostics.h
#ifndef ECU_DIAGNOSTICS_H
#define ECU_DIAGNOSTICS_H

#include "ecu_types.h"

typedef enum {
    DIAG_OK = 0,
    DIAG_SENSOR_FAILURE,
    DIAG_ACTUATOR_FAILURE,
    DIAG_COMM_FAILURE,
    DIAG_OVERHEAT,
    DIAG_OVERREV,
    DIAG_LOW_VOLTAGE
} DiagnosticCode_t;

typedef struct {
    DiagnosticCode_t active_codes[10];
    uint8_t code_count;
    uint32_t ecu_uptime;
    uint16_t error_count;
} DiagnosticData_t;

ECU_Status Diagnostics_Init(DiagnosticData_t* diag);
ECU_Status Diagnostics_Update(DiagnosticData_t* diag, SensorData_t* sensors, ActuatorOutput_t* actuators);
ECU_Status Diagnostics_GetReport(DiagnosticData_t* diag, char* report, uint16_t len);
ECU_Status Diagnostics_ClearCodes(DiagnosticData_t* diag);

#endif // ECU_DIAGNOSTICS_H