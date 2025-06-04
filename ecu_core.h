//ecu_core.h
#ifndef ECU_CORE_H
#define ECU_CORE_H

#include "ecu_types.h"
#include "tbw_manager.h"
#include "gear_manager.h"
#include "fuel_ignition.h"
#include "launch_control.h"
#include "quick_shift.h"
#include "pit_limiter.h"
#include "engine_protection.h"

typedef struct {
    /* Sensor Inputs */
    struct {
        uint16_t rpm;
        float tps;              // 0-100%
        float engine_temp;      // °C
        float map_pressure;     // kPa
        float lambda;           // AFR
        float knock_level;      // 0-1.0
        float gearbox_temp;     // °C
    } sensors;

    /* Actuator Outputs */
    struct {
        float fuel_injection;   // ms
        float ignition_advance; // degrees
        float idle_valve;       // %
        uint8_t fan_control;    // ON/OFF
    } actuators;

    /* Subsystem Managers */
    TBW_Manager_t tbw;
    GearManager_t gear;
    FuelIgnition_t fuel_ignition;
    LaunchControl_t launch_control;
    QuickShift_t quick_shift;
    PitLimiter_t pit_limiter;

    /* Control Flags */
    struct {
        uint8_t tbw_enabled : 1;
        uint8_t quick_shift_enabled : 1;
        uint8_t launch_control_enabled : 1;
        uint8_t pit_limiter_enabled : 1;
        uint8_t idle_lope_enabled : 1;
        uint8_t decel_backfire_enabled : 1;
        uint8_t launch_backfire_enabled : 1;
        uint8_t overrun_enabled : 1;
    } controls;

    /* Protection and Calibration */
    EngineProtection_t protection;
    TPS_Calibration_t tps_cal;
    SensorCalibration_t calibration;
    
    /* Performance Tuning */
    TuningParams_t tuning;
    
    /* Diagnostics */
    DiagnosticData_t diagnostics;
    DataLogger_t logger;

    /* Runtime Info */
    uint32_t run_time;         // ms
    uint16_t error_codes;
    BackfireManager_t backfire;  // Bitmask
} ECU_Core_t;

/* Web Control Commands */
#define WEB_CMD_ENABLE_TBW          "TBW_ON"
#define WEB_CMD_DISABLE_TBW         "TBW_OFF"
#define WEB_CMD_ENABLE_QUICKSHIFT   "QSHIFT_ON"
#define WEB_CMD_DISABLE_QUICKSHIFT  "QSHIFT_OFF"
#define WEB_CMD_ENABLE_LAUNCH       "LAUNCH_ON"
#define WEB_CMD_DISABLE_LAUNCH      "LAUNCH_OFF"
#define WEB_CMD_ENABLE_PITLIMIT     "PITLIM_ON"
#define WEB_CMD_DISABLE_PITLIMIT    "PITLIM_OFF"
#define WEB_CMD_ENABLE_BACKFIRE     "BACKFIRE_ON"
#define WEB_CMD_DISABLE_BACKFIRE    "BACKFIRE_OFF"
#define WEB_CMD_ENABLE_LOPE         "LOPE_ON"
#define WEB_CMD_DISABLE_LOPE        "LOPE_OFF"
#define WEB_CMD_GET_STATUS          "GET_STATUS"
#define WEB_CMD_RESET_ERRORS        "RESET_ERRORS"

/* Function Prototypes */
ECU_Status ECU_Init(ECU_Core_t* ecu);
ECU_Status ECU_Update(ECU_Core_t* ecu);
ECU_Status ECU_GetDiagnostics(ECU_Core_t* ecu, char* diag_str, uint16_t len);
ECU_Status ECU_ProcessWebCommand(ECU_Core_t* ecu, const char* command);

#endif // ECU_CORE_H