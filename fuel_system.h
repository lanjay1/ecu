//fuel_system.h
#pragma once
#include "ecu_types.h"

// Configuration constants
#define FUEL_MAP_RPM_BINS     16
#define FUEL_MAP_LOAD_BINS    16
#define TEMP_COMP_CURVE_SIZE  10

typedef struct {
    // Base fuel mapping
    float base_map[FUEL_MAP_RPM_BINS][FUEL_MAP_LOAD_BINS];  
        float temp_comp_curve[TEMP_COMP_CURVE_SIZE];  // Temperature compensation (0-100°C)
        float baro_correction;    // Barometric pressure compensation factor
        float voltage_comp;       // Battery voltage compensation
    } comp;
    
    // Transient fuel model
    struct {
        float wall_fuel;          // Fuel film mass (mg)
        float evaporation_rate;   // mg/ms
        float intake_wet_factor;  // 0.0-1.0
    } transient;
    
    // Closed-loop control
    struct {
        bool enabled;
        float target_afr;
        float pid_gains[3];      // [P, I, D]
        float learned_correction; // Current correction factor
    } closed_loop;
    
    // Runtime state
    float last_pulse_width;
    uint32_t last_update;
} FuelSystem_t;

// Public API
void FuelSystem_Init(FuelSystem_t* fs);
float FuelSystem_Calculate(FuelSystem_t* fs, const SensorData_t* sensors, float delta_time);
void FuelSystem_UpdateTransient(FuelSystem_t* fs, const Sensors_t* sensors);
void FuelSystem_UpdateClosedLoop(FuelSystem_t* fs, const Sensors_t* sensors, float afr_actual);