//Ignition_calch.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

// Konfigurasi khusus R15 V3
#define MAX_SPARK_ANGLE       35.0f  // BTDC (sesuai manual)
#define MIN_SPARK_ANGLE       -5.0f  // ATDC (anti-kickback)
#define DWELL_TIME_MS         3.5f   // Untuk coil standar
#define VVA_SWITCH_RPM        7400   // RPM aktivasi VVA
#define OVERREV_RETARD_ANGLE  -8.0f  // Saat >11500 RPM
#define RPM_MAP_SIZE 12 // Sesuaikan dengan definisi aktual
#define TPS_MAP_SIZE 10 // Sesuaikan dengan definisi aktual

typedef struct {
    // Tabel utama
    float advance_map[RPM_MAP_SIZE][TPS_MAP_SIZE]; // Gunakan Makro
    float dwell_map[RPM_MAP_SIZE][TPS_MAP_SIZE];  // Dwell time adjustment
    
    // Koreksi dinamis
    float temp_comp;    // -0.15°/°C saat >100°C
    float volt_comp;    //Koefisien koreksi sudut pengapian berdasarkan voltase aki
    float vva_comp;     // +2° saat VVA aktif
    
    // State
    bool vva_active;    // Status VVA
    uint8_t spark_cut;  // Untuk launch control
} IgnitionSystem_t;

// Public API
void Ignition_Init(IgnitionSystem_t* ign);
float Ignition_CalculateAdvance(IgnitionSystem_t* ign, float rpm, float tps);
float Ignition_CalculateDwell(IgnitionSystem_t* ign, float voltage);
void Ignition_HandleVVA(IgnitionSystem_t* ign, float rpm);