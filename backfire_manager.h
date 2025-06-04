//backfire_manager.h
#ifndef BACKFIRE_MANAGER_H
#define BACKFIRE_MANAGER_H

#include "ecu_core.h"  // Untuk ECU_Core_t dan SensorData_t

// Mode Backfire (diambil dari fuel_ignition.h)
typedef enum {
    BACKFIRE_OFF = 0,
    BACKFIRE_SOFT,      // Pop kecil
    BACKFIRE_AGGRESSIVE // Bang kuat (anti-lag style)
} BackfireMode;

// Konfigurasi Backfire
typedef struct {
    BackfireMode mode;
    uint32_t last_event_time;       // Gantikan waktu_terakhir_backfire
    uint32_t cooldown_ms;     // Gantikan BACKFIRE_COOLDOWN_MS
    float rpm_threshold;            // Gantikan BACKFIRE_RPM_MIN
    float tps_threshold;            // Gantikan BACKFIRE_TPS_MAX
    float fuel_extra;               // Gantikan BACKFIRE_FUEL_EXTRA
    float retard_angle;             // Sudut retardasi (derajat)
} BackfireManager_t;

// Public API
void Backfire_Init(BackfireManager_t* bfm);
void Backfire_Update(BackfireManager_t* bfm, ECU_Core_t* ecu);

#endif