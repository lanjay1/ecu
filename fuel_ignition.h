//fuel_ignition.h
#ifndef FUEL_IGNITION_H
#define FUEL_IGNITION_H

#include <stdint.h>
#include "backfire_manager.h"  // Include untuk BackfireMode

// Konfigurasi Ignition Map
#define RPM_MIN        500
#define RPM_MAX        11000
#define RPM_BINS       16
#define TPS_BINS       6

typedef struct {
    struct {
        uint16_t rpm;
        float tps;
        float engine_temp;
        uint8_t gear;
    } sensors;
    
    struct {
        uint8_t idle_lope_enabled : 1;
        // Hapus backfire_mode dari sini
    } controls;
} ECU_State_t;

// Deklarasi fungsi ignition
float Ignition_GetAdvance(ECU_State_t *state);
void Ignition_Update(float advance_deg);

#endif