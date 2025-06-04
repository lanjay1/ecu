//backfire_manager.c
#include "backfire_manager.h"
#include "hal_tim.h"       // Untuk HAL_GetTick
#include "injector_driver.h" // Untuk HAL_Injector_Cut()
#include "thresholds.h"

// Default configuration
#define DEFAULT_COOLDOWN_MS      300
#define DEFAULT_RPM_THRESHOLD    3500
#define DEFAULT_TPS_THRESHOLD    2.0f
#define DEFAULT_FUEL_EXTRA       1.5f
#define DEFAULT_RETARD_ANGLE     10.0f


// Inisialisasi modul backfire
void Backfire_Init(BackfireManager_t* bfm) {
    bfm->mode = BACKFIRE_OFF;
    bfm->last_event_time = 0;
    // Menggunakan konstanta dari .h atau nilai default jika .h tidak mendefinisikannya
    bfm->rpm_threshold = DEFAULT_RPM_THRESHOLD;     // Pastikan DEFAULT_* didefinisikan
    bfm->tps_threshold = DEFAULT_TPS_THRESHOLD;
    bfm->fuel_extra = DEFAULT_FUEL_EXTRA;
    bfm->retard_angle = DEFAULT_RETARD_ANGLE;
  
}

void Backfire_Update(BackfireManager_t* bfm, ECU_Core_t* ecu) {
    // 1. Cek jika backfire di-disable dari webserver atau mode OFF
    if (!ecu->controls.decel_backfire_enabled || bfm->mode == BACKFIRE_OFF) {
        return;
    }

    // 2. Cek kondisi sensor
    uint32_t current_time = HAL_GetTick();
    if (ecu->sensors.tps < bfm->tps_threshold && 
        ecu->sensors.rpm > bfm->rpm_threshold &&
        (current_time - bfm->last_event_time) > bfm->cooldown_ms) 
        if (ecu->sensors.engine_temp < 70.0f) {
        bfm->mode = BACKFIRE_OFF;
        return;
    }
        // 3. Trigger backfire berdasarkan mode
        switch (bfm->mode) {
            case BACKFIRE_SOFT:
                ecu->actuators.fuel_injection += bfm->fuel_extra * 0.5f; // 50% fuel extra
                ecu->actuators.ignition_advance -= bfm->retard_angle * 0.5f; // 5° retard
                break;

            case BACKFIRE_AGGRESSIVE:
                ecu->actuators.fuel_injection += bfm->fuel_extra; // 100% fuel extra
                ecu->actuators.ignition_advance -= bfm->retard_angle; // 10° retard
                
                break;

            default:
                break;
        }

        // 4. Update  trigger
        bfm->last_event_time = current_time;
}