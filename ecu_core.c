//ecu_core.c
#include "ecu_core.h"
#include "sensor_manager.h"
#include "actuator_manager.h"
#include "data_logger.h"
#include "ecu_diagnostics.h"
#include "engine_protection.h"
#include <stdio.h>
#include <string.h>
#include "speed_output.h"
#include "w25q128_driver.h"

static bool crank_sync_ok = false;
static uint16_t crank_sync_counter = 0;
#define CKP_SYNC_THRESHOLD 10

// Variabel status operasional ECU
static ECU_OperationalState_t ecu_state = ECU_STATE_INIT;
IdleControl_t idle_control;
StarterManager_t starter_manager;

ECU_Status ECU_Init(ECU_Core_t* ecu) {
    if (ecu == NULL) {
        return ECU_PARAM_ERROR;
    }

    // Inisialisasi semua modul dengan pengecekan error
    ECU_Status status = ECU_OK;
    
    status |= SensorManager_Init();
    status |= ActuatorManager_Init();
    status |= TBW_Manager_Init(&ecu->tbw);
    status |= GearManager_Init(&ecu->gear);
    status |= FuelIgnition_Init(&ecu->fuel_ignition);
    status |= DataLogger_Init();
    status |= Diagnostics_Init();
    IdleControl_Init(&idle_control);
    StarterManager_Init(&starter_manager);
    ECU_MapData_t map;
    SpeedOutput_Init();
    W25Q128_Init();
    W25Q128_Init();
MapStorage_LoadAll();  // Load map dari flash ke RAM

if (MapStorage_Load(&map)) {
    MapStorage_ApplyToSystem(&map);
} else {
    // Fallback ke map default (hardcoded di program)
    ECU_Tuning_LoadDefaultMaps();   // fungsi Anda sendiri
    TPS_SetCalibration(0.5f, 4.5f); // nilai default untuk sensor
    ECU_SetIdleTarget(1300.0f);     // default idle RPM
}

    // Verifikasi semua modul berhasil diinisialisasi
    if (status != ECU_OK) {
        ecu_state = ECU_STATE_ERROR;
        return ECU_INIT_ERROR;
    }

    // Reset data sensor dan aktuator
    memset(&ecu->sensors, 0, sizeof(ecu->sensors));
    memset(&ecu->actuators, 0, sizeof(ecu->actuators));

    // Setel ke mode default
    ecu_state = ECU_STATE_READY;
    ecu->controls.tbw_enabled = 0;           // TBW aktif default
    ecu->controls.quick_shift_enabled = 0;   // Quick Shift aktif
    ecu->controls.launch_control_enabled = 0; // Launch Control nonaktif default
    
    return ECU_OK;
}
ECU_Status ECU_Update(ECU_Core_t* ecu) {
    if (ecu == NULL) {
        return ECU_PARAM_ERROR;
    }

    // Cek status ECU sebelum proses
    if (ecu_state != ECU_STATE_READY && ecu_state != ECU_STATE_RUNNING) {
        return ECU_STATE_ERROR;
    }
    // 1. Baca semua sensor
    SensorManager_ReadAll(&ecu->sensors);
   // idle
    IdleControl_Update(&idle_control, ecu);
    StarterManager_Update(&starter_manager, ecu);
    CrankSync_Update();
ECU_CheckESP32Status();
ECU_LogEvents();
SpeedOutput_Update(SensorData.vehicle_speed_kph);
   

    // 2. Jalankan proteksi mesin
    EngineProtection_Check(&ecu->sensors, &ecu->actuators);

    // 3. Proses modul kontrol (dengan pengecekan flag)
    
    // Throttle-by-Wire (jika aktif)
    if (ecu->controls.tbw_enabled) {
        TBW_Manager_Update(&ecu->tbw, ecu->sensors.throttle_position);
    }
    
    // Gear Manager (selalu aktif)
    GearManager_Update(&ecu->gear, ecu->sensors.rpm);
    
       if (ECU_PowerFail_Detect()) {
    ECU_Setting_SaveAll(); // save ke W25Q128 atau internal flash
    delay_ms(100); // beri waktu sebelum mati
}

    // Quick Shift (jika aktif)
    if (ecu->controls.quick_shift_enabled) {
        QuickShift_Update(&ecu->quick_shift, ecu->sensors.rpm);
    }

    // Sistem bahan bakar dan pengapian
    FuelIgnition_Update(&ecu->fuel_ignition, &ecu->sensors, &ecu->actuators);

    // Backfire saat decel (jika aktif)
   if (ecu->controls.decel_backfire_enabled) {
    Backfire_Update(&ecu->backfire, ecu);  // Fungsi baru dari modul terpisah
}

    // 4. Update aktuator
    ActuatorManager_UpdateAll(&ecu->actuators);

    // 5. Log data untuk diagnosa
    _WriteEntry(&ecu->sensors, &ecu->actuators);

    ecu_state = ECU_STATE_RUNNING;
    return ECU_OK;
}

ECU_Status ECU_SetOperationalMode(ECU_Core_t* ecu, ECU_OperationalMode_t mode) {
    // Validasi parameter
    if (ecu == NULL) {
        DataLogger_LogError("ECU_SetOperationalMode: ECU pointer is NULL");
        return ECU_PARAM_ERROR;
    }
    
    if (mode >= ECU_MODE_COUNT) {
        DataLogger_LogError("ECU_SetOperationalMode: Invalid mode %d", mode);
        return ECU_PARAM_ERROR;
    }

    // Simpan mode sebelumnya untuk deteksi transisi
    static ECU_OperationalMode_t prev_mode = ECU_MODE_NORMAL;
    
    // Jika mode tidak berubah, langsung return
    if (prev_mode == mode) {
        return ECU_OK;
    }

    // Handle transisi mode
    switch(mode) {
        case ECU_MODE_NORMAL:
            ecu->state = ECU_STATE_READY;
            // Nonaktifkan semua fitur performa
            ecu->controls.tbw_enabled = 0;
            ecu->controls.quick_shift_enabled = 0;
            ecu->controls.launch_control_enabled = 0;
            ecu->controls.decel_backfire_enabled = 0;
            ecu->controls.pit_limiter_enabled = 0;
            break;
            
        case ECU_MODE_LIMP_HOME:
            ecu->state = ECU_STATE_LIMP;
            // Nonaktifkan semua fitur performa
            ecu->controls.tbw_enabled = 0;
            ecu->controls.quick_shift_enabled = 0;
            ecu->controls.launch_control_enabled = 0;
            ecu->controls.decel_backfire_enabled = 0;
            ecu->controls.pit_limiter_enabled = 0;
            
            // Setel parameter aman dengan batasan
            ecu->actuators.fuel_injection = MAX(ecu->actuators.fuel_injection * 0.9f, ecu->config.min_fuel_injection);
            ecu->actuators.ignition_advance = MIN(ecu->actuators.ignition_advance - 5.0f, ecu->config.max_ignition_retard);
            break;
            
        case ECU_MODE_DIAGNOSTICS:
            ecu->state = ECU_STATE_DIAG;
            // Nonaktifkan kontrol utama tapi biarkan logging aktif
            ecu->controls.tbw_enabled = 0;
            ecu->controls.quick_shift_enabled = 0;
            ecu->controls.launch_control_enabled = 0;
            break;
            
        default:
            // Seharusnya tidak sampai sini karena sudah divalidasi di awal
            DataLogger_LogError("ECU_SetOperationalMode: Unhandled mode %d", mode);
            return ECU_INTERNAL_ERROR;
    }

    // Log perubahan mode
    DataLogger_LogEvent("ECU mode changed from %d to %d", prev_mode, mode);
    
    // Update previous mode setelah transisi berhasil
    prev_mode = mode;
    
    return ECU_OK;
}

ECU_Status ECU_GetDiagnostics(ECU_Core_t* ecu, char* diag_str, uint16_t len) {
    if (ecu == NULL || diag_str == NULL || len < 256) {
        return ECU_PARAM_ERROR;
    }

    snprintf(diag_str, len, 
             "ECU Status: %s\n"
             "Run Time: %lu ms\n"
             "RPM: %d\n"
             "Throttle: %.1f%%\n"
             "Gear: %d\n"
             "Fuel: %.2f ms\n"
             "Ignition: %.1f° BTDC\n"
             "Engine Temp: %.1f°C\n"
             "Knock: %.1f\n"
             "VVA: %s\n"
             "Active Features:\n"
             " - TBW: %d\n"
             " - QS: %d\n"
             " - LC: %d\n"
             " - Backfire: %d\n"
             "Errors: 0x%04X",
             ECU_StateToString(ecu_state),
             ecu->run_time,
             ecu->sensors.rpm,
             ecu->sensors.tps,
             ecu->gear.current_gear,
             ecu->actuators.fuel_injection,
             ecu->actuators.ignition_advance,
             ecu->sensors.engine_temp,
             ecu->sensors.knock_level,
             ecu->sensors.rpm > VVA_ACTIVATION_RPM ? "ON" : "OFF",
             ecu->controls.tbw_enabled,
             ecu->controls.quick_shift_enabled,
             ecu->controls.launch_control_enabled,
             ecu->controls.decel_backfire_enabled,
             ecu->error_codes);

    return ECU_OK;
}

const char* ECU_StateToString(ECU_OperationalState_t state) {
    static const char* states[] = {
        "INISIALISASI", "SIAP", "BERJALAN", "ERROR", "MODE DARURAT", "DIAGNOSA"
    };
    
    if (state >= ECU_STATE_COUNT) {
        return "TIDAK DIKENAL";
    }
    return states[state];
}
void CrankSync_Update(void) {
    if (SensorData.RPM > 200 && SensorData.CKP_valid) {
        crank_sync_counter++;
        if (crank_sync_counter >= CKP_SYNC_THRESHOLD) {
            crank_sync_ok = true;
        }
    } else {
        crank_sync_counter = 0;
        crank_sync_ok = false;
    }
}

// Fungsi untuk modul lain memeriksa status
bool ECU_IsCrankSynchronized(void) {
    return crank_sync_ok;
}
bool ECU_PowerFail_Detect(void) {
    float voltage = Sensor_GetBatteryVoltage(); // dari ADC
    return (voltage < 9.0f);
}
