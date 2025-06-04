//ecu_state.h
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "ecu_config.h" // Tambahkan include untuk menggunakan konstanta dari config

typedef enum {
    GEAR_NEUTRAL = 0,
    GEAR_1,
    GEAR_2,
    GEAR_3,
    GEAR_4,
    GEAR_5,
    GEAR_6,
    GEAR_INVALID = 0xFF
} GearPosition_t;

typedef struct {
    // Data Sensor (8 byte)
    uint16_t rpm;              // 0-MAX_RPM_R15V3 RPM
    uint8_t tps;               // 0-100% dengan resolusi 0.5%
    uint16_t engine_temp;      // °C x10 (range: 0-150.0°C)
    
    // Sistem Transmisi (2 byte)
    GearPosition_t current_gear;
    GearPosition_t prev_gear;
    
    // Status Sistem (Bit-field, 1 byte)
    struct {
        bool gear_change_detected : 1;  // Deteksi perubahan gigi
        bool engine_running : 1;        // Mesin hidup/mati
        bool vva_active : 1;            // Status VVA (Variable Valve Actuation)
        bool quick_shift_enabled : 1;   // Mode quick shift aktif
        bool auto_blip_enabled : 1;     // Fitur auto blip aktif
        bool launch_control_active : 1; // Launch control sedang berjalan
        bool pit_limiter_active : 1;    // Pit speed limiter aktif
        bool gear_sensor_fault : 1;     // Fault sensor gigi
    };
    uint8_t active_profile; 
    // Drive-by-Wire (2 byte)
    uint8_t tbw_target_pos;    // Posisi throttle target (0-100%)
    uint8_t tbw_actual_pos;    // Posisi throttle aktual (0-100%)
    
    // Tambahan untuk R15 V3 (4 byte)
    uint16_t fuel_consumption; // Konsumsi bahan bakar (mL/min)
    uint8_t gear_ratio;        // Rasio gigi saat ini
    uint8_t pad;               // Padding untuk alignment
} ECU_State_t;

extern volatile ECU_State_t ecu_state;  // volatile untuk akses multi-thread