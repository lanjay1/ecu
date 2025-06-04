//fuel_ignition.c
#include "fuel_ignition.h"
#include "engine_protection.h"
#include "hal_tim.h"
#include "injector_driver.h"
#include <stdlib.h> // Untuk fungsi random
#include "ecu_config.h"
#include "thresholds.h"
float ignition_map[RPM_BINS][TPS_BINS];
bool ignition_map_loaded = false;
// FUNGSI UTAMA
// =============================================

// Konversi RPM ke indeks
uint8_t RPM_To_Index(uint16_t rpm) {
    if (rpm < RPM_MIN) return 0;
    if (rpm >= RPM_MAX) return RPM_BINS - 1;
    return (uint8_t)((rpm - RPM_MIN) / 500);
}

// Konversi TPS ke indeks
uint8_t TPS_To_Index(float tps) {
    if (tps <= 0.0f) return 0;
    if (tps >= TPS_MAX_PERCENT) return TPS_BINS - 1;
    return (uint8_t)(tps / 20.0f);  // 20% per bin
}

// Interpolasi Bilinear
float Interpolate_2D(const float map[RPM_BINS][TPS_BINS],
                    uint8_t rpm_idx, uint8_t tps_idx,
                    float rpm, float tps) {
    // Interpolasi RPM
    float rpm_low = RPM_MIN + rpm_idx * 500;
    float rpm_high = rpm_low + 500;
    float rpm_ratio = (rpm - rpm_low) / (rpm_high - rpm_low);
    
    // Interpolasi TPS
    float tps_low = tps_idx * 20.0f;
    float tps_high = tps_low + 20.0f;
    float tps_ratio = (tps - tps_low) / (tps_high - tps_low);
    
    // Interpolasi 4 titik
    float v1 = map[rpm_idx][tps_idx];
    float v2 = map[rpm_idx][tps_idx+1];
    float v3 = map[rpm_idx+1][tps_idx];
    float v4 = map[rpm_idx+1][tps_idx+1];
    
    // Bilinear interpolation
    float i1 = v1 + (v2 - v1) * tps_ratio;
    float i2 = v3 + (v4 - v3) * tps_ratio;
    
    return i1 + (i2 - i1) * rpm_ratio;
}

// Hitung timing pengapian dengan kompensasi
// Hitung timing pengapian dengan interpolasi
float Hitung_Timing_Pengapian(ECU_State_t *state) {
  if (!ignition_map_loaded) {
        ECU_Tuning_GetIgnitionMap(ignition_map);
        ignition_map_loaded = true;
    }
    uint8_t rpm_idx = RPM_To_Index(state->sensors.rpm);
    float tps_filtered = TPS_GetPosition(); // sebelum TPS_Ke_Indeks()
    uint8_t tps_idx = TPS_To_Index(tps_filtered);

    float advance = ignition_map[rpm_idx][tps_idx];

    // Kompensasi suhu mesin (timing mundur jika dingin)
    if (state->sensors.engine_temp < 80.0f) {
        advance -= 2.0f * (80.0f - state->sensors.engine_temp) / 80.0f;
    }

    // Kompensasi knocking
    if (state->sensors.knock_level > 0.5f) {
        advance -= 5.0f * state->sensors.knock_level;
    }

    // Proteksi RPM tinggi
    if (state->sensors.rpm > HIGH_RPM_ZONE) {
        advance -= (state->sensors.rpm - HIGH_RPM_ZONE) / 500.0f;
    }
     // Tambahkan efek lope saat idle
    if (state->controls.idle_lope_enabled && 
        abs(state->sensors.rpm - IDLE_RPM_TARGET) <= IDLE_RPM_RANGE) {
        advance += Get_Idle_Lope_Advance(state->sensors.rpm);
    }

    // Batasi nilai advance
    if (advance > MAX_ADVANCE) advance = MAX_ADVANCE;
    if (advance < MIN_ADVANCE) advance = MIN_ADVANCE;
    if (state->sensors.engine_temp > 105.0f && 
    state->sensors.afr > 15.0f &&
    state->sensors.rpm > 9000) {
    advance -= 10.0f; // Retard besar sebagai proteksi
}


    return advance;
}
// =============================================
// **FUNGSI INTEGRASI UTAMA**
// =============================================

void Update_Sistem_Pengapian(ECU_State_t *state) {
    // Hitung timing normal
    float advance = Hitung_Timing_Pengapian(state);
    Update_Pengapian_Ke_Hardware(advance);
}
void Update_Pengapian_Ke_Hardware(float derajat_advance) {
    // Gunakan timer 32-bit di H7
    uint32_t ticks = (uint32_t)((derajat_advance / 360.0f) * 0xFFFFFFFF);
    HAL_TIM_SetCompare(IGNITION_TIMER, TIM_CHANNEL_1, ticks);
}
void FuelIgnition_update(void) {
    // 1. Update shifting logic (quickshift, blipper, gear)
    ShiftControl_update();

    // 2. Update ignition advance (RPM, throttle, MAP)
    IgnitionAdvance_update();

    // 3. Ambil nilai sensor
    int rpm = get_engine_rpm();
    int throttle = get_throttle_position();
    int map = get_map_sensor();

    // 4. Ambil nilai dari tabel fuel
    int fuel_ms = FuelMap_get_injection_time(rpm, throttle, map);

    // 5. Eksekusi injeksi bahan bakar
    FuelInjector_trigger(fuel_ms);

    // 6. Eksekusi ignition control (kalau belum dikendalikan via advance langsung)
    int ignition_deg = IgnitionAdvance_get_timing();
    Ignition_hw_set_timing(ignition_deg);

    // TODO: Tambah fitur seperti cut-off limiter, enrichment, atau warm-up di sini
}
