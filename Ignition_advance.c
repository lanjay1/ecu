//Ignition_advance.c
#include "ignition_types.h"
#include "sensor_data.h"  // For battery voltage reading
#include "thresholds.h"
#include <math.h> // Untuk fabsf()
#include "ecu_config.h" 
#define LOPE_PATTERN_SIZE  4
#define OVERREV_LIMIT      11500
#define VOLTAGE_NOMINAL    12.0f
#define TEMP_COMP_REFERENCE 80.0f // Contoh suhu referensi untuk kompensasi
#define VOLTAGE_NOMINAL_ADV_COMP 13.5f // Contoh voltase nominal untuk kompensasi advance
#define TPS_TO_INDEX_DIVISOR 10.0f
#define RPM_RESOLUTION_DIVISOR_IGN 500 // Sesuaikan dengan resolusi map RPM Anda, misal 500 RPM per bin
#define LOPE_PATTERN_UPDATE_INTERVAL_MS 200

// Private function prototype
static float interpolate_2d(const float map[RPM_MAP_SIZE][TPS_MAP_SIZE], float rpm, float tps);
// Kompensasi dwell time berdasarkan voltase
float Get_Compensated_Dwell(float base_dwell_ms, float voltage) {
    const float NOMINAL_VOLTAGE = 13.5f;
    if (voltage <= 0.1f) { // Hindari pembagian dengan nol atau nilai sangat kecil
        return base_dwell_ms * (NOMINAL_VOLTAGE_DWELL / 0.1f); // Batasi efek kompensasi
    }
    return base_dwell_ms * (NOMINAL_VOLTAGE / voltage);
}

float Ignition_CalculateAdvance(IgnitionSystem_t* ign, float rpm, float tps) 
{
    // 1. Get base advance from map
    float base_advance = interpolate_2d(ign->advance_map, rpm, tps);
    
    // 2. Dynamic corrections
    float final_advance = base_advance
        + ign->temp_comp * (get_coolant_temp() - TEMP_COMP_REFERENCE)  // Koreksi suhu
        + ign->volt_comp * (VOLTAGE_NOMINAL_ADV_COMP - get_battery_voltage());   // Voltage compensation
    
    // 3. VVA compensation if active
    if (ign->vva_active) {
        final_advance += ign->vva_comp;
    }
    
    // 4. Overrev protection
    if (rpm > OVERREV_LIMIT) {
        final_advance += OVERREV_RETARD_ANGLE;
    }
    
    // 5. Clamping to safe limits
    return (final_advance > MAX_SPARK_ANGLE) ? MAX_SPARK_ANGLE : 
           (final_advance < MIN_SPARK_ANGLE) ? MIN_SPARK_ANGLE : final_advance;
}

// 2D interpolation implementation
static float interpolate_2d(const float map[RPM_MAP_SIZE][TPS_MAP_SIZE], float rpm, float tps)
{
    // Hitung posisi indeks dasar
    float rpm_f_idx = (rpm - RPM_MAP_MIN_RPM) / RPM_RESOLUTION_DIVISOR_IGN;
    float tps_f_idx = tps / TPS_TO_INDEX_DIVISOR;

    // Clamp agar tetap dalam batas
    if (rpm_f_idx < 0) rpm_f_idx = 0;
    if (tps_f_idx < 0) tps_f_idx = 0;

    uint8_t rpm_idx = (uint8_t)rpm_f_idx;
    uint8_t tps_idx = (uint8_t)tps_f_idx;

    // Pastikan tidak keluar dari batas interpolasi (butuh rpm_idx + 1 dan tps_idx + 1)
    if (rpm_idx >= RPM_MAP_SIZE - 1) rpm_idx = RPM_MAP_SIZE - 2;
    if (tps_idx >= TPS_MAP_SIZE - 1) tps_idx = TPS_MAP_SIZE - 2;

    // Delta fractional untuk interpolasi
    float rpm_frac = rpm_f_idx - rpm_idx;
    float tps_frac = tps_f_idx - tps_idx;

    // Ambil 4 nilai di sekitarnya
    float v00 = map[rpm_idx][tps_idx];
    float v10 = map[rpm_idx + 1][tps_idx];
    float v01 = map[rpm_idx][tps_idx + 1];
    float v11 = map[rpm_idx + 1][tps_idx + 1];

    // Interpolasi bilinear
    float v0 = v00 + (v10 - v00) * rpm_frac;
    float v1 = v01 + (v11 - v01) * rpm_frac;
    float result = v0 + (v1 - v0) * tps_frac;

    return result;
}

// Lope idle pattern for cammed engines
static const float lope_pattern[LOPE_PATTERN_SIZE] = {12.0f, 8.0f, 15.0f, 10.0f};

float Get_Idle_Lope_Advance(uint16_t rpm) 
{
    static uint8_t pattern_index = 0;
    static uint32_t last_update = 0;
    
    // Only active in idle range
    if (fabsf(rpm - IDLE_RPM_TARGET) > IDLE_RPM_RANGE) {
        return 0.0f;
    }

    // Update pattern every 200ms
    if (HAL_GetTick() - last_update > LOPE_PATTERN_UPDATE_INTERVAL_MS) {
        pattern_index = (pattern_index + 1) % LOPE_PATTERN_SIZE;
        last_update = HAL_GetTick();
    }
    
    return lope_pattern[pattern_index];
}