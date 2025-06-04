//fuel_system.c
#include "fuel_system.h"

const uint16_t rpm_axis [NUM_RPM_AXIS] = (1000, 2000, 3000, 4000, 5000, 6000};

const uint8_t tps_axis [NUM_TPS_AXIS] = {0, 10, 25, 50, 75, 100};

const float fuel_map [NUM_RPM_AXIS] [NUM_TPS_AXIS] = {

{1.0, 1.1, 1.2, 1.3, 1.4, 1.5},

{1.1, 1.2, 1.3, 1.4, 1.5, 1.6}, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7}, {1.3, 1.4, 1.5, 1.6, 1.7, 1.8}, {1.4, 1.5, 1.6, 1.7, 1.8, 1.9}, {1.5, 1.6, 1.7, 1.8, 1.9, 2.0} };
float get_fuel_value(uint16_t rpm, uint16_t tps) { uint8_t i_rpm = 0; uint8_t i_tps = 0;
// Cari index RPM
for (uint8_t i = 0; i < RPM_POINTS - 1; i++) { if (rpm_axis[i] <= rpm && rpm < rpm_axis [i + 1]) { i_rpm = i;break;
}
}
// Cari index TPS
for (uint8_t i = 0; i <
TPS_POINTS 1; i++) {
if (tps_axis[i] <= tps && tps < tps_axis [i + 1]) { i_tps = i;break;}
}
uint16_t rpm1 = rpm_axis[i_rpm];
uint16_t rpm2 = rpm_axis[i_rpm + 1];
uint8_t  tps1 = tps_axis[i_tps];
uint8_t  tps2 = tps_axis[i_tps + 1];

// Rasio interpolasi
float rpm_ratio = (float)(rpm - rpm1) / (rpm2 - rpm1);
float tps_ratio = (float)(tps - tps1) / (tps2 - tps1);

float Q11 = fuel_map[i_rpm][i_tps];
float Q21 = fuel_map[i_rpm + 1][i_tps];
float Q12 = fuel_map[i_rpm][i_tps + 1];
float Q22 = fuel_map[i_rpm + 1][i_tps + 1];

// Interpolasi bilinear
float interpolated =
    Q11 * (1 - rpm_ratio) * (1 - tps_ratio) +
    Q21 * rpm_ratio       * (1 - tps_ratio) +
    Q12 * (1 - rpm_ratio) * tps_ratio +
    Q22 * rpm_ratio       * tps_ratio;

return interpolated;


float FuelSystem_Calculate(FuelSystem_t* fs, const SensorData_t* sensors, float dt) {
    // 1. Parameter validation
    if (!fs || !sensors || dt <= 0 || dt > 0.1f) {  // dt max 100ms
        return 0.0f;  // Nilai default aman
    }

    // Auto correction udara tipis berdasarkan tekanan MAP
if (sensors->map_sensor < 80.0f) {
    fs->baro_correction = 0.90f;  // Kurangi BBM 10%
} else {
    fs->baro_correction = 1.0f;
    base_pw *= fs->iat_correction / 100.0f;
}
// DFC: Matikan injeksi saat gas ditutup di RPM tinggi
if (sensors->tps < 2.0f && sensors->rpm > 3500 && sensors->vehicle_speed > 10.0f) {
    base_pw = 0.0f;  // Potong bahan bakar total
}


    // 2. Base fuel calculation with boundary checks
    float base_pw = Get_Base_Pulse(sensors->rpm, sensors->tps);
    base_pw = CLAMP(base_pw, 0.0f, MAX_INJECTION_TIME_MS);

    // 3. Apply compensations
    base_pw *= Interpolate_Curve(fs->temp_comp_curve, sensors->coolant_temp);
    base_pw *= fs->baro_correction;

    // 4. Transient fuel (jika aktif)
    if (fs->enable_transient) {
        base_pw += fs->transient.wall_fuel;
    }

    // 5. Closed-loop adjustment (jika O2 sensor aktif)
    if (fs->closed_loop.enabled && sensors->o2_voltage > 0.1f) {
        base_pw *= (1.0f + fs->closed_loop.learned_correction);
    }

    // 6. Final clamping untuk keamanan
    return CLAMP(base_pw, MIN_INJECTION_TIME_MS, MAX_INJECTION_TIME_MS);
}
void FuelSystem_ImportMaps(FuelSystem_t* fs, const float rpm_tps_map[][11], const float temp_comp[]) {
    // Convert RPMxTPS map to normalized base map
    for (int rpm = 0; rpm < FUEL_MAP_RPM_BINS; rpm++) {
        for (int load = 0; load < FUEL_MAP_LOAD_BINS; load++) {
            fs->base_map[rpm][load] = rpm_tps_map[rpm][load] / MAX_INJECTION_TIME_MS;
        }
    }
    
    // Copy temperature compensation
    memcpy(fs->temp_comp_curve, temp_comp, sizeof(fs->temp_comp_curve));
}
void FuelSystem_UpdateTransient(FuelSystem_t* fs, const Sensors_t* sensors) {
    static float last_tps = 0.0f;
    float delta_tps = sensors->throttle_position - last_tps;
    last_tps = sensors->throttle_position;

    if (!fs->enable_transient) return;

    // Threshold to ignore small fluctuations
    const float threshold = 2.0f;

    if (delta_tps > threshold) {
        // Acceleration enrichment
        fs->transient.wall_fuel += delta_tps * fs->transient.intake_wet_factor;
    } else if (delta_tps < -threshold) {
        // Deceleration enleanment
        fs->transient.wall_fuel -= -delta_tps * fs->transient.intake_wet_factor * 0.5f;
    }

    // Apply natural decay
    fs->transient.wall_fuel *= fs->transient.evaporation_rate;

    // Clamp to safe range
    if (fs->transient.wall_fuel < 0.0f) {
        fs->transient.wall_fuel = 0.0f;
    } else if (fs->transient.wall_fuel > 1.0f) {
        fs->transient.wall_fuel = 1.0f;
    }
}
void FuelSystem_UpdateClosedLoop(FuelSystem_t* fs, const Sensors_t* sensors, float afr_actual) {
    if (!fs->closed_loop.enabled) return;

    float target_afr = fs->closed_loop.target_afr;
    float error = target_afr - afr_actual;

    float correction = PID_Compute(&fs->closed_loop.pid, error);
    fs->closed_loop.learned_correction += correction;

    // Clamp to ±20%
    if (fs->closed_loop.learned_correction > 0.2f)
        fs->closed_loop.learned_correction = 0.2f;
    else if (fs->closed_loop.learned_correction < -0.2f)
        fs->closed_loop.learned_correction = -0.2f;
}
