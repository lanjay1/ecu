//ecu_config.h
#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H
#define ECU_UPDATE_RATE_HZ        100    // 100Hz update rate
// RPM Configuration
 #define RPM_SOFT_LIMIT 10500
#define RPM_HARD_LIMIT 12000
#define IDLE_RPM_TARGET           1300   // Target RPM idle
 // RPM aktivasi VVA (Variable Valve Actuation)
#define VVA_ON_RPM     6500
#define VVA_OFF_RPM    6200
#define VVA_ON_TPS     40.0f
#define VVA_OFF_TPS    35.0f

#define LAUNCH_RPM_DEFAULT        4000   // RPM default untuk launch control
// ======== STANDARD CONSTANTS ========
#define PIT_THROTTLE_LIMIT     30.0f     // %
#define PIT_RPM_LIMIT          5000.0f   // rpm
#define QS_MIN_TPS             20.0f     // %
#define QS_MIN_RPM             4000.0f   // rpm
#define LAUNCH_THROTTLE_MIN    90.0f     // %
#define MIN_ADVANCE            -5.0f     // derajat BTDC


// Temperature Configuration
#define TEMP_NORMAL_CELSIUS       85.0f    // 85°C suhu operasi normal

#define TEMP_MAX_CELSIUS          115.0f    // 115°C suhu maksimum
#define TEMP_MAX                  1150   // 115.0°C (suhu x10 untuk presisi)

// Fuel & Ignition Configuration
#define DECEL_RPM_THRESHOLD    3000
#define IDLE_RPM_RANGE      200
#define MAX_ADVANCE         50.0f
#define MAX_INJECTION_TIME_MS     20.0   // Maksimum injection time dalam ms
#define MAX_IGNITION_ADVANCE_DEG  45.0   // Maksimum ignition advance

// Launch Control
#define LAUNCH_RETARD_DEG         10.0f  // Derajat pengurangan timing saat launch control
#define LAUNCH_EXTRA_FUEL         1.5f   // Faktor pengkayaan bahan bakar saat launch control
// Timer Configuration
#define IGNITION_TIMER            &htim1  // Gunakan TIM1 untuk ketepatan tinggi
#define INJECTOR_TIMER            &htim8  // TIM8 untuk PWM injektor
#define CPU_CACHE_ALIGN           __attribute__((aligned(32))) // Untuk DMA

// Pin Configuration (sesuaikan dengan hardware Anda)
#define RPM_SENSOR_PIN            GPIO_PIN_0
#define THROTTLE_SENSOR_PIN       GPIO_PIN_1
#define INJECTOR_OUTPUT_PIN       GPIO_PIN_2
#define IGNITION_OUTPUT_PIN       GPIO_PIN_3
#define QUICK_SHIFT_PIN           GPIO_PIN_4
#define GEAR_SENSOR_PIN           GPIO_PIN_5
#define O2_SENSOR_PIN             GPIO_PIN_6
//#define KNOCK_SENSOR_PIN          GPIO_PIN_7
#define ECU_FIRMWARE_VERSION  "1.0.0"

typedef struct {
    uint16_t rpm_limit;
    uint16_t idle_target_rpm;
    uint16_t vva_activation_rpm;
    uint16_t launch_control_rpm;
    float    pit_throttle_limit;
    uint16_t pit_rpm_limit;
    float    launch_retard_deg;
    float    launch_extra_fuel;
    bool     quickshifter_enabled;
    const char* firmware_version;
    uint8_t last_profile;
} ECU_Config_t;

ECU_Config_t* ECU_Config_Get(void);
void ECU_Config_ResetToDefault(void);
void ECU_Config_LoadFromFlash(void);
void ECU_Config_SaveToFlash(void);

#endif // ECU_CONFIG_H