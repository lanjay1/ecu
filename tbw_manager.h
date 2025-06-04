//tbw_manager.h
#pragma once
#include "stm32h7xx_hal.h"
#define TBW_MIN_PULSE_US   1000  // Limit bawah PWM
#define TBW_MAX_PULSE_US   2000  // Limit atas PWM
// Di tbw_manager.h
#define TPS_R15_MIN_VOLTAGE 0.5f   // 0% throttle
#define TPS_R15_MAX_VOLTAGE 4.5f   // 100% throttle
typedef enum {
    TBW_ERROR_NONE = 0,
    TBW_ERROR_OVERFLOW,
    TBW_ERROR_SENSOR,
    TBW_ERROR_INVALID_MODE
} TBW_Error_t;

typedef enum {
    TBW_MODE_COMFORT,   // Respon linear
    TBW_MODE_SPORT,     // Respon agresif (kurva kuadratik)
    TBW_MODE_SPORT_PLUS // Respon instan (eksponensial)
} TBW_Mode;
typedef struct {
    TIM_HandleTypeDef* tim;
    ADC_HandleTypeDef* adc;
    float tps_min_voltage;  // 0.5V untuk R15
    float tps_max_voltage;  // 4.5V untuk R15
    bool is_calibrated;
    void (*error_callback)(TBW_Error_t);
} TBW_Handle_t;


void TBW_Init(TIM_HandleTypeDef *pwm_tim, ADC_HandleTypeDef *tps_adc);
void TBW_SetMode(TBW_Mode mode);
float TBW_GetTPS();  // Baca posisi throttle (0.0 - 1.0)
void TBW_Update(TBW_Handle_t* h, float tps_input, TBW_Mode mode);
  // Proses kontrol throttle

// Callback untuk error
void TBW_SetErrorCallback(void (*callback)(uint8_t error_code));
ECU_Status TBW_Calibrate(TBW_Handle_t* h, float min_voltage, float max_voltage);