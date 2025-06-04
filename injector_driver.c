//injector_driver.c
#include "stm32h7xx_hal.h"
#include "ecu_safety.h"  // Untuk proteksi sistem
#include "ecu_log.h"
#include "thresholds.h"


// Konfigurasi Hardware (Harus disesuaikan dengan board ECU)
#define INJ_PORT           GPIOA
#define INJ_PIN            GPIO_PIN_0
#define INJ_TIM            htim3
#define INJ_CHANNEL        TIM_CHANNEL_1
#define INJ_TIMER_CLK_HZ   84000000  // 84MHz untuk STM32H7

// Parameter Injektor Yamaha R15 V3 (Harus dikalibrasi!)
#define INJ_DEADTIME_US       150     // Diukur dengan oscilloscope
#define INJ_MIN_PULSE_US      1000    // 1ms (minimal opening time)
#define INJ_MAX_PULSE_US      20000   // 20ms (batas aman injektor OEM)
#define INJ_MAX_DUTY_CYCLE    0.85f   // 85% duty cycle maksimal

// Prototipe fungsi privat
static uint32_t _calculate_safe_pulse(float pulse_width_ms, float rpm);

/**
 * @brief Inisialisasi injektor dengan proteksi hardware
 */
void Injector_Init(void) {
  // Konfigurasi timer: prescaler untuk resolusi 1us (84MHz/84 = 1MHz)
  INJ_TIM.Instance->PSC = (INJ_TIMER_CLK_HZ / 1000000) - 1; 
  INJ_TIM.Instance->ARR = 0xFFFF;  // Periode maksimal (16-bit)
  
  HAL_TIM_PWM_Start(&INJ_TIM, INJ_CHANNEL);
  __HAL_TIM_SET_COMPARE(&INJ_TIM, INJ_CHANNEL, 0); // Pastikan injector OFF
}

/**
 * @brief Kontrol injektor dengan proteksi multi-layer
 * @param pulse_width_ms Durasi pulse dalam milidetik
 * @param rpm RPM mesin saat ini (untuk perhitungan duty cycle)
 */
void Injector_Actuate(float pulse_width_ms, float rpm) {
  // Validasi input
  if (pulse_width_ms <= 0 || rpm < 500) return;
  
  // Hitung pulse width yang aman
  uint32_t pulse_us = _calculate_safe_pulse(pulse_width_ms, rpm);
  
  // Trigger PWM
  __HAL_TIM_SET_COMPARE(&INJ_TIM, INJ_CHANNEL, pulse_us);
  
  // Logging untuk debugging
  ECU_Log("INJ Pulse: %lu us | RPM: %.0f", pulse_us, rpm);
}

/**
 * @brief Emergency cut-off dengan watchdog
 * @param duration_ms 0 = cut permanen sampai reset
 */
void Injector_Cut(uint32_t duration_ms) {
  __HAL_TIM_SET_COMPARE(&INJ_TIM, INJ_CHANNEL, 0); // Force OFF
  
  if (duration_ms > 0) {
    // Gunakan timer hardware (bukan HAL_Delay!)
    HAL_TIM_Base_Start_IT(&htim7); // TIM7 sebagai timeout
    __HAL_TIM_SET_AUTORELOAD(&htim7, duration_ms);
  }
}

//--------------------- FUNGSI PRIVAT ---------------------//
static uint32_t _calculate_safe_pulse(float pulse_width_ms, float rpm) {
  // Konversi ke mikrodetik + deadtime
  uint32_t pulse_us = (uint32_t)(pulse_width_ms * MS_TO_US) + INJ_DEADTIME_US;
  
  // 1. Clamp ke batasan absolut
  pulse_us = CLAMP(pulse_us, INJ_MIN_PULSE_US, INJ_MAX_PULSE_US);
  
  // 2. Batasi duty cycle berdasarkan RPM
  float max_duty_pulse = (60000.0f / rpm) * INJ_MAX_DUTY_CYCLE * MS_TO_US;
  if (pulse_us > max_duty_pulse) {
    pulse_us = (uint32_t)max_duty_pulse;
    ECU_TriggerWarning(INJ_DUTY_OVERFLOW); // Notifikasi sistem
  }
  
  return pulse_us;
}
void Injector_Update(...) {
    if (duty_cycle > MAX_DUTY_PERCENT) {
        ECU_Log("Duty overflow: %.1f%%", duty_cycle);
        ECU_TriggerWarning(INJ_DUTY_OVERFLOW);
    }
}