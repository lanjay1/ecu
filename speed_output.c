// speed_output.c
#include "ecu_type.h"
#include "main.h"
#include "stm32h7xx_hal.h"

#define SPEED_OUT_GPIO     GPIOB
#define SPEED_OUT_PIN      GPIO_PIN_10
#define PULSES_PER_KM      5000  // Ubah sesuai kalibrasi speedometer OEM

static uint32_t last_toggle_us = 0;
static bool pin_state = false;
static uint16_t wheel_speed = 0;

void SpeedOutput_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = SPEED_OUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SPEED_OUT_GPIO, &GPIO_InitStruct);

    HAL_GPIO_WritePin(SPEED_OUT_GPIO, SPEED_OUT_PIN, GPIO_PIN_RESET);
}

void SpeedOutput_update(uint16_t speed_kph) {
    wheel_speed = speed_kph;

    // Kalibrasi: 6 pulsa per rotasi, 1 rotasi = keliling roda ~1.9m
    // Jadi: pulse per second = (speed_kph * 1000) / 3600 / 1.9 * 6
    float speed_mps = (float)speed_kph / 3.6f;
    float pulses_per_sec = (speed_mps / 1.9f) * 6.0f;
    uint32_t freq = (uint32_t)pulses_per_sec;

    timer_set_frequency(TIMER_SPEED_OUT, freq);
}

