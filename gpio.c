// gpio.c
#include "gpio.h"
#include "main.h"

void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // === ENABLE CLOCK UNTUK PORT YANG DIGUNAKAN ===
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // === INISIALISASI PIN FAN RELAY ===
    GPIO_InitStruct.Pin = FAN_RELAY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FAN_RELAY_GPIO_Port, &GPIO_InitStruct);

    // Default state OFF
    HAL_GPIO_WritePin(FAN_RELAY_GPIO_Port, FAN_RELAY_Pin, GPIO_PIN_RESET);
}
void GPIO_SetFan(bool on) {
    HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
