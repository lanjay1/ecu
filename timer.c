//timer.c
#include "timer.h"
#include "stm32h7xx_hal.h"

uint32_t millis(void) {
    return HAL_GetTick();
}