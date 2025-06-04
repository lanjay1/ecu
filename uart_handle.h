//uart_handle.h
#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <stdint.h>

void UART_SendFrame(uint8_t cmd, const char* payload);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);


#endif
