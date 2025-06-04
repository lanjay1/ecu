// uart_handler.c
#include "uart_handler.h"
#include "usart.h"  // Sesuaikan dengan file HAL UART Anda
#include <string.h>

void UART_SendFrame(uint8_t cmd, const char* payload) {
    uint8_t header[4];
    uint16_t len = strlen(payload);
    header[0] = 0xAA;            // Start byte
    header[1] = cmd;            // Command ID
    header[2] = (len >> 8) & 0xFF; // Length High
    header[3] = len & 0xFF;        // Length Low

    HAL_UART_Transmit(&huart1, header, 4, HAL_MAX_DELAY);            // Kirim header
    HAL_UART_Transmit(&huart1, (uint8_t*)payload, len, HAL_MAX_DELAY); // Kirim payload JSON
}
