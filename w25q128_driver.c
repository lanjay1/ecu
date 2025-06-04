// w25q128_driver.c
#include "stm32h7xx_hal.h"
#include <string.h>

#define W25Q128_CS_GPIO     GPIOA
#define W25Q128_CS_PIN      GPIO_PIN_4
#define W25Q128_CMD_READ_ID 0x9F
#define W25Q128_CMD_READ    0x03
#define W25Q128_CMD_WRITE   0x02
#define W25Q128_CMD_WREN    0x06
#define W25Q128_CMD_SECTOR_ERASE 0x20
#define W25Q128_PAGE_SIZE   256

extern SPI_HandleTypeDef hspi1;

static void W25Q128_CS_Low()  { HAL_GPIO_WritePin(W25Q128_CS_GPIO, W25Q128_CS_PIN, GPIO_PIN_RESET); }
static void W25Q128_CS_High() { HAL_GPIO_WritePin(W25Q128_CS_GPIO, W25Q128_CS_PIN, GPIO_PIN_SET); }

void W25Q128_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = W25Q128_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(W25Q128_CS_GPIO, &GPIO_InitStruct);

    W25Q128_CS_High();
}

uint32_t W25Q128_ReadID(void) {
    uint8_t cmd = W25Q128_CMD_READ_ID;
    uint8_t id[3] = {0};
    W25Q128_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, id, 3, HAL_MAX_DELAY);
    W25Q128_CS_High();
    return (id[0] << 16) | (id[1] << 8) | id[2];
}

void W25Q128_WriteEnable(void) {
    uint8_t cmd = W25Q128_CMD_WREN;
    W25Q128_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    W25Q128_CS_High();
}

void W25Q128_EraseSector(uint32_t addr) {
    uint8_t cmd[4];
    cmd[0] = W25Q128_CMD_SECTOR_ERASE;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    W25Q128_WriteEnable();
    W25Q128_CS_Low();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    W25Q128_CS_High();
    HAL_Delay(400); // tunggu erase selesai
}

void W25Q128_Read(uint32_t addr, uint8_t *buf, uint32_t len) {
    uint8_t cmd[4];
    cmd[0] = W25Q128_CMD_READ;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    W25Q128_CS_Low();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
    W25Q128_CS_High();
}

void W25Q128_WritePage(uint32_t addr, uint8_t *buf, uint32_t len) {
    if (len > W25Q128_PAGE_SIZE) len = W25Q128_PAGE_SIZE;
    uint8_t cmd[4];
    cmd[0] = W25Q128_CMD_WRITE;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    W25Q128_WriteEnable();
    W25Q128_CS_Low();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, buf, len, HAL_MAX_DELAY);
    W25Q128_CS_High();
    HAL_Delay(5);
}
