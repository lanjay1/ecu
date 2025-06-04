//flash_w25q128.h
#ifndef FLASH_W25Q128_H
#define FLASH_W25Q128_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define W25Q128_SECTOR_SIZE 4096
#define W25Q128_PAGE_SIZE   256
#define W25Q128_CAPACITY    (16 * 1024 * 1024)

bool W25Q128_Init(void);
bool W25Q128_Read(uint32_t addr, uint8_t *buf, size_t len);
bool W25Q128_Write(uint32_t addr, const uint8_t *buf, size_t len);
bool W25Q128_EraseSector(uint32_t addr);
bool W25Q128_ReadID(uint8_t *manufacturer_id, uint8_t *memory_type, uint8_t *capacity);

#endif // FLASH_W25Q128_H