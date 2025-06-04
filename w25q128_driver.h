//w25q128_driver.h
#ifndef W25Q128_DRIVER_H
#define W25Q128_DRIVER_H

#include <stdint.h>

void W25Q128_Init(void);
uint32_t W25Q128_ReadID(void);
void W25Q128_WriteEnable(void);
void W25Q128_EraseSector(uint32_t addr);
void W25Q128_Read(uint32_t addr, uint8_t *buf, uint32_t len);
void W25Q128_WritePage(uint32_t addr, uint8_t *buf, uint32_t len);

#endif // W25Q128_DRIVER_H
