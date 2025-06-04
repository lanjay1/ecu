//flash_writer.h
#ifndef FLASH_WRITER_H
#define FLASH_WRITER_H

#include <stdint.h>
#include <stdbool.h>

bool Flash_Erase(uint32_t start_addr, uint32_t len);
bool Flash_Write(uint32_t addr, const uint8_t *data, uint32_t len);

#endif
