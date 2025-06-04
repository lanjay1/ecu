//flash_writer.c
#include "flash_writer.h"
#include "stm32h7xx_hal.h"

bool Flash_Erase(uint32_t start_addr, uint32_t len) {
    HAL_FLASH_Unlock();

    uint32_t sector_error = 0;
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = FLASH_SECTOR_4; // sesuaikan
    erase.NbSectors = 4; // jumlah sektor yang akan dihapus

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    HAL_FLASH_Lock();
    return true;
}

bool Flash_Write(uint32_t addr, const uint8_t *data, uint32_t len) {
    HAL_FLASH_Unlock();

    for (uint32_t i = 0; i < len; i += 8) {
        uint64_t dword = 0xFFFFFFFFFFFFFFFF;
        memcpy(&dword, data + i, (len - i >= 8) ? 8 : len - i);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dword) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    return true;
}
