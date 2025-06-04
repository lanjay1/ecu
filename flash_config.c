//flash_config.c
#include "flash_config.h"
#include "w25q128_driver.h"
#include <string.h>

static uint16_t calculate_checksum(const ECU_Config* cfg) {
    const uint8_t* data = (const uint8_t*)cfg;
    uint16_t sum = 0;
    for (size_t i = 0; i < sizeof(ECU_Config) - 2; ++i)
        sum += data[i];
    return sum;
}

bool FlashConfig_Load(ECU_Config *cfg) {
    W25Q_Read(CONFIG_FLASH_ADDR, (uint8_t*)cfg, sizeof(ECU_Config));
    return cfg->checksum == calculate_checksum(cfg);
}

bool FlashConfig_Save(const ECU_Config *cfg) {
    ECU_Config temp = *cfg;
    temp.checksum = calculate_checksum(&temp);
    W25Q_Erase_Sector(CONFIG_FLASH_ADDR);
    return W25Q_Write(CONFIG_FLASH_ADDR, (uint8_t*)&temp, sizeof(ECU_Config));
}

void FlashConfig_Default(ECU_Config *cfg) {
    memset(cfg, 0, sizeof(ECU_Config));
    cfg->rpm_limit = 12000;
    cfg->blip_duration = 90;
    cfg->vva_mode = 1;
}
