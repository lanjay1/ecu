#ifndef FLASH_CONFIG_H
#define FLASH_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#define CONFIG_FLASH_ADDR  0x000000 // Misalnya page pertama

typedef struct {
    uint16_t rpm_limit;
    uint16_t blip_duration;
    uint8_t  vva_mode;
    // Tambahkan parameter lain
    uint16_t checksum;
} ECU_Config;

bool FlashConfig_Load(ECU_Config *cfg);
bool FlashConfig_Save(const ECU_Config *cfg);
void FlashConfig_Default(ECU_Config *cfg);

#endif
