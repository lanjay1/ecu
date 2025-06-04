//ecu_config.c
#include "ecu_config.h"
#include "flash_writer.h"
#include "checksum_manager.h"
#include <string.h>
 #include "ecu_state.h"

#define ECU_CONFIG_FLASH_ADDR   0x080E0000  // Sesuaikan dengan layout flash kamu
#define ECU_CONFIG_MAGIC        0xDEADBEEF  // Penanda valid

typedef struct {
    uint32_t magic;
    uint32_t checksum;
    ECU_Config_t config;
} ECU_ConfigFlashBlock_t;

static ECU_Config_t ecu_config;
static ECU_ConfigFlashBlock_t last_saved_block;


ECU_Config_t* ECU_Config_Get(void) {
    return &ecu_config;
}

void ECU_Config_ResetToDefault(void) {
    ecu_config.rpm_limit              = 10500;
    ecu_config.idle_target_rpm       = 1300;
    ecu_config.vva_activation_rpm    = 7400;
    ecu_config.launch_control_rpm    = 4000;
    ecu_config.quickshifter_enabled  = true;
    ecu_config.pit_rpm_limit         = 5000;
    ecu_config.pit_throttle_limit    = 30.0f;
    ecu_config.launch_retard_deg     = 10.0f;
    ecu_config.launch_extra_fuel     = 1.5f;
    ecu_config.firmware_version      = ECU_FIRMWARE_VERSION;
}

void ECU_Config_LoadFromFlash(void) {
    ECU_ConfigFlashBlock_t block;
    Flash_Read(ECU_CONFIG_FLASH_ADDR, &block, sizeof(block));

    if (block.magic == ECU_CONFIG_MAGIC &&
        block.checksum == Checksum_Calculate32((uint8_t*)&block.config, sizeof(ECU_Config_t))) {

        memcpy(&ecu_config, &block.config, sizeof(ECU_Config_t));
        memcpy(&last_saved_block, &block, sizeof(block));
        ecu_state.active_profile = ecu_config.last_profile;
    } else {
        ECU_Config_ResetToDefault();
        ECU_Config_SaveToFlash(); // auto-repair
    }
}

void ECU_Config_SaveToFlash(void) {
    ECU_ConfigFlashBlock_t block;
    block.magic = ECU_CONFIG_MAGIC;
    memcpy(&block.config, &ecu_config, sizeof(ECU_Config_t));
    block.checksum = Checksum_Calculate32((uint8_t*)&block.config, sizeof(ECU_Config_t));

    if (memcmp(&block, &last_saved_block, sizeof(block)) != 0) {
      
        if (Flash_Erase(ECU_CONFIG_FLASH_ADDR, sizeof(ECU_ConfigFlashBlock_t)) == true) { // Kirim panjang data
            Flash_Write(ECU_CONFIG_FLASH_ADDR, (const uint8_t*)&block, sizeof(ECU_ConfigFlashBlock_t)); // Cast ke const uint8_t*
            memcpy(&last_saved_block, &block, sizeof(block));
        } else {
            // Handle erase error
        }
    }
}