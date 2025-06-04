//map_storage.c
#include "map_storage.h"
#include "ecu_tuning.h"
#include "tps_calibration.h"
#include "ecu_config.h"
#include <string.h>
#include "w25q128_driver.h"

// Lokasi penyimpanan di Flash
#define FLASH_STORAGE_ADDR  ((uint32_t)0x081E0000)  // pastikan tidak bentrok dengan code
#define STORAGE_SIZE        sizeof(ECU_MapData_t)
#define MAX_PROFILES 3
#define PROFILE_ADDR_0  0x081C0000  // Map Harian
#define PROFILE_ADDR_1  0x081D0000  // Map Race


static bool CheckChecksum(const ECU_MapData_t* map) {
    uint32_t sum = 0;
    const uint8_t* data = (const uint8_t*)map;
    for (size_t i = 0; i < sizeof(ECU_MapData_t) - sizeof(uint32_t); i++) {
        sum += data[i];
    }
    return (sum == map->checksum);
}

static void CalculateChecksum(ECU_MapData_t* map) {
    map->checksum = 0;
    const uint8_t* data = (const uint8_t*)map;
    for (size_t i = 0; i < sizeof(ECU_MapData_t) - sizeof(uint32_t); i++) {
        map->checksum += data[i];
    }
}

bool MapStorage_Load(ECU_MapData_t* map) {
    if (!map) return false;

    memcpy(map, (void*)FLASH_STORAGE_ADDR, sizeof(ECU_MapData_t));
    if (map->magic != MAP_STORAGE_MAGIC || map->version != MAP_STORAGE_VERSION) return false;
    if (!CheckChecksum(map)) return false;

    return true;
}

bool MapStorage_Save(const ECU_MapData_t* map_in) {
    if (!map_in) return false;

    ECU_MapData_t map;
    memcpy(&map, map_in, sizeof(ECU_MapData_t));
    map.magic = MAP_STORAGE_MAGIC;
    map.version = MAP_STORAGE_VERSION;
    CalculateChecksum(&map);

    // Erase & write flash
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = FLASH_BANK_1,
        .Sector = FLASH_SECTOR_7,  // pastikan sektor ini sesuai
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    uint32_t error;
    HAL_FLASHEx_Erase(&erase, &error);

    const uint32_t* src = (const uint32_t*)&map;
    for (uint32_t i = 0; i < sizeof(ECU_MapData_t) / 4; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDR + i * 4, src[i]);
    }

    HAL_FLASH_Lock();
    return true;
}

void MapStorage_ApplyToSystem(const ECU_MapData_t* map) {
    if (!map) return;

    ECU_Tuning_SetFuelMap(map->fuel_map);
    ECU_Tuning_SetIgnitionMap(map->ignition_map);
    TPS_SetCalibration(map->tps_min, map->tps_max);
    ECU_SetIdleTarget(map->idle_rpm_target);
}

void MapStorage_ReadFromSystem(ECU_MapData_t* map) {
    if (!map) return;

    ECU_Tuning_GetFuelMap(map->fuel_map);
    ECU_Tuning_GetIgnitionMap(map->ignition_map);
    map->tps_min = TPS_GetMin();
    map->tps_max = TPS_GetMax();
    map->idle_rpm_target = ECU_GetIdleTarget();
}
void MapStorage_SaveAll(void) {
    W25Q128_WritePage(FUELMAP_ADDR, (uint8_t*)fuelMap, sizeof(fuelMap));
    W25Q128_WritePage(IGNMAP_ADDR, (uint8_t*)ignMap, sizeof(ignMap));
    W25Q128_WritePage(AFRMAP_ADDR, (uint8_t*)afrMap, sizeof(afrMap));
}

void MapStorage_LoadAll(void) {
    W25Q128_Read(FUELMAP_ADDR, (uint8_t*)fuelMap, sizeof(fuelMap));
    W25Q128_Read(IGNMAP_ADDR, (uint8_t*)ignMap, sizeof(ignMap));
    W25Q128_Read(AFRMAP_ADDR, (uint8_t*)afrMap, sizeof(afrMap));
}
void MapStorage_LoadProfile(uint8_t index) {
    uint32_t addr = (index == 0) ? PROFILE_ADDR_0 : PROFILE_ADDR_1;
    W25Q128_Read(addr, (uint8_t*)&map_shadow, sizeof(map_shadow));
    MapStorage_ApplyToSystem(&map_shadow);
}

void MapStorage_SaveProfile(uint8_t index) {
    uint32_t addr = (index == 0) ? PROFILE_ADDR_0 : PROFILE_ADDR_1;
    W25Q128_EraseSector(addr);
    CalculateChecksum(&map_shadow);
    W25Q128_WritePage(addr, (uint8_t*)&map_shadow, sizeof(map_shadow));
}
