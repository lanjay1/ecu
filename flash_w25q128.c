//flash_w25q128.c
#include "flash_w25q128.h"
#include "spi_driver.h" // Harus diadaptasi dengan SPI di STM32H7
#include "gpio_driver.h" // Untuk CS control
#include <string.h>

#define CMD_READ_ID         0x9F
#define CMD_READ_DATA       0x03
#define CMD_PAGE_PROGRAM    0x02
#define CMD_SECTOR_ERASE    0x20
#define CMD_WRITE_ENABLE    0x06

#define FLASH_CS_LOW()      gpio_write(GPIO_FLASH_CS, 0)
#define FLASH_CS_HIGH()     gpio_write(GPIO_FLASH_CS, 1)

static void W25Q128_WriteEnable(void) {
    uint8_t cmd = CMD_WRITE_ENABLE;
    FLASH_CS_LOW();
    spi_transmit(&cmd, 1);
    FLASH_CS_HIGH();
}

bool W25Q128_ReadID(uint8_t *manufacturer_id, uint8_t *memory_type, uint8_t *capacity) {
    uint8_t cmd = CMD_READ_ID;
    uint8_t response[3];

    FLASH_CS_LOW();
    spi_transmit(&cmd, 1);
    spi_receive(response, 3);
    FLASH_CS_HIGH();

    *manufacturer_id = response[0];
    *memory_type = response[1];
    *capacity = response[2];
    return true;
}

bool W25Q128_Init(void) {
    gpio_set_output(GPIO_FLASH_CS);
    FLASH_CS_HIGH();

    uint8_t mid, type, cap;
    return W25Q128_ReadID(&mid, &type, &cap);
}

bool W25Q128_Read(uint32_t addr, uint8_t *buf, size_t len) {
    uint8_t cmd[4] = {
        CMD_READ_DATA,
        (addr >> 16) & 0xFF,
        (addr >> 8) & 0xFF,
        addr & 0xFF
    };

    FLASH_CS_LOW();
    spi_transmit(cmd, 4);
    spi_receive(buf, len);
    FLASH_CS_HIGH();
    return true;
}

bool W25Q128_Write(uint32_t addr, const uint8_t *buf, size_t len) {
    size_t written = 0;
    while (written < len) {
        size_t page_offset = addr % W25Q128_PAGE_SIZE;
        size_t space_in_page = W25Q128_PAGE_SIZE - page_offset;
        size_t chunk = (len - written > space_in_page) ? space_in_page : (len - written);

        W25Q128_WriteEnable();

        uint8_t cmd[4] = {
            CMD_PAGE_PROGRAM,
            (addr >> 16) & 0xFF,
            (addr >> 8) & 0xFF,
            addr & 0xFF
        };

        FLASH_CS_LOW();
        spi_transmit(cmd, 4);
        spi_transmit((uint8_t*)(buf + written), chunk);
        FLASH_CS_HIGH();

        addr += chunk;
        written += chunk;
    }
    return true;
}

bool W25Q128_EraseSector(uint32_t addr) {
    W25Q128_WriteEnable();

    uint8_t cmd[4] = {
        CMD_SECTOR_ERASE,
        (addr >> 16) & 0xFF,
        (addr >> 8) & 0xFF,
        addr & 0xFF
    };

    FLASH_CS_LOW();
    spi_transmit(cmd, 4);
    FLASH_CS_HIGH();
    return true;
}
