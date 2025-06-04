//iap.c
#include "iap.h"
#include "flash_writer.h"
#include "checksum_manager.h"
#include "main.h"

#define IAP_BASE_ADDR  0x08020000
#define IAP_MAX_SIZE   (512 * 1024)

static uint32_t iap_received_size = 0;

void IAP_Init(void) {
    Flash_Erase(IAP_BASE_ADDR, IAP_MAX_SIZE);
    iap_received_size = 0;
}

void IAP_ReceiveChunk(uint32_t offset, const uint8_t *data, uint32_t len) {
    if (offset + len > IAP_MAX_SIZE) return;
    Flash_Write(IAP_BASE_ADDR + offset, data, len);
    iap_received_size += len;
}

bool IAP_Complete(uint32_t firmware_size, uint32_t expected_checksum) {
    if (iap_received_size != firmware_size) return false;

    const uint8_t *fw_data = (const uint8_t *) IAP_BASE_ADDR;
    uint32_t calc = CHECKSUM(fw_data, firmware_size);
    return (calc == expected_checksum);
}
