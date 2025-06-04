//checksum_manager.h
#pragma once
#include <stdint.h>

typedef enum {
    CHECKSUM_CRC16,
    CHECKSUM_CRC32,
    CHECKSUM_SUM8      // Simple 8-bit sum
} ChecksumType_t;

uint8_t  Checksum_CalculateSum8(const uint8_t* data, uint32_t len);
uint16_t Checksum_Calculate16(const uint8_t* data, uint32_t len);
uint32_t Checksum_Calculate32(const uint8_t* data, uint32_t len);

// Convenience macro
#define CHECKSUM(type, data, len) \
    ((type) == CHECKSUM_SUM8  ? Checksum_CalculateSum8(data, len) : \
     /* ((type) == CHECKSUM_CRC8  ? Checksum_CalculateCRC8(data, len) : \ */ \
     ((type) == CHECKSUM_CRC16 ? Checksum_Calculate16(data, len) : \
     ((type) == CHECKSUM_CRC32 ? Checksum_Calculate32(data, len) : \
     0))) // Default value atau error handling jika tipe tidak dikenal