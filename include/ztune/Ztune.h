// ZtuneECU - tipe umum & utilitas
#pragma once
#include <stdint.h>
#include <stddef.h>

#define ZTUNE_SCHEMA_VERSION 6u
#define ZTUNE_FW_VERSION "0.5.0-m5"

namespace ztune {

enum class TriggerState : uint8_t { Hunting = 0, Synced = 1, Lost = 2 };

// CRC32 (polinomial 0xEDB88320) - dipakai persist config NVS & frame ZTP.
// Vektor uji standar: crc32("123456789") == 0xCBF43926
inline uint32_t crc32(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        c ^= p[i];
        for (int k = 0; k < 8; ++k)
            c = (c & 1u) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
    }
    return ~c;
}

} // namespace ztune
