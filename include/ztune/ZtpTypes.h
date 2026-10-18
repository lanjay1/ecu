// ZTP v2 (Ztune Protocol) - definisi tipe wire-level.
// v2 (M5): LiveFrame diperluas dgn field O2/EGO, tabel target AFR 16x16
// (TableId::AfrTarget) masuk TABLE_READ/WRITE, MAX_PAYLOAD 8192 karena
// blob EcuConfig tumbuh (schema v6). PROTO_VER dinaikkan -> app lama tahu.
// Payload CONFIG tetap blob struct EcuConfig (bukan JSON).
// Filosofi frame ala rusEFI: magic + ver + type + len + CRC32.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "Ztune.h"

namespace ztune {
namespace ztp {

constexpr uint8_t MAGIC0 = 0x5A;        // 'Z'
constexpr uint8_t MAGIC1 = 0x54;        // 'T'
constexpr uint8_t PROTO_VER = 0x02;
constexpr size_t HEADER_SIZE = 6;       // magic(2) + ver(1) + type(1) + len(2 LE)
constexpr size_t CRC_SIZE = 4;          // CRC32 LE, mencakup ver..payload (magic di luar)
constexpr size_t MAX_PAYLOAD = 8192;   // >= sizeof(EcuConfig) schema v6, invarian dites

enum class Type : uint8_t {
    HELLO       = 0x01,   // handshake: host tahu versi protokol/schema/firmware
    CONFIG_GET  = 0x02,
    CONFIG_SET  = 0x03,
    TABLE_READ  = 0x04,
    TABLE_WRITE = 0x05,
    LIVE_START  = 0x06,   // payload: rateMs u16 LE
    LIVE_STOP   = 0x07,
    LIVE_FRAME  = 0x08,   // firmware -> host
    LOG_READ    = 0x09,   // placeholder (M6+)
    PING        = 0x0A,
    PONG        = 0x0B,
    ERROR       = 0xFF,
};

enum class Err : uint8_t {
    NONE = 0,
    BAD_CRC     = 1,
    BAD_VERSION = 2,
    BAD_TYPE    = 3,
    BAD_PAYLOAD = 4,
    VALIDATION  = 5,      // config/table ditolak validate()
    BAD_TABLE_ID= 6,
    BUSY        = 7,
};

enum class TableId : uint8_t { VE = 1, ADVANCE = 2, DWELL = 3, AFR_TARGET = 4 };

// Konvensi respons: sukses = type request yang sama; gagal = type ERROR
// dengan payload [code u8][reqType u8][pesan teks opsional].

// --- payload struct (packed, little-endian) ---
#pragma pack(push, 1)
struct HelloInfo {
    uint8_t  protoVer;
    uint32_t schemaVersion;
    uint32_t caps;            // bit0 USB, bit1 BLE, bit2 WiFi
    char     fwVersion[16];
};

struct LiveFrameV1 {
    uint32_t tMs;
    float    rpm;
    uint8_t  trigState;       // TriggerState
    uint8_t  toothIndex;
    float    vePct;           // hasil lookup VE kondisi saat ini
    float    advanceDeg;
    float    pwMs;
    float    mapKpa;
    float    tpsPct;
    float    iatC;
    float    cltC;
    float    battV;
};

// v2 (M5): + telemetri O2/closed loop.
struct LiveFrameV2 {
    uint32_t tMs;
    float    rpm;
    uint8_t  trigState;
    uint8_t  toothIndex;
    float    vePct;
    float    advanceDeg;
    float    pwMs;
    float    mapKpa;
    float    tpsPct;
    float    iatC;
    float    cltC;
    float    battV;
    float    afrMeasured;     // AFR terukur (wideband; 0 saat NB)
    float    afrTarget;       // target AFR (tabel; 0 saat NB)
    float    egoPct;          // koreksi closed loop %
    uint8_t  egoActive;       // gate closed loop OK
};

struct ErrorBody {
    uint8_t  code;            // Err
    uint8_t  reqType;
};
#pragma pack(pop)

inline const char* errName(uint8_t e) {
    switch ((Err)e) {
        case Err::BAD_CRC:      return "BAD_CRC";
        case Err::BAD_VERSION:  return "BAD_VERSION";
        case Err::BAD_TYPE:     return "BAD_TYPE";
        case Err::BAD_PAYLOAD:  return "BAD_PAYLOAD";
        case Err::VALIDATION:   return "VALIDATION";
        case Err::BAD_TABLE_ID: return "BAD_TABLE_ID";
        case Err::BUSY:         return "BUSY";
        default:                return "ERR";
    }
}

} // namespace ztp
} // namespace ztune
