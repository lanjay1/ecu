// ZTP v1 - dispatcher request -> response (pure, dites native).
// Semua handler murni fungsi dari (ctx, request); efek samping via ctx
// (mutasi EcuConfig + hook onPersist untuk NVS) sehingga mudah diuji.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <cstdio>
#include <functional>
#include "Ztune.h"
#include "ZtpTypes.h"
#include "ZtpFrame.h"
#include "EcuConfig.h"
#include "TriggerDecoder.h"
#include "EgoControl.h"

namespace ztune {
namespace ztp {

struct ZtpCtx {
    EcuConfig* cfg = nullptr;
    bool* liveActive = nullptr;
    uint16_t liveRateMs = 100;
    std::function<void(const EcuConfig&)> onPersist;   // hook persist NVS (ESP32)
};

// ---- helper internal ----

inline size_t tableFloatCount(uint8_t id) {
    switch ((TableId)id) {
        case TableId::VE:
        case TableId::ADVANCE:
        case TableId::AFR_TARGET: return 16 + 16 + 16 * 16;   // 288 float
        case TableId::DWELL:      return 16 + 16;             // 32 float
        default: return 0;
    }
}

// Urutan payload tabel: [axisX 16][axisY 16][values 16x16 row-major]
// (VE/advance/AFR target) untuk dwell: [x 16][y 16].
// axisX = rpm, axisY = load (TPS%/MAP kPa).
inline size_t fillTablePayload(const EcuConfig& cfg, uint8_t id, uint8_t* out, size_t cap) {
    size_t nf = tableFloatCount(id);
    if (nf == 0 || cap < 1 + nf * 4) return 0;
    out[0] = id;
    float* f = (float*)(out + 1);
    switch ((TableId)id) {
        case TableId::VE:
            memcpy(f, cfg.fuel.ve.xAxis, 16 * 4);
            memcpy(f + 16, cfg.fuel.ve.yAxis, 16 * 4);
            memcpy(f + 32, cfg.fuel.ve.values, 256 * 4);
            break;
        case TableId::ADVANCE:
            memcpy(f, cfg.ign.advance.xAxis, 16 * 4);
            memcpy(f + 16, cfg.ign.advance.yAxis, 16 * 4);
            memcpy(f + 32, cfg.ign.advance.values, 256 * 4);
            break;
        case TableId::AFR_TARGET:
            memcpy(f, cfg.ego.afrTarget.xAxis, 16 * 4);
            memcpy(f + 16, cfg.ego.afrTarget.yAxis, 16 * 4);
            memcpy(f + 32, cfg.ego.afrTarget.values, 256 * 4);
            break;
        case TableId::DWELL:
            memcpy(f, cfg.ign.dwell.x, 16 * 4);
            memcpy(f + 16, cfg.ign.dwell.y, 16 * 4);
            break;
        default: return 0;
    }
    return 1 + nf * 4;
}

inline bool applyTable(EcuConfig& cfg, uint8_t id, const float* f) {
    switch ((TableId)id) {
        case TableId::VE:
            memcpy(cfg.fuel.ve.xAxis, f, 16 * 4);
            memcpy(cfg.fuel.ve.yAxis, f + 16, 16 * 4);
            memcpy(cfg.fuel.ve.values, f + 32, 256 * 4);
            return true;
        case TableId::ADVANCE:
            memcpy(cfg.ign.advance.xAxis, f, 16 * 4);
            memcpy(cfg.ign.advance.yAxis, f + 16, 16 * 4);
            memcpy(cfg.ign.advance.values, f + 32, 256 * 4);
            return true;
        case TableId::AFR_TARGET:
            memcpy(cfg.ego.afrTarget.xAxis, f, 16 * 4);
            memcpy(cfg.ego.afrTarget.yAxis, f + 16, 16 * 4);
            memcpy(cfg.ego.afrTarget.values, f + 32, 256 * 4);
            return true;
        case TableId::DWELL:
            memcpy(cfg.ign.dwell.x, f, 16 * 4);
            memcpy(cfg.ign.dwell.y, f + 16, 16 * 4);
            return true;
        default: return false;
    }
}

inline size_t err(uint8_t* out, size_t cap, Err code, uint8_t reqType, const char* msg) {
    ErrorBody b{(uint8_t)code, reqType};
    size_t ml = msg ? strlen(msg) : 0;
    if (ml > 64) ml = 64;
    uint8_t tmp[2 + 64];
    memcpy(tmp, &b, 2);
    if (ml) memcpy(tmp + 2, msg, ml);
    return encodeFrame(out, cap, (uint8_t)Type::ERROR, tmp, 2 + ml);
}

// ---- dispatcher utama ----

// Proses satu request -> tulis response frame ke out. Return byte response.
inline size_t dispatch(ZtpCtx& ctx, uint8_t reqType, const uint8_t* p, size_t len,
                       uint8_t* out, size_t cap) {
    EcuConfig& cfg = *ctx.cfg;

    switch ((Type)reqType) {
    case Type::HELLO: {
        HelloInfo h{};
        h.protoVer = PROTO_VER;
        h.schemaVersion = ZTUNE_SCHEMA_VERSION;
        // caps: bit0 USB (selalu), bit1 BLE, bit2 WiFi - sesuai config comms.
        h.caps = 0x1 | (cfg.comms.bleEnable ? 0x2 : 0) |
                 (cfg.comms.wifiEnable ? 0x4 : 0);
        snprintf(h.fwVersion, sizeof(h.fwVersion), "%s", ZTUNE_FW_VERSION);
        return encodeFrame(out, cap, (uint8_t)Type::HELLO, &h, sizeof(h));
    }

    case Type::PING:
        return encodeFrame(out, cap, (uint8_t)Type::PONG, p, len);   // echo

    case Type::CONFIG_GET:
        return encodeFrame(out, cap, (uint8_t)Type::CONFIG_GET, &cfg, sizeof(EcuConfig));

    case Type::CONFIG_SET: {
        if (len != sizeof(EcuConfig))
            return err(out, cap, Err::BAD_PAYLOAD, reqType, "size config salah");
        EcuConfig cand;
        memcpy(&cand, p, sizeof(cand));
        char emsg[96] = {0};
        if (!cand.validate(emsg, sizeof(emsg)))
            return err(out, cap, Err::VALIDATION, reqType, emsg);
        cfg = cand;
        if (ctx.onPersist) ctx.onPersist(cfg);
        return encodeFrame(out, cap, reqType, &cfg.schemaVersion, 4);
    }

    case Type::TABLE_READ: {
        if (len != 1) return err(out, cap, Err::BAD_PAYLOAD, reqType, "butuh tableId");
        uint8_t tmp[1 + 288 * 4];
        size_t n = fillTablePayload(cfg, p[0], tmp, sizeof(tmp));
        if (n == 0) return err(out, cap, Err::BAD_TABLE_ID, reqType, "tableId tak dikenal");
        return encodeFrame(out, cap, reqType, tmp, n);
    }

    case Type::TABLE_WRITE: {
        if (len < 1) return err(out, cap, Err::BAD_PAYLOAD, reqType, "payload kosong");
        uint8_t id = p[0];
        size_t expect = tableFloatCount(id);
        if (expect == 0) return err(out, cap, Err::BAD_TABLE_ID, reqType, "tableId tak dikenal");
        if (len != 1 + expect * 4)
            return err(out, cap, Err::BAD_PAYLOAD, reqType, "size tabel salah");
        applyTable(cfg, id, (const float*)(p + 1));
        // TODO(M3.1): rollback bila suatu saat tabel ikut divalidasi silang
        char emsg[96] = {0};
        if (!cfg.validate(emsg, sizeof(emsg)))
            return err(out, cap, Err::VALIDATION, reqType, emsg);
        if (ctx.onPersist) ctx.onPersist(cfg);
        return encodeFrame(out, cap, reqType, p, 1);
    }

    case Type::LIVE_START: {
        if (len != 2) return err(out, cap, Err::BAD_PAYLOAD, reqType, "butuh rateMs u16");
        uint16_t rate = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
        if (rate < 10) rate = 10;
        ctx.liveRateMs = rate;
        *ctx.liveActive = true;
        return encodeFrame(out, cap, reqType, p, 2);
    }

    case Type::LIVE_STOP:
        *ctx.liveActive = false;
        return encodeFrame(out, cap, reqType, nullptr, 0);

    default:
        return err(out, cap, Err::BAD_TYPE, reqType, "type tidak dikenal");
    }
}

// ---- builder LIVE_FRAME (pure) ----

// Snapshot sensor ter-smooth (M4.1) untuk live telemetry.
struct LiveSensors {
    float tpsPct = 0.0f;
    float mapKpa = 0.0f;
    float cltC   = 0.0f;
    float iatC   = 0.0f;
    float battV  = 0.0f;
};

inline LiveFrameV2 buildLiveFrame(const EcuConfig& cfg, const TriggerDecoder& dec,
                                  uint32_t tMs, const LiveSensors& sen,
                                  float pwMs, float advanceDeg,
                                  const EgoState& ego = EgoState{}) {
    LiveFrameV2 f{};
    f.tMs = tMs;
    f.rpm = dec.rpm();
    f.trigState = (uint8_t)dec.state();
    f.toothIndex = dec.toothIndex();
    // Load mengikuti konfigurasi: MAP (speed-density style) atau TPS (alpha-N)
    float load = (cfg.fuel.loadSource == LoadSource::MAP) ? sen.mapKpa : sen.tpsPct;
    f.vePct = cfg.fuel.ve.lookup(dec.rpm(), load);   // axis: rpm x load
    f.advanceDeg = advanceDeg;
    f.pwMs = pwMs;
    f.tpsPct = sen.tpsPct;
    f.mapKpa = sen.mapKpa;
    f.iatC = sen.iatC;
    f.cltC = sen.cltC;
    f.battV = sen.battV;
    f.afrMeasured = ego.measuredAfr;
    f.afrTarget   = ego.targetAfr;
    f.egoPct      = ego.corrPct;
    f.egoActive   = ego.active ? 1 : 0;
    return f;
}

} // namespace ztp
} // namespace ztune
