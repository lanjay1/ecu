// EcuConfig - SEMUA parameter tunable firmware dalam satu struct versioned.
// Dipersist ke NVS sebagai blob + CRC32; diubah via aplikasi tuning (ZTP).
//
// KEBIJAKAN NO-HARDCODE (keputusan user):
//  - Tidak ada satu pun nilai mesin yang dibekukan di kode.
//  - EcuConfig::defaults() hanyalah safe-mode placeholder: firmware hanya
//    menjalankan decoder + telemetri sampai tuneAccepted=true ditulis oleh
//    aplikasi tuning setelah setup (cylinder, trigger, injektor, sensor...).
//  - Nilai default yang ADA berasal dari keputusan user yang sudah final:
//    trigger 12-1 VR, gapRatio 1.5, timeout 200 ms, tabel 16x16,
//    opsi A tanpa cam -> wasted spark + 2 squirt per siklus.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstdio>
#include <cstring>
#include "Ztune.h"
#include "TriggerConfig.h"
#include "InterpTable.h"
#include "FuelMath.h"

namespace ztune {

enum class LoadSource : uint8_t { MAP = 0, TPS = 1 };
enum class StrokeType : uint8_t { FourStroke = 0, TwoStroke = 1 };
// M5: jenis sensor O2. Narrowband = sensor switching standar (hanya bermakna
// di sekitar stoich); Wideband = pengendali eksternal dgn output linier
// 0-5 V (via pembagi ke ADC) -> AFR terukur.
enum class O2Type : uint8_t { None = 0, Narrowband = 1, Wideband = 2 };

struct IoConfig {
    uint8_t ckpGpio = 4;   // PLACEHOLDER pin - WAJIB disesuaikan wiring via aplikasi
    // Output per silinder (maks 6): injektor & coil. Wasted spark (crank-only)
    // digerakkan per-silinder di software; pemetaan ke pasangan coil di hardware
    // adalah keputusan wiring, bukan firmware.
    uint8_t injGpio[6] = {5, 5, 5, 5, 5, 5};   // PLACEHOLDER
    uint8_t ignGpio[6] = {6, 6, 6, 6, 6, 6};   // PLACEHOLDER
    uint8_t qsInputGpio = 7;                   // sensor gear quickshifter, PLACEHOLDER
    // Sensor analog (M4.1) - WAJIB ADC1 (GPIO 1..10): ADC2 tidak dapat dipakai
    // bersamaan dengan WiFi. Semua PLACEHOLDER - sesuaikan wiring via app.
    uint8_t tpsGpio  = 1;
    uint8_t mapGpio  = 2;
    uint8_t cltGpio  = 3;
    uint8_t iatGpio  = 4;
    uint8_t battGpio = 10;
    uint8_t o2Gpio   = 8;   // M5: output sensor O2 (NB/WB via pembagi), PLACEHOLDER
};

// Closed loop bahan bakar (M5): koreksi EGO % menuju target AFR (wideband)
// atau titik switch stoich (narrowband). Koreksi dijumlahkan seperti WUE/
// IAT/ASE dan dibatasi limitPct. Hanya aktif di jendela cruise (gate).
struct EgoConfig {
    O2Type o2Type = O2Type::None;
    bool   enabled = false;             // master closed loop
    // Narrowband: tegangan switch stoich + histeresis anti-flapping
    float  nbSwitchV = 0.45f;
    float  nbHystV   = 0.05f;
    // Wideband: kalibrasi linier 2 titik ADC -> AFR (kurva pengendali)
    int    wbAdcLow  = 500;   float wbAfrLow  = 9.0f;
    int    wbAdcHigh = 3500;  float wbAfrHigh = 19.0f;
    // Kontrol
    float    limitPct    = 10.0f;   // clamp koreksi +-10%
    float    stepPct     = 1.0f;    // langkah per update
    float    deadbandAfr = 0.10f;   // error AFR diabaikan dalam band ini
    uint16_t updateMs    = 125;     // periode penyesuaian
    uint16_t sensorDelayMs = 1000;  // delay transportasi gas + settle sensor
                                    // (set besar utk warmup pengendali WB)
    // Gate kondisi aktif: cruise, mesin hangat, rpm masuk akal
    uint16_t minRpm   = 1000;
    uint16_t maxRpm   = 12000;
    float    maxTpsPct = 70.0f;     // di atas ini = power regime -> open loop
    float    minCltC   = 70.0f;
    AfrTargetTable16 afrTarget{};   // sumbu diisi defaults(); inert saat NB
};

struct EcuConfig {
    uint32_t schemaVersion = ZTUNE_SCHEMA_VERSION;
    TriggerConfig trigger = TriggerConfig::defaults();

    struct Engine {
        uint8_t    cylinders = 1;
        StrokeType stroke = StrokeType::FourStroke;
        bool       camSensorPresent = false;   // OPSI A (keputusan user): tanpa cam dahulu
        float      dispPerCylCc = 125.0f;      // PLACEHOLDER - wajib diisi saat setup
        float      stoichAfr = 14.7f;
        uint8_t    squirtsPerCycle = 2;        // tanpa cam: 2 squirt/siklus (tiap 360 derajat)
    } engine;

    struct Fuel {
        float      injectorCcPerMin = 160.0f;  // PLACEHOLDER - wajib diisi saat setup
        // Dead time injektor vs tegangan baterai (M4.2): injektor membuka lebih
        // lambat saat volt rendah (cranking) - koreksi non-linear, makanya KURVA
        // bukan skalar. x = volt (grid 1 V: 6..21 V), y = ms. Clamp di ujung.
        DtimeCurve16 dtime{};
        LoadSource loadSource = LoadSource::TPS;  // motor: alpha-N (TPS) umum; bisa MAP
        // Kurva koreksi bahan bakar (M4.1), isi defaults() masuk akal utk motor
        WueCurve16  wue{};       // % tambahan vs suhu coolant (C)
        IatCurve16  iatCorr{};   // % koreksi densitas vs suhu udara (C)
        // Acceleration Enrichment (tip-in)
        float    aseTpsRatePctPerS = 40.0f;  // ambang laju TPS utk deteksi tip-in
        float    asePct = 20.0f;             // besaran enrich di awal window
        uint16_t aseDurationMs = 600;        // durasi ramp-down
        // Sudut END of injeksi (konvensi MegaSquirt): jarak SEBELUM TDC kompresi
        // silinder terkait. Awal buka = end - PW(sudut). Dikalibrasi via app.
        float      injEndBeforeTdcDeg = 60.0f;
        VeTable16  ve{};                       // sumbu diisi defaults(); nilai inert
    } fuel;

    struct Ign {
        AdvanceTable16 advance{};              // derajat BTDC vs load x rpm
        DwellCurve16   dwell{};                // ms vs rpm
    } ign;

    // Pembatas RPM: hard fuel-cut di hardLimitRpm, resume setelah turun
    // resumeDropRpm (histeresis anti-flapping). Prioritas fitur user (M4).
    struct RevLimit {
        bool     enabled = true;
        uint16_t hardLimitRpm = 11000;   // PLACEHOLDER - wajib diisi saat setup
        uint16_t resumeDropRpm = 300;
    } revLimit;

    // Quickshifter: fuel cut singkat saat upshift agar beban gigi lepas tanpa
    // tarik kopling. Dipicu sensor gear (qsInputGpio) dengan debounce.
    struct QuickShift {
        bool     enabled = false;
        uint16_t cutMs = 80;             // durasi potong bahan bakar per shift
        uint16_t minRpm = 4000;          // abaikan sinyal di bawah rpm ini
        uint16_t debounceMs = 30;        // debounce sensor gear
    } quickShift;

    // Transport tuning (M3.1): BLE selalu tersedia (paddock-friendly);
    // WiFi STA opsional. SSID/password adalah DATA tunable -> tidak boleh
    // di-hardcode di firmware (aturan no-hardcode berlaku juga di sini).
    struct Comms {
        bool     bleEnable = true;
        bool     wifiEnable = false;            // default off: BLE dulu
        char     bleName[17] = "ZtuneECU";      // nama advertise, 16 char + NUL
        char     wifiSsid[33] = "";             // 32 char + NUL
        char     wifiPass[65] = "";             // 63 char + NUL
        uint16_t wifiTcpPort = 6666;            // TCP server ZTP
    } comms;

    // Kalibrasi sensor (M4.1). Semua nilai PLACEHOLDER safe-mode - wajib
    // di-set via aplikasi tuning sesuai sensor & wiring nyata.
    struct Sensors {
        // TPS: 2 titik ADC raw (ADC naik saat buka throttle)
        int      tpsClosedAdc = 100;
        int      tpsWotAdc    = 3000;
        // MAP: sensor linier 2 titik ADC -> kPa (mis. sensor 3-bar)
        int      mapZeroAdc = 620;
        float    mapZeroKpa = 20.0f;
        int      mapFullAdc = 2900;
        float    mapFullKpa = 250.0f;
        // NTC CLT/IAT: persamaan beta (pull-up ke 3.3V, sensor ke GND)
        float    cltPullupOhm = 10000.0f;
        float    cltBetaK     = 3380.0f;
        float    cltR0Ohm     = 10000.0f;
        float    cltT0C       = 25.0f;
        float    iatPullupOhm = 10000.0f;
        float    iatBetaK     = 3380.0f;
        float    iatR0Ohm     = 10000.0f;
        float    iatT0C       = 25.0f;
        // Baterai: pembagi resistif, ratio = (Rtop+Rbot)/Rbot
        float    battDivRatio = 11.0f;
    } sensors;

    EgoConfig ego;   // M5: closed loop EGO (definisi struct di bawah)

    IoConfig io;
    bool tuneAccepted = false;  // gate SAFE MODE: output mesin terkunci sampai setup selesai

    bool validate(char* err, size_t errLen) const {
        auto fail = [&](const char* m) {
            if (err) snprintf(err, errLen, "%s", m);
            return false;
        };
        if (schemaVersion != ZTUNE_SCHEMA_VERSION) return fail("schemaVersion beda");
        char sub[112];
        if (!trigger.validate(sub, sizeof(sub))) {
            if (err) snprintf(err, errLen, "%s", sub);
            return false;
        }
        if (engine.cylinders < 1 || engine.cylinders > 6) return fail("cylinders 1..6");
        if (engine.dispPerCylCc < 25.0f || engine.dispPerCylCc > 2000.0f)
            return fail("dispPerCylCc 25..2000 cc");
        if (engine.stoichAfr < 6.0f || engine.stoichAfr > 17.5f) return fail("stoichAfr 6..17.5");
        if (engine.squirtsPerCycle < 1 || engine.squirtsPerCycle > 4)
            return fail("squirtsPerCycle 1..4");
        if (engine.stroke == StrokeType::TwoStroke && engine.camSensorPresent)
            return fail("mesin 2-tak tidak memakai cam sensor");
        if (fuel.injectorCcPerMin < 40.0f || fuel.injectorCcPerMin > 2500.0f)
            return fail("injectorCcPerMin 40..2500");
        // Kurva dead time: x harus naik ketat & dalam rentang wajar, y 0..5 ms
        for (int i = 0; i < 16; ++i) {
            if (fuel.dtime.x[i] < 0.0f || fuel.dtime.x[i] > 30.0f ||
                fuel.dtime.y[i] < 0.0f || fuel.dtime.y[i] > 5.0f)
                return fail("dtime x 0..30 V, y 0..5 ms");
            if (i > 0 && fuel.dtime.x[i] <= fuel.dtime.x[i - 1])
                return fail("dtime sumbu x harus naik ketat");
        }
        if ((uint8_t)fuel.loadSource > 1) return fail("loadSource tidak valid");
        if (fuel.injEndBeforeTdcDeg < 0.0f || fuel.injEndBeforeTdcDeg > 180.0f)
            return fail("injEndBeforeTdcDeg 0..180");
        if (revLimit.hardLimitRpm < 1000 || revLimit.hardLimitRpm > 25000)
            return fail("revLimit 1000..25000 rpm");
        if (revLimit.resumeDropRpm < 50 || revLimit.resumeDropRpm > 5000 ||
            revLimit.resumeDropRpm >= revLimit.hardLimitRpm)
            return fail("resumeDropRpm 50..5000 dan < hardLimit");
        if (quickShift.cutMs < 10 || quickShift.cutMs > 500)
            return fail("qs cutMs 10..500");
        if (quickShift.minRpm < 500 || quickShift.minRpm > 20000)
            return fail("qs minRpm 500..20000");
        if (quickShift.debounceMs > 500) return fail("qs debounceMs 0..500");
        if (io.ckpGpio > 48 || io.qsInputGpio > 48)
            return fail("GPIO harus 0..48 (ESP32-S3)");
        for (int i = 0; i < 6; ++i)
            if (io.injGpio[i] > 48 || io.ignGpio[i] > 48)
                return fail("GPIO harus 0..48 (ESP32-S3)");
        // Sensor analog WAJIB ADC1 (GPIO 1..10) - ADC2 bentrok dgn WiFi
        if (io.tpsGpio < 1 || io.tpsGpio > 10 || io.mapGpio < 1 || io.mapGpio > 10 ||
            io.cltGpio < 1 || io.cltGpio > 10 || io.iatGpio < 1 || io.iatGpio > 10 ||
            io.battGpio < 1 || io.battGpio > 10 || io.o2Gpio < 1 || io.o2Gpio > 10)
            return fail("sensor ADC wajib GPIO 1..10 (ADC1, kompatibel WiFi)");
        if (sensors.tpsWotAdc <= sensors.tpsClosedAdc + 200)
            return fail("tpsWotAdc harus > tpsClosedAdc + 200");
        if (sensors.mapFullAdc <= sensors.mapZeroAdc)
            return fail("mapFullAdc harus > mapZeroAdc");
        if (sensors.mapZeroKpa < 5.0f || sensors.mapFullKpa > 400.0f ||
            sensors.mapFullKpa <= sensors.mapZeroKpa)
            return fail("rentang MAP kPa tidak valid");
        auto ntcOk = [](const float s[4]) {
            return s[0] >= 100.0f && s[0] <= 100000.0f &&    // pullup
                   s[1] >= 1000.0f && s[1] <= 6000.0f &&     // beta
                   s[2] >= 100.0f && s[2] <= 100000.0f &&    // r0
                   s[3] >= -40.0f && s[3] <= 150.0f;         // t0
        };
        float clt[4] = {sensors.cltPullupOhm, sensors.cltBetaK, sensors.cltR0Ohm, sensors.cltT0C};
        float iat[4] = {sensors.iatPullupOhm, sensors.iatBetaK, sensors.iatR0Ohm, sensors.iatT0C};
        if (!ntcOk(clt) || !ntcOk(iat)) return fail("kalibrasi NTC tidak valid");
        if (sensors.battDivRatio < 1.0f || sensors.battDivRatio > 100.0f)
            return fail("battDivRatio 1..100");
        if (fuel.aseTpsRatePctPerS < 5.0f || fuel.aseTpsRatePctPerS > 500.0f)
            return fail("aseTpsRate 5..500 %/s");
        if (fuel.asePct < 0.0f || fuel.asePct > 100.0f) return fail("asePct 0..100");
        if (fuel.aseDurationMs < 100 || fuel.aseDurationMs > 5000)
            return fail("aseDurationMs 100..5000");
        if (memchr(comms.bleName, 0, sizeof(comms.bleName)) == nullptr)
            return fail("bleName tak ber-NUL");
        if (comms.wifiEnable) {
            if (comms.wifiSsid[0] == 0) return fail("wifiEnable tapi ssid kosong");
            if (comms.wifiTcpPort == 0) return fail("wifiTcpPort 0");
        }
        if (memchr(comms.wifiSsid, 0, sizeof(comms.wifiSsid)) == nullptr ||
            memchr(comms.wifiPass, 0, sizeof(comms.wifiPass)) == nullptr)
            return fail("ssid/pass tak ber-NUL");
        // Closed loop EGO (M5)
        if ((uint8_t)ego.o2Type > 2) return fail("o2Type tidak valid");
        if (ego.enabled && ego.o2Type == O2Type::None)
            return fail("ego enabled tapi o2Type None");
        if (ego.nbSwitchV < 0.0f || ego.nbSwitchV > 1.2f) return fail("nbSwitchV 0..1.2 V");
        if (ego.nbHystV < 0.0f || ego.nbHystV > 0.5f ||
            ego.nbSwitchV - ego.nbHystV < 0.05f) return fail("nbHystV terlalu besar");
        if (ego.wbAdcLow < 0 || ego.wbAdcLow > 4095 ||
            ego.wbAdcHigh < 0 || ego.wbAdcHigh > 4095)
            return fail("wbAdc 0..4095");
        if (ego.wbAdcHigh <= ego.wbAdcLow + 200)
            return fail("wbAdcHigh harus > wbAdcLow + 200");
        if (ego.wbAfrLow < 5.0f || ego.wbAfrHigh > 25.0f ||
            ego.wbAfrHigh <= ego.wbAfrLow) return fail("wbAfr 5..25 dan naik");
        if (ego.limitPct < 0.0f || ego.limitPct > 30.0f) return fail("ego limitPct 0..30");
        if (ego.stepPct < 0.1f || ego.stepPct > 10.0f) return fail("ego stepPct 0.1..10");
        if (ego.deadbandAfr < 0.0f || ego.deadbandAfr > 1.0f) return fail("deadbandAfr 0..1");
        if (ego.updateMs < 10 || ego.updateMs > 2000) return fail("ego updateMs 10..2000");
        if (ego.sensorDelayMs > 60000) return fail("ego sensorDelayMs 0..60000");
        if (ego.minRpm < 100 || ego.minRpm > 25000 ||
            ego.maxRpm < 100 || ego.maxRpm > 25000 || ego.maxRpm <= ego.minRpm)
            return fail("ego rpm gate tidak valid");
        if (ego.maxTpsPct < 1.0f || ego.maxTpsPct > 100.0f) return fail("ego maxTpsPct 1..100");
        if (ego.minCltC < -20.0f || ego.minCltC > 120.0f) return fail("ego minCltC -20..120");
        return true;
    }

    static EcuConfig defaults() {
        EcuConfig c{};   // semua nilai NSDMI
        for (int i = 0; i < 16; ++i) {
            float rpm = 500.0f + (14500.0f / 15.0f) * (float)i;  // 500..15000 rpm
            float tps = (100.0f / 15.0f) * (float)i;             // 0..100 %
            c.fuel.ve.xAxis[i] = rpm;
            c.fuel.ve.yAxis[i] = tps;
            c.ign.advance.xAxis[i] = rpm;
            c.ign.advance.yAxis[i] = tps;
            c.ign.dwell.x[i] = rpm;
            c.ign.dwell.y[i] = 3.0f;
        }
        for (int iy = 0; iy < 16; ++iy)
            for (int ix = 0; ix < 16; ++ix) {
                c.fuel.ve.values[iy][ix] = 40.0f;      // VE placeholder - inert (safe mode)
                c.ign.advance.values[iy][ix] = 10.0f;  // advance placeholder - inert
            }
        // Kurva koreksi M4.1 - nilai masuk akal untuk motor (bukan inert):
        // WUE: -10 C -> +45% turun linear ke 0% di 80 C (grid: 10 C/titik)
        for (int i = 0; i < 16; ++i) {
            float clt = -10.0f + 10.0f * (float)i;              // -10..140 C
            c.fuel.wue.x[i] = clt;
            float y = 40.0f - clt * 0.5f;
            if (y < 0.0f) y = 0.0f;
            c.fuel.wue.y[i] = y;
        }
        // Dead time injektor (M4.2): kurva khas high-Z kecil, 12 V ~ 0.82 ms.
        // Grid 1 V dari 6..21 V; di luar rentang di-clamp oleh Curve1D.
        static const float DT_Y[16] = { 1.60f, 1.30f, 1.12f, 1.00f, 0.92f, 0.86f,
            0.82f, 0.79f, 0.77f, 0.75f, 0.74f, 0.73f, 0.72f, 0.71f, 0.71f, 0.70f };
        for (int i = 0; i < 16; ++i) {
            c.fuel.dtime.x[i] = 6.0f + (float)i;   // 6..21 V
            c.fuel.dtime.y[i] = DT_Y[i];
        }
        // IAT: koreksi densitas +0.3%/C di bawah 20 C, -0.3%/C di atas,
        // clamp +-8% (grid: 10 C/titik agar 0/20/40/80 C tepat di titik)
        for (int i = 0; i < 16; ++i) {
            float iat = -20.0f + 10.0f * (float)i;              // -20..130 C
            c.fuel.iatCorr.x[i] = iat;
            float y = (20.0f - iat) * 0.3f;
            if (y > 8.0f) y = 8.0f;
            if (y < -8.0f) y = -8.0f;
            c.fuel.iatCorr.y[i] = y;
        }
        // Target AFR (M5): sumbu sama dgn VE (rpm x load); default 14.7 di
        // throttle kecil melinier ke 12.8 di WOT. Inert sampai ego.enabled.
        for (int i = 0; i < 16; ++i) {
            float rpm = 500.0f + (14500.0f / 15.0f) * (float)i;
            float tps = (100.0f / 15.0f) * (float)i;
            c.ego.afrTarget.xAxis[i] = rpm;
            c.ego.afrTarget.yAxis[i] = tps;
        }
        for (int iy = 0; iy < 16; ++iy)
            for (int ix = 0; ix < 16; ++ix)
                c.ego.afrTarget.values[iy][ix] =
                    14.7f - (c.ego.afrTarget.yAxis[iy] / 100.0f) * 1.9f;
        return c;
    }
};

} // namespace ztune
