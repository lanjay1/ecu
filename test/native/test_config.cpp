// Test native EcuConfig: validasi default, penolakan nilai invalid,
// dan round-trip blob (simulasi persist NVS + CRC).
#include "ztune/EcuConfig.h"
#include "test_util.h"
#include <cstring>

using namespace ztune;

int main() {
    { // Default pabrik HARUS valid (safe-mode, mesin terkunci)
        EcuConfig c = EcuConfig::defaults();
        char err[128] = {0};
        CHECK(c.validate(err, sizeof(err)));
        CHECK(!c.tuneAccepted);   // gate safe mode aktif dari pabrik
    }
    { // Validasi menolak nilai tidak masuk akal
        EcuConfig c = EcuConfig::defaults();
        c.trigger.totalTeeth = 0;
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults(); c.engine.cylinders = 0;
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults(); c.engine.squirtsPerCycle = 0;
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults(); c.fuel.injectorCcPerMin = 0.0f;
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults(); c.trigger.gapRatio = 0.5f;
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults(); c.io.ckpGpio = 99;
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults();
        c.engine.stroke = StrokeType::TwoStroke;
        c.engine.camSensorPresent = true;
        CHECK(!c.validate(nullptr, 0));   // 2-tak + cam = konfigurasi salah
    }
    { // M4: validasi field scheduler/limiter/quickshifter
        EcuConfig c = EcuConfig::defaults();
        c.fuel.injEndBeforeTdcDeg = 200.0f;
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults(); c.trigger.phaseOffsetDeg = 400.0f;
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults(); c.revLimit.hardLimitRpm = 500;
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults(); c.revLimit.resumeDropRpm = 12000;  // >= hardLimit
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults(); c.quickShift.cutMs = 5;
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults(); c.io.qsInputGpio = 99;
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults(); c.io.injGpio[3] = 60;
        CHECK(!c.validate(nullptr, 0));

        // Nilai valid di batas atas tetap diterima
        c = EcuConfig::defaults();
        c.trigger.phaseOffsetDeg = 359.0f;
        c.fuel.injEndBeforeTdcDeg = 180.0f;
        c.revLimit.hardLimitRpm = 25000;
        c.quickShift.cutMs = 500;
        CHECK(c.validate(nullptr, 0));
        CHECK(c.revLimit.hardLimitRpm == 25000);
    }
    { // M3.1: validasi Comms (transport BLE/WiFi)
        EcuConfig c = EcuConfig::defaults();
        CHECK(c.validate(nullptr, 0));               // default: BLE on, WiFi off
        CHECK(c.comms.bleEnable);
        CHECK(!c.comms.wifiEnable);

        c = EcuConfig::defaults();
        c.comms.wifiEnable = true;                   // wifi on tapi ssid kosong
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults();
        c.comms.wifiEnable = true;                   // ssid tanpa NUL
        memset(c.comms.wifiSsid, 'x', sizeof(c.comms.wifiSsid));
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults();
        c.comms.wifiEnable = true;                   // pass tanpa NUL
        memset(c.comms.wifiPass, 'y', sizeof(c.comms.wifiPass));
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults();
        c.comms.wifiEnable = true;
        snprintf(c.comms.wifiSsid, sizeof(c.comms.wifiSsid), "HomeNet");
        snprintf(c.comms.wifiPass, sizeof(c.comms.wifiPass), "secret123");
        c.comms.wifiTcpPort = 7777;
        CHECK(c.validate(nullptr, 0));               // WiFi lengkap -> valid

        c = EcuConfig::defaults();                   // bleName tanpa NUL
        memset(c.comms.bleName, 'z', sizeof(c.comms.bleName));
        CHECK(!c.validate(nullptr, 0));

        // Round-trip blob tetap konsisten dengan field Comms
        EcuConfig a = EcuConfig::defaults();
        snprintf(a.comms.wifiSsid, sizeof(a.comms.wifiSsid), "NetA");
        uint8_t buf[sizeof(EcuConfig)];
        memcpy(buf, &a, sizeof(buf));
        EcuConfig b;
        memcpy(&b, buf, sizeof(buf));
        CHECK(crc32(&b, sizeof(EcuConfig)) == crc32(buf, sizeof(buf)));
        CHECK(strcmp(b.comms.wifiSsid, "NetA") == 0);
        CHECK(b.comms.wifiTcpPort == 6666);
    }
    { // M4.1: validasi sensor + kurva koreksi
        EcuConfig c = EcuConfig::defaults();
        CHECK(c.validate(nullptr, 0));               // default valid

        c = EcuConfig::defaults();
        c.sensors.tpsWotAdc = 200;                   // <= closed + 200
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults();
        c.sensors.mapFullAdc = 500;                  // < zero
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults();
        c.sensors.cltBetaK = 500.0f;                 // beta di luar rentang
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults();
        c.io.tpsGpio = 15;                           // di luar ADC1 (GPIO 1..10)
        CHECK(!c.validate(nullptr, 0));

        c = EcuConfig::defaults();
        c.fuel.asePct = 150.0f;
        CHECK(!c.validate(nullptr, 0));

        // Kurva default terisi & masuk akal
        c = EcuConfig::defaults();
        CHECK_NEAR(c.fuel.wue.lookup(-10.0f), 45.0, 1e-6);
        CHECK_NEAR(c.fuel.wue.lookup(100.0f), 0.0, 1e-6);
        CHECK_NEAR(c.fuel.iatCorr.lookup(20.0f), 0.0, 1e-6);

        // Round-trip blob membawa kalibrasi sensor utuh
        EcuConfig a = EcuConfig::defaults();
        a.sensors.tpsClosedAdc = 555;
        a.sensors.battDivRatio = 12.5f;
        uint8_t buf[sizeof(EcuConfig)];
        memcpy(buf, &a, sizeof(buf));
        EcuConfig b;
        memcpy(&b, buf, sizeof(buf));
        CHECK(b.sensors.tpsClosedAdc == 555);
        CHECK_NEAR(b.sensors.battDivRatio, 12.5, 1e-9);
        CHECK(crc32(&b, sizeof(EcuConfig)) == crc32(buf, sizeof(buf)));
    }
    { // Round-trip blob (simulasi NVS): memcpy + CRC tetap konsisten
        EcuConfig a = EcuConfig::defaults();
        uint8_t buf[sizeof(EcuConfig)];
        memcpy(buf, &a, sizeof(buf));
        uint32_t crc = crc32(buf, sizeof(buf));

        EcuConfig b;
        memcpy(&b, buf, sizeof(buf));
        CHECK(crc32(&b, sizeof(EcuConfig)) == crc);
        CHECK(b.trigger.totalTeeth == a.trigger.totalTeeth);
        CHECK(b.trigger.gapRatio == a.trigger.gapRatio);
        CHECK_NEAR(b.fuel.ve.values[3][4], a.fuel.ve.values[3][4], 1e-9);
        CHECK(!b.tuneAccepted);
    }
    { // Opsi A tanpa cam: config default harus konsisten (wasted spark, 2 squirt)
        EcuConfig c = EcuConfig::defaults();
        CHECK(!c.engine.camSensorPresent);
        CHECK(c.engine.squirtsPerCycle == 2);
        CHECK(c.trigger.totalTeeth == 12 && c.trigger.missingTeeth == 1);
    }
    { // M5: default EGO + sumbu tabel target AFR + gate GPIO O2
        EcuConfig c = EcuConfig::defaults();
        CHECK(c.ego.o2Type == O2Type::None);
        CHECK(!c.ego.enabled);
        CHECK(c.io.o2Gpio == 8);
        CHECK_NEAR(c.ego.afrTarget.xAxis[0], 500.0, 1e-3);
        CHECK_NEAR(c.ego.afrTarget.xAxis[15], 15000.0, 1e-3);
        CHECK_NEAR(c.ego.afrTarget.yAxis[15], 100.0, 1e-3);
        CHECK_NEAR(c.ego.afrTarget.values[0][0], 14.7, 1e-6);
        CHECK_NEAR(c.ego.afrTarget.values[15][15], 12.8, 1e-6);
        // WB kalibrasi default -> config tetap valid
        c.ego.o2Type = O2Type::Wideband;
        CHECK(c.validate(nullptr, 0));
        // NB: histeresis kebesaran (switch - hyst < 0.05 V) -> ditolak
        c.ego.nbHystV = 0.45f;
        CHECK(!c.validate(nullptr, 0));
    }
    return report("config");
}
