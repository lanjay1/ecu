// Test native M5: SensorMath O2 (o2Volts/o2Afr), EgoControl (wideband +
// narrowband, gate, deadband, sensor delay, clamp), pipeline FuelMath dgn
// egoPct, dan round-trip config closed loop.
#include "ztune/SensorMath.h"
#include "ztune/EgoControl.h"
#include "ztune/FuelMath.h"
#include "ztune/EcuConfig.h"
#include "ztune/Ztune.h"
#include "test_util.h"

using namespace ztune;

// Config ego wideband utk test: target tabel FLAT 14.7, step 1%/125 ms,
// limit 10%, delay 500 ms, gate 1000..12000 rpm / TPS<=70 / CLT>=70.
static EgoConfig wbEgo() {
    EcuConfig c = EcuConfig::defaults();
    c.ego.enabled = true;
    c.ego.o2Type = O2Type::Wideband;
    c.ego.sensorDelayMs = 500;   // dipercepat utk skenario test
    for (int iy = 0; iy < 16; ++iy)
        for (int ix = 0; ix < 16; ++ix)
            c.ego.afrTarget.values[iy][ix] = 14.7f;
    return c.ego;
}

static EgoControl::Input mkIn(uint32_t tMs, float afr, float rpm = 3000.0f,
                              float tps = 30.0f, float clt = 90.0f,
                              float o2V = 0.45f) {
    EgoControl::Input in{};
    in.tMs = tMs;
    in.rpm = rpm;
    in.load = tps;
    in.tpsPct = tps;
    in.cltC = clt;
    in.o2V = o2V;
    in.o2Afr = afr;
    in.o2Fault = false;
    in.fuelCut = false;
    in.aseActive = false;
    return in;
}

int main() {
    { // ---- SensorMath O2 ----
        // o2Volts: raw -> V full-scale 3.3
        CHECK_NEAR(SensorMath::o2Volts(2048), 1.6507, 0.001);
        CHECK_NEAR(SensorMath::o2Volts(0), 0.0, 1e-6);
        CHECK_NEAR(SensorMath::o2Volts(4095), 3.3, 1e-6);
        // o2Afr kalibrasi 2 titik: 500->9.0, 3500->19.0
        CHECK_NEAR(SensorMath::o2Afr(500, 500, 9.0f, 3500, 19.0f), 9.0, 1e-6);
        CHECK_NEAR(SensorMath::o2Afr(3500, 500, 9.0f, 3500, 19.0f), 19.0, 1e-6);
        // tengah: 2000 -> 9 + (1500/3000)*10 = 14.0
        CHECK_NEAR(SensorMath::o2Afr(2000, 500, 9.0f, 3500, 19.0f), 14.0, 1e-6);
        // clamp di luar rentang kalibrasi
        CHECK_NEAR(SensorMath::o2Afr(0, 500, 9.0f, 3500, 19.0f), 9.0, 1e-6);
        CHECK_NEAR(SensorMath::o2Afr(4095, 500, 9.0f, 3500, 19.0f), 19.0, 1e-6);
        // kalibrasi salah -> inert
        CHECK_NEAR(SensorMath::o2Afr(1000, 2000, 9.0f, 2000, 19.0f), 9.0, 1e-6);
    }
    { // ---- Wideband: gate & masa delay ----
        EgoControl ego;
        ego.configure(wbEgo());
        // Mesin dingin -> gate gagal
        auto s = ego.update(mkIn(100, 16.0f, 3000.0f, 30.0f, 40.0f));
        CHECK(!s.active);
        CHECK_NEAR(s.corrPct, 0.0, 1e-9);
        // Masih dingin di t lebih besar
        s = ego.update(mkIn(900, 16.0f, 3000.0f, 30.0f, 40.0f));
        CHECK(!s.active);
        // Hangat (t=1000): gate OK tapi masih masa delay (500 ms)
        s = ego.update(mkIn(1000, 16.0f));
        CHECK(s.active);
        CHECK_NEAR(s.corrPct, 0.0, 1e-9);
        CHECK_NEAR(s.targetAfr, 14.7, 1e-6);
        CHECK_NEAR(s.measuredAfr, 16.0, 1e-6);
        // Di dalam delay: tetap 0
        s = ego.update(mkIn(1400, 16.0f));
        CHECK(s.active);
        CHECK_NEAR(s.corrPct, 0.0, 1e-9);
        // Setelah delay + window update (125 ms): penyesuaian pertama +1
        s = ego.update(mkIn(1125 + 500, 16.0f));   // t = 1625: 1625-1000 >= 500, 1625-1000 >= 125
        CHECK_NEAR(s.corrPct, 1.0, 1e-6);
    }
    { // ---- Wideband: integrasi lean/rich/deadband/period/clamp ----
        EgoControl ego;
        ego.configure(wbEgo());
        // Lean 16.0 vs target 14.7 (deadband 0.1) -> +step tiap window
        ego.update(mkIn(0, 16.0f));
        ego.update(mkIn(600, 16.0f));    // adjust pertama (delay 500 terlewati)
        CHECK_NEAR(ego.state().corrPct, 1.0, 1e-6);
        ego.update(mkIn(650, 16.0f));    // < updateMs: tidak berubah
        CHECK_NEAR(ego.state().corrPct, 1.0, 1e-6);
        ego.update(mkIn(725, 16.0f));    // window berikut: +1 lagi
        CHECK_NEAR(ego.state().corrPct, 2.0, 1e-6);
        ego.update(mkIn(850, 16.0f));
        CHECK_NEAR(ego.state().corrPct, 3.0, 1e-6);
        // Rich 13.5 < 14.6 -> -step
        ego.update(mkIn(975, 13.5f));
        CHECK_NEAR(ego.state().corrPct, 2.0, 1e-6);
        // Deadband: 14.75 ada di 14.7 +- 0.1 -> tidak berubah
        ego.update(mkIn(1100, 14.75f));
        CHECK_NEAR(ego.state().corrPct, 2.0, 1e-6);
        // Lean lagi
        ego.update(mkIn(1225, 16.5f));
        CHECK_NEAR(ego.state().corrPct, 3.0, 1e-6);
        // Clamp atas: terus lean sampai tercapai limit 10
        for (uint32_t t = 1350; t <= 3000; t += 125)
            ego.update(mkIn(t, 18.0f));
        CHECK_NEAR(ego.state().corrPct, 10.0, 1e-6);
        CHECK(ego.state().corrPct == ego.state().corrPct);   // bukan NaN
    }
    { // ---- Wideband: clamp bawah -10 ----
        EgoControl ego;
        ego.configure(wbEgo());
        for (uint32_t t = 0; t <= 3000; t += 125)
            ego.update(mkIn(t, 11.0f));   // sangat rich
        CHECK_NEAR(ego.state().corrPct, -10.0, 1e-6);
    }
    { // ---- Gate keluar -> koreksi dibuang; masuk lagi -> delay ulang ----
        EgoControl ego;
        ego.configure(wbEgo());
        ego.update(mkIn(0, 16.0f));
        ego.update(mkIn(600, 16.0f));
        CHECK_NEAR(ego.state().corrPct, 1.0, 1e-6);
        // Fuel cut (rev limiter) -> keluar gate
        auto in = mkIn(700, 16.0f);
        in.fuelCut = true;
        auto s = ego.update(in);
        CHECK(!s.active);
        CHECK_NEAR(s.corrPct, 0.0, 1e-6);
        // Gate OK lagi -> harus lewat delay 500 ms dulu (mulai dari 700)
        ego.update(mkIn(800, 16.0f));
        CHECK_NEAR(ego.state().corrPct, 0.0, 1e-6);
        ego.update(mkIn(1150, 16.0f));   // 1150-800 = 350 < 500: masih delay
        CHECK_NEAR(ego.state().corrPct, 0.0, 1e-6);
        ego.update(mkIn(1350, 16.0f));   // 1350-800 = 550 >= 500, >= 125: adjust
        CHECK_NEAR(ego.state().corrPct, 1.0, 1e-6);
    }
    { // ---- Gate variatif: TPS, rpm, fault, ASE ----
        EgoControl ego;
        ego.configure(wbEgo());
        ego.update(mkIn(0, 16.0f));
        ego.update(mkIn(600, 16.0f));
        CHECK_NEAR(ego.state().corrPct, 1.0, 1e-6);
        // TPS di atas max (power regime) -> open loop
        auto s1 = ego.update(mkIn(700, 16.0f, 3000.0f, 90.0f));
        CHECK(!s1.active);
        // rpm di bawah minRpm
        auto s2 = ego.update(mkIn(700, 16.0f, 500.0f));
        CHECK(!s2.active);
        // O2 fault
        auto in3 = mkIn(700, 16.0f);
        in3.o2Fault = true;
        auto s3 = ego.update(in3);
        CHECK(!s3.active);
        // ASE aktif
        auto in4 = mkIn(700, 16.0f);
        in4.aseActive = true;
        auto s4 = ego.update(in4);
        CHECK(!s4.active);
        // semua normal lagi -> gate OK (delay dari 700)
        auto s5 = ego.update(mkIn(1300, 16.0f));
        CHECK(s5.active);
    }
    { // ---- Master disabled / o2Type None -> tidak pernah aktif ----
        EgoControl ego;
        EgoConfig e = wbEgo();
        e.enabled = false;
        ego.configure(e);
        for (uint32_t t = 0; t <= 5000; t += 125)
            ego.update(mkIn(t, 18.0f));
        CHECK(!ego.state().active);
        CHECK_NEAR(ego.state().corrPct, 0.0, 1e-9);

        EgoControl ego2;
        EgoConfig e2 = wbEgo();
        e2.o2Type = O2Type::None;
        ego2.configure(e2);
        for (uint32_t t = 0; t <= 5000; t += 125)
            ego2.update(mkIn(t, 18.0f));
        CHECK(!ego2.state().active);
    }
    { // ---- Target tabel ter-interpolasi (bukan flat) ----
        EgoControl ego;
        EgoConfig e = wbEgo();
        // Target = 14.7 + rpm/15000 (naik dgn rpm; nilai arbitrer utk test)
        for (int iy = 0; iy < 16; ++iy)
            for (int ix = 0; ix < 16; ++ix)
                e.afrTarget.values[iy][ix] =
                    14.7f + e.afrTarget.xAxis[ix] / 15000.0f;
        ego.configure(e);
        // rpm 7500 -> target ~ 14.7 + 0.5 = 15.2 (node tengah tabel)
        auto s = ego.update(mkIn(0, 15.2f, 7500.0f));
        CHECK_NEAR(s.targetAfr, 15.2, 0.02);
        // 15.2 di deadband -> tidak ada penyesuaian setelah delay
        ego.update(mkIn(600, 15.2f, 7500.0f));
        CHECK_NEAR(ego.state().corrPct, 0.0, 1e-9);
        // lean kecil di luar deadband
        ego.update(mkIn(725, 15.4f, 7500.0f));
        CHECK_NEAR(ego.state().corrPct, 1.0, 1e-6);
    }
    { // ---- Narrowband: osilasi di sekitar switch point ----
        EgoControl ego;
        EcuConfig c = EcuConfig::defaults();
        c.ego.enabled = true;
        c.ego.o2Type = O2Type::Narrowband;
        c.ego.sensorDelayMs = 500;   // dipercepat utk skenario test
        ego.configure(c.ego);
        // Rich: 0.85 V > 0.45+0.05 -> -step (pangkas bahan bakar)
        ego.update(mkIn(0, 0.0f, 3000.0f, 30.0f, 90.0f, 0.85f));  // afr diabaikan NB
        auto s = ego.update(mkIn(600, 0.0f, 3000.0f, 30.0f, 90.0f, 0.85f));
        CHECK(s.active);
        CHECK_NEAR(s.corrPct, -1.0, 1e-6);
        CHECK_NEAR(s.targetAfr, 0.0, 1e-9);   // NB: target AFR tidak dipakai
        CHECK_NEAR(s.measuredAfr, 0.0, 1e-9);
        // Dalam band histeresis (0.40..0.50): tidak berubah
        ego.update(mkIn(725, 0.0f, 3000.0f, 30.0f, 90.0f, 0.45f));
        CHECK_NEAR(ego.state().corrPct, -1.0, 1e-6);
        // Lean: 0.10 V < 0.40 -> +step (osilasi khas NB)
        ego.update(mkIn(850, 0.0f, 3000.0f, 30.0f, 90.0f, 0.10f));
        CHECK_NEAR(ego.state().corrPct, 0.0, 1e-6);
        // Rich lagi -> turun lagi
        ego.update(mkIn(975, 0.0f, 3000.0f, 30.0f, 90.0f, 0.90f));
        CHECK_NEAR(ego.state().corrPct, -1.0, 1e-6);
        // Clamp NB: lean terus -> naik sampai limit +10
        EgoControl ego2;
        ego2.configure(c.ego);
        for (uint32_t t = 0; t <= 5000; t += 125) {
            auto in = mkIn(t, 0.0f, 3000.0f, 30.0f, 90.0f, 0.05f);
            ego2.update(in);
        }
        CHECK_NEAR(ego2.state().corrPct, 10.0, 1e-6);
    }
    { // ---- Pipeline FuelMath: egoPct masuk korektor ----
        float rf = FuelMath::reqFuelMs(125.0f, 14.7f, 160.0f);   // 5.742 ms
        // Tanpa koreksi apa pun
        CHECK_NEAR(FuelMath::correctedPwMs(rf, 0, 0, 0, 0), rf, 1e-6);
        // EGO +10% saja
        CHECK_NEAR(FuelMath::correctedPwMs(rf, 0, 0, 0, 10.0f), rf * 1.10, 1e-6);
        // Gabungan WUE 45 + EGO -10 -> mult 1.35
        CHECK_NEAR(FuelMath::correctedPwMs(rf, 45.0f, 0, 0, -10.0f), rf * 1.35, 1e-6);
        // Clamp keselamatan 0.2x..3.0x tetap berlaku
        CHECK_NEAR(FuelMath::correctedPwMs(rf, 0, 0, 0, 500.0f), rf * 3.0, 1e-6);
        CHECK_NEAR(FuelMath::correctedPwMs(rf, -100.0f, 0, 0, -100.0f), rf * 0.2, 1e-6);
    }
    { // ---- Validasi EcuConfig bagian EGO ----
        char err[128];
        EcuConfig c = EcuConfig::defaults();
        CHECK(c.validate(err, sizeof(err)));    // default (None, disabled) valid

        c = EcuConfig::defaults();
        c.ego.enabled = true;                   // enabled tapi o2Type None
        CHECK(!c.validate(err, sizeof(err)));

        c = EcuConfig::defaults();
        c.ego.o2Type = (O2Type)7;               // tipe tak dikenal
        CHECK(!c.validate(err, sizeof(err)));

        c = EcuConfig::defaults();
        c.ego.o2Type = O2Type::Wideband;
        c.ego.limitPct = 50.0f;                 // di luar 0..30
        CHECK(!c.validate(err, sizeof(err)));

        c = EcuConfig::defaults();
        c.ego.o2Type = O2Type::Wideband;
        c.ego.wbAdcHigh = 600;                  // <= low + 200
        CHECK(!c.validate(err, sizeof(err)));

        c = EcuConfig::defaults();
        c.ego.o2Type = O2Type::Wideband;
        c.ego.wbAfrHigh = 8.0f;                 // < afrLow
        CHECK(!c.validate(err, sizeof(err)));

        c = EcuConfig::defaults();
        c.ego.o2Type = O2Type::Wideband;
        c.ego.stepPct = 0.05f;                  // < 0.1
        CHECK(!c.validate(err, sizeof(err)));

        c = EcuConfig::defaults();
        c.ego.o2Type = O2Type::Narrowband;
        c.ego.updateMs = 5;                     // < 10
        CHECK(!c.validate(err, sizeof(err)));

        c = EcuConfig::defaults();
        c.ego.o2Type = O2Type::Narrowband;
        c.ego.minRpm = 8000;
        c.ego.maxRpm = 5000;                    // max <= min
        CHECK(!c.validate(err, sizeof(err)));

        c = EcuConfig::defaults();
        c.ego.o2Type = O2Type::Narrowband;
        c.ego.maxTpsPct = 0.5f;                 // < 1
        CHECK(!c.validate(err, sizeof(err)));

        c = EcuConfig::defaults();
        c.ego.o2Type = O2Type::Wideband;
        c.ego.minCltC = -30.0f;                 // < -20
        CHECK(!c.validate(err, sizeof(err)));

        c = EcuConfig::defaults();
        c.ego.o2Type = O2Type::Wideband;
        c.ego.sensorDelayMs = 61000;            // > 60000
        CHECK(!c.validate(err, sizeof(err)));

        c = EcuConfig::defaults();
        c.ego.o2Type = O2Type::Wideband;
        c.ego.enabled = true;
        CHECK(c.validate(err, sizeof(err)));    // WB lengkap -> valid

        // io.o2Gpio wajib ADC1
        c = EcuConfig::defaults();
        c.io.o2Gpio = 12;
        CHECK(!c.validate(err, sizeof(err)));
    }
    { // ---- Round-trip blob membawa kalibrasi EGO utuh ----
        EcuConfig a = EcuConfig::defaults();
        a.ego.o2Type = O2Type::Wideband;
        a.ego.enabled = true;
        a.ego.wbAfrLow = 8.5f;
        a.ego.afrTarget.values[5][5] = 13.1f;
        uint8_t buf[sizeof(EcuConfig)];
        memcpy(buf, &a, sizeof(buf));
        EcuConfig b;
        memcpy(&b, buf, sizeof(buf));
        CHECK(b.ego.o2Type == O2Type::Wideband);
        CHECK(b.ego.enabled);
        CHECK_NEAR(b.ego.wbAfrLow, 8.5, 1e-9);
        CHECK_NEAR(b.ego.afrTarget.values[5][5], 13.1f, 1e-9);
        CHECK(crc32(&b, sizeof(EcuConfig)) == crc32(buf, sizeof(buf)));
    }
    { // ---- Default tabel target AFR masuk akal ----
        EcuConfig c = EcuConfig::defaults();
        CHECK_NEAR(c.ego.afrTarget.lookup(1000.0f, 10.0f), 14.7 - 0.19, 0.05);
        CHECK_NEAR(c.ego.afrTarget.lookup(1000.0f, 100.0f), 12.8, 0.01);
        CHECK_NEAR(c.ego.afrTarget.lookup(8000.0f, 50.0f), 14.7 - 0.95, 0.05);
    }
    return report("ego");
}
