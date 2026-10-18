// Test native M4.1: SensorMath (TPS/MAP/NTC beta/Batt), EmaFilter,
// AccelEnrichment, koreksi PW, dan kurva WUE/IAT defaults.
// Nilai NTC dihitung manual (beta 3380, R0 10k, pullup 10k):
//   25 C -> R=10k0 -> raw=2047; 80 C -> R=1712 -> raw=598; 0 C -> R=28.2k -> raw=3022
#include "ztune/SensorMath.h"
#include "ztune/AccelEnrichment.h"
#include "ztune/FuelMath.h"
#include "ztune/EcuConfig.h"
#include "test_util.h"

using namespace ztune;

int main() {
    { // ---- TPS: linier 2 titik + clamp ----
        CHECK_NEAR(SensorMath::tpsPct(100, 100, 3000), 0.0, 1e-6);
        CHECK_NEAR(SensorMath::tpsPct(3000, 100, 3000), 100.0, 1e-6);
        CHECK_NEAR(SensorMath::tpsPct(1550, 100, 3000), 50.0, 1e-6);
        CHECK_NEAR(SensorMath::tpsPct(50, 100, 3000), 0.0, 1e-6);     // clamp bawah
        CHECK_NEAR(SensorMath::tpsPct(3500, 100, 3000), 100.0, 1e-6); // clamp atas
        CHECK_NEAR(SensorMath::tpsPct(2000, 3000, 3000), 0.0, 1e-6);  // kalibrasi salah
    }
    { // ---- MAP: linier 2 titik + clamp ----
        CHECK_NEAR(SensorMath::mapKpa(620, 620, 2900, 20.0f, 250.0f), 20.0, 1e-6);
        CHECK_NEAR(SensorMath::mapKpa(2900, 620, 2900, 20.0f, 250.0f), 250.0, 1e-6);
        CHECK_NEAR(SensorMath::mapKpa(1760, 620, 2900, 20.0f, 250.0f), 135.0, 1e-6);
        CHECK_NEAR(SensorMath::mapKpa(100, 620, 2900, 20.0f, 250.0f), 20.0, 1e-6);
        CHECK_NEAR(SensorMath::mapKpa(4000, 620, 2900, 20.0f, 250.0f), 250.0, 1e-6);
        CHECK_NEAR(SensorMath::mapKpa(100, 620, 620, 20.0f, 250.0f), 20.0, 1e-6); // div 0
    }
    { // ---- NTC beta: titik kalibrasi nyata (beta 3380, R0 10k, pullup 10k) ----
        CHECK_NEAR(SensorMath::ntcC(2047, 10000.0f, 3380.0f, 10000.0f, 25.0f), 25.0, 0.2);
        CHECK_NEAR(SensorMath::ntcC(598,  10000.0f, 3380.0f, 10000.0f, 25.0f), 80.0, 0.3);
        CHECK_NEAR(SensorMath::ntcC(3022, 10000.0f, 3380.0f, 10000.0f, 25.0f), 0.0, 0.3);
        CHECK_NEAR(SensorMath::ntcC(4093, 10000.0f, 3380.0f, 10000.0f, 25.0f), -100.0, 1e-6); // open
        CHECK_NEAR(SensorMath::ntcC(1,    10000.0f, 3380.0f, 10000.0f, 25.0f), -100.0, 1e-6); // short
    }
    { // ---- Batt via pembagi ----
        CHECK_NEAR(SensorMath::battV(2047, 11.0f), 18.15, 0.05);
        CHECK_NEAR(SensorMath::battV(4095, 11.0f), 36.3, 0.05);
        CHECK_NEAR(SensorMath::battV(0, 11.0f), 0.0, 1e-6);
    }
    { // ---- EmaFilter ----
        EmaFilter e(0.25f);
        CHECK_NEAR(e.update(100.0f), 100.0, 1e-6);    // sampel pertama = langsung
        CHECK_NEAR(e.update(100.0f), 100.0, 1e-6);
        CHECK_NEAR(e.update(200.0f), 125.0, 1e-6);    // 100 + 0.25x(200-100)
        CHECK_NEAR(e.update(200.0f), 143.75, 1e-6);
        e.reset();
        CHECK(!e.valid());
        CHECK_NEAR(e.update(50.0f), 50.0, 1e-6);
    }
    { // ---- AccelEnrichment: tip-in, ramp-down, retrigger ----
        AccelEnrichment a;
        a.configure(40.0f /*%/s*/, 20.0f, 600 /*ms*/);
        a.reset();

        // TPS diam lalu tip-in cepat: 10% -> 30% dalam 200 ms = 100 %/s
        a.onTps(10.0f, 1000000);
        CHECK(!a.active());
        a.onTps(30.0f, 1200000);
        CHECK(a.active());
        CHECK_NEAR(a.enrichPct(1200000), 20.0, 1e-6);      // t=0 -> asePct
        CHECK_NEAR(a.enrichPct(1350000), 15.0, 1e-6);      // t=150 ms (25% window)
        CHECK_NEAR(a.enrichPct(1500000), 10.0, 1e-6);      // t=300 ms (50% window)
        CHECK_NEAR(a.enrichPct(1799999), 0.0, 0.01);       // nyaris expire
        CHECK(a.active());
        CHECK_NEAR(a.enrichPct(1800000), 0.0, 1e-6);       // expire tepat 600 ms
        CHECK(!a.active());

        // Buka pelan: 2.5 %/s -> tidak trigger
        a.onTps(10.0f, 2000000);
        a.onTps(15.0f, 4000000);
        CHECK(!a.active());
        CHECK_NEAR(a.enrichPct(4000001), 0.0, 1e-6);

        // Retrigger setelah expire
        a.onTps(30.0f, 4100000);                           // 15->30 dalam 100 ms
        CHECK(a.active());
        CHECK(a.trips() == 2);

        // Sample < 1 ms diabaikan (noise)
        AccelEnrichment n;
        n.configure(40.0f, 20.0f, 600);
        n.onTps(0.0f, 5000);
        n.onTps(100.0f, 5400);                             // dt 400 us -> skip
        CHECK(!n.active());

        // Saat aktif, tip-in lagi tidak extend
        AccelEnrichment e2;
        e2.configure(40.0f, 20.0f, 600);
        e2.onTps(0.0f, 0);
        e2.onTps(50.0f, 200000);                           // 250 %/s -> trigger
        uint32_t t1 = e2.trips();
        e2.onTps(100.0f, 300000);                          // masih aktif -> abaikan
        CHECK(e2.trips() == t1);
    }
    { // ---- Koreksi PW ----
        CHECK_NEAR(FuelMath::correctedPwMs(10.0f, 0, 0, 0), 10.0, 1e-6);
        CHECK_NEAR(FuelMath::correctedPwMs(10.0f, 20.0f, 0.0f, 10.0f), 13.0, 1e-6);
        CHECK_NEAR(FuelMath::correctedPwMs(10.0f, 500.0f, 0, 0), 30.0, 1e-6);   // clamp 3x
        CHECK_NEAR(FuelMath::correctedPwMs(10.0f, -100.0f, -50.0f, 0), 2.0, 1e-6); // clamp 0.2x
        CHECK_NEAR(FuelMath::correctedPwMs(0.0f, 40.0f, 0, 0), 0.0, 1e-6);
    }
    { // ---- Kurva WUE/IAT default + pipeline PW end-to-end ----
        EcuConfig c = EcuConfig::defaults();
        CHECK_NEAR(c.fuel.wue.lookup(-10.0f), 45.0, 1e-6);  // 40 - (-10x0.5)
        CHECK_NEAR(c.fuel.wue.lookup(20.0f), 30.0, 1e-6);   // 40 - 20x0.5
        CHECK_NEAR(c.fuel.wue.lookup(80.0f), 0.0, 1e-6);
        CHECK_NEAR(c.fuel.wue.lookup(150.0f), 0.0, 1e-6);

        CHECK_NEAR(c.fuel.iatCorr.lookup(20.0f), 0.0, 1e-6);
        CHECK_NEAR(c.fuel.iatCorr.lookup(0.0f), 6.0, 1e-6);
        CHECK_NEAR(c.fuel.iatCorr.lookup(40.0f), -6.0, 1e-6);
        CHECK_NEAR(c.fuel.iatCorr.lookup(-20.0f), 8.0, 1e-6);  // clamp +8
        CHECK_NEAR(c.fuel.iatCorr.lookup(80.0f), -8.0, 1e-6);  // clamp -8

        // Pipeline contoh FuelMath: 500 cc/cyl, inj 440, stoich 14.7
        float reqFuel = FuelMath::reqFuelMs(500.0f, 14.7f, 440.0f);
        CHECK_NEAR(reqFuel, 8.352, 1e-3);
        float pwBase = FuelMath::pulseWidthMs(reqFuel, 40.0f, 2, 0.8f);
        CHECK_NEAR(pwBase, 2.4704, 1e-3);
        float pw = FuelMath::correctedPwMs(pwBase, 10.0f, 2.0f, 15.0f);
        CHECK_NEAR(pw, 3.1374, 1e-3);                       // x1.27
    }
    return report("sensor");
}
