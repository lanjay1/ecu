// Test native M4.2: kurva dead time injektor vs tegangan baterai.
// Nilai expected dihitung manual dari kurva default (grid 1 V, 6..21 V):
// 6 V = 1.60 ms, 12 V = 0.82 ms, 21 V = 0.70 ms; interpolasi linier antar
// titik; di luar rentang di-clamp ke ujung.
#include "ztune/EcuConfig.h"
#include "ztune/FuelMath.h"
#include "ztune/Ztune.h"
#include "test_util.h"
#include <cstring>

using namespace ztune;

int main() {
    char err[128] = {0};

    { // ---- kurva default: grid & monotonic ----
        EcuConfig c = EcuConfig::defaults();
        CHECK(ZTUNE_SCHEMA_VERSION == 5u);
        CHECK_NEAR(c.fuel.dtime.x[0], 6.0, 1e-6);
        CHECK_NEAR(c.fuel.dtime.x[15], 21.0, 1e-6);
        for (int i = 0; i < 15; ++i)
            CHECK(c.fuel.dtime.x[i + 1] > c.fuel.dtime.x[i]);
        CHECK(c.validate(err, sizeof(err)));
    }
    { // ---- lookup tepat di titik grid ----
        EcuConfig c = EcuConfig::defaults();
        CHECK_NEAR(c.fuel.dtime.lookup(6.0f), 1.60, 1e-5);
        CHECK_NEAR(c.fuel.dtime.lookup(7.0f), 1.30, 1e-5);
        CHECK_NEAR(c.fuel.dtime.lookup(12.0f), 0.82, 1e-5);
        CHECK_NEAR(c.fuel.dtime.lookup(21.0f), 0.70, 1e-5);
    }
    { // ---- interpolasi linier antar titik ----
        EcuConfig c = EcuConfig::defaults();
        CHECK_NEAR(c.fuel.dtime.lookup(11.5f), 0.84, 1e-5);   // (0.86+0.82)/2
        CHECK_NEAR(c.fuel.dtime.lookup(9.5f), 0.96, 1e-5);    // (1.00+0.92)/2
    }
    { // ---- clamp di luar rentang ----
        EcuConfig c = EcuConfig::defaults();
        CHECK_NEAR(c.fuel.dtime.lookup(4.0f), 1.60, 1e-5);    // cranking < 6 V
        CHECK_NEAR(c.fuel.dtime.lookup(0.0f), 1.60, 1e-5);
        CHECK_NEAR(c.fuel.dtime.lookup(25.0f), 0.70, 1e-5);
    }
    { // ---- pipeline PW: volt rendah -> PW eksekusi membesar ----
        // reqFuel 125 cc/cyl, stoich 14.7, inj 160 cc/min = 5.7422 ms;
        // 2 squirt -> base 2.8711 ms + dead time.
        EcuConfig c = EcuConfig::defaults();
        float req = FuelMath::reqFuelMs(125.0f, 14.7f, 160.0f);
        float pw6  = FuelMath::pulseWidthMs(req, 100.0f, 2,
                                            c.fuel.dtime.lookup(6.0f));
        float pw12 = FuelMath::pulseWidthMs(req, 100.0f, 2,
                                            c.fuel.dtime.lookup(12.0f));
        CHECK(pw6 > pw12);
        CHECK_NEAR(pw12, req / 2.0f + 0.82, 1e-4);
        CHECK_NEAR(pw6,  req / 2.0f + 1.60, 1e-4);
        // dead time naik saat tegangan turun (cranking)
        float d18V = c.fuel.dtime.lookup(18.0f);
        CHECK(d18V < pw12 - req / 2.0f + 1e-6);   // 18 V lebih kecil dari 12 V
    }
    { // ---- validasi config menolak kurva rusak ----
        EcuConfig c = EcuConfig::defaults();
        c.fuel.dtime.y[3] = 5.1f;
        CHECK(!c.validate(err, sizeof(err)));
        c = EcuConfig::defaults();
        c.fuel.dtime.y[3] = -0.1f;
        CHECK(!c.validate(err, sizeof(err)));
        c = EcuConfig::defaults();
        c.fuel.dtime.x[5] = c.fuel.dtime.x[4];    // tidak naik ketat
        CHECK(!c.validate(err, sizeof(err)));
        c = EcuConfig::defaults();
        c.fuel.dtime.x[5] = 31.0f;                // x di luar 0..30
        CHECK(!c.validate(err, sizeof(err)));
    }
    { // ---- round trip blob NVS: CRC & nilai utuh ----
        EcuConfig c = EcuConfig::defaults();
        uint8_t blob[sizeof(EcuConfig)];
        std::memcpy(blob, &c, sizeof(EcuConfig));
        EcuConfig c2;
        std::memcpy(&c2, blob, sizeof(EcuConfig));
        CHECK(crc32(&c2, sizeof(EcuConfig)) == crc32(&c, sizeof(EcuConfig)));
        CHECK(c2.validate(err, sizeof(err)));
        CHECK_NEAR(c2.fuel.dtime.lookup(12.0f), 0.82, 1e-6);
        CHECK_NEAR(c2.fuel.dtime.lookup(6.5f), 1.45, 1e-6);   // (1.60+1.30)/2
    }

    return report("test_deadtime");
}
