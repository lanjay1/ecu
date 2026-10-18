// Test native FuelMath (konvensi MegaSquirt), InterpTable, dan CRC32.
#include "ztune/FuelMath.h"
#include "ztune/InterpTable.h"
#include "ztune/Ztune.h"
#include "test_util.h"

using namespace ztune;

int main() {
    { // reqFuel gaya MegaSquirt
        // 4-sil 2000cc (500 cc/cyl), inj 440 cc/min, stoich 14.7 -> 8.352 ms
        CHECK_NEAR(FuelMath::reqFuelMs(500.0f, 14.7f, 440.0f), 8.35227, 0.01);
        // 1-sil 125cc, inj 160 cc/min -> 5.742 ms
        CHECK_NEAR(FuelMath::reqFuelMs(125.0f, 14.7f, 160.0f), 5.74219, 0.01);
        // guard pembagi nol
        CHECK(FuelMath::reqFuelMs(0.0f, 14.7f, 440.0f) == 0.0f);
        CHECK(FuelMath::reqFuelMs(500.0f, 14.7f, 0.0f) == 0.0f);
    }
    { // PW = reqFuel x VE% / squirts + dead time
        float rf = FuelMath::reqFuelMs(500.0f, 14.7f, 440.0f);
        CHECK_NEAR(FuelMath::pulseWidthMs(rf, 100.0f, 1, 0.0f), 8.35227, 0.01);
        // Opsi A crank-only: 2 squirt per siklus, VE 80%, dead time 0.8 ms
        CHECK_NEAR(FuelMath::pulseWidthMs(rf, 80.0f, 2, 0.8f), 4.14091, 0.01);
        // VE 0 -> hanya dead time
        CHECK_NEAR(FuelMath::pulseWidthMs(rf, 0.0f, 2, 0.8f), 0.8, 0.001);
        // guard squirts = 0
        CHECK(FuelMath::pulseWidthMs(rf, 100.0f, 0, 0.0f) == 0.0f);
    }
    { // Tabel 2D 16x16: interpolasi bilinear + clamp tepi
        Table2D<16, 16> t;
        for (int i = 0; i < 16; ++i) {
            t.xAxis[i] = 1000.0f * i;             // 0..15000 (rpm)
            t.yAxis[i] = (100.0f / 15.0f) * i;    // 0..100 (TPS %)
        }
        for (int iy = 0; iy < 16; ++iy)
            for (int ix = 0; ix < 16; ++ix)
                t.values[iy][ix] = t.xAxis[ix] * 0.01f + t.yAxis[iy] * 0.1f;

        CHECK_NEAR(t.lookup(5000.0f, 50.0f), 55.0, 1e-3);   // tepat di node
        CHECK_NEAR(t.lookup(5500.0f, 50.0f), 60.0, 1e-3);   // tengah sel x
        CHECK_NEAR(t.lookup(5000.0f, 55.0f), 55.5, 1e-3);   // tengah sel y
        CHECK_NEAR(t.lookup(-100.0f, 50.0f), 5.0, 1e-3);    // clamp bawah x
        CHECK_NEAR(t.lookup(99999.0f, 50.0f), 155.0, 1e-3); // clamp atas x
        CHECK_NEAR(t.lookup(5000.0f, -1.0f), 50.0, 1e-3);   // clamp bawah y
        CHECK_NEAR(t.lookup(5000.0f, 999.0f), 60.0, 1e-3);  // clamp atas y
    }
    { // Kurva 1D dwell: interpolasi + clamp
        Curve1D<4> c;
        c.x[0] = 0;   c.x[1] = 100; c.x[2] = 200; c.x[3] = 300;
        c.y[0] = 0;   c.y[1] = 10;  c.y[2] = 20;  c.y[3] = 30;
        CHECK_NEAR(c.lookup(150.0f), 15.0, 1e-6);
        CHECK_NEAR(c.lookup(-5.0f), 0.0, 1e-6);
        CHECK_NEAR(c.lookup(350.0f), 30.0, 1e-6);
    }
    { // CRC32 vektor standar
        CHECK(crc32("123456789", 9) == 0xCBF43926u);
    }
    return report("fuel");
}
