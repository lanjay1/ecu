// Matematika bahan bakar gaya MegaSquirt (keputusan user):
//   PW = reqFuel x VE%  (runtime, dikali koreksi pada milestone lanjutan)
// Referensi analisis LibreTune: rumus reqFuel-nya MegaSquirt-style, namun
// LibreTune mengandung indikasi bug skala (41,76 ms di-clamp 25,5 ms).
// ZtuneECU memakai konvensi klasik MegaSquirt yang sudah diverifikasi:
//   REQ_FUEL (ms) = DISP_per_cyl(L) x STOICH x 500 / INJ(cc/min)
// Contoh cek: 4-sil 2000cc (500cc/cyl), inj 440 cc/min, stoich 14.7
//   -> 0.5 x 14.7 x 500 / 440 = 8.352 ms
#pragma once
#include <stdint.h>

namespace ztune {

struct FuelMath {
    // Waktu injeksi dasar per event bila VE = 100%.
    // 500 = konstanta konversi satuan konvensi MegaSquirt (mengandaikan
    // kerapatan udara standar); deviasi kondisi nyata diserap tabel VE.
    static float reqFuelMs(float dispPerCylCc, float stoichAfr, float injCcPerMin) {
        if (dispPerCylCc <= 0.0f || injCcPerMin <= 0.0f) return 0.0f;
        return (dispPerCylCc * 0.001f * stoichAfr * 500.0f) / injCcPerMin;
    }

    // PW aktual per squirt: reqFuel x VE% dibagi jumlah squirt per siklus,
    // plus dead time injektor (buka-tutup tanpa aliran).
    // crank-only (opsi A): squirtsPerCycle = 2 -> tiap squirt separuh muatan,
    // total per siklus tetap reqFuel x VE%.
    static float pulseWidthMs(float reqFuelMs, float vePercent,
                              uint8_t squirtsPerCycle, float deadTimeMs) {
        if (squirtsPerCycle == 0) return 0.0f;
        float pw = reqFuelMs * (vePercent / 100.0f) / (float)squirtsPerCycle;
        if (pw < 0.0f) pw = 0.0f;
        return pw + deadTimeMs;
    }

    // Koreksi gaya MegaSquirt (M4.1, ego ditambah M5): korektor dijumlahkan
    // sebagai prosentase (WUE warmup + IAT densitas + ASE tip-in + EGO closed
    // loop), dibatasi clamp keselamatan 0.2x..3.0x agar kesalahan kalibrasi
    // tidak meledakkan PW.
    // wuePct/iatPct/asePct/egoPct = 0 berarti tanpa koreksi.
    static float correctedPwMs(float pwBase, float wuePct, float iatPct,
                               float asePct, float egoPct = 0.0f) {
        float mult = 1.0f + (wuePct + iatPct + asePct + egoPct) / 100.0f;
        if (mult < 0.2f) mult = 0.2f;
        if (mult > 3.0f) mult = 3.0f;
        return pwBase * mult;
    }
};

} // namespace ztune
