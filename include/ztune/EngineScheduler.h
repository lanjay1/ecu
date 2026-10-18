// Scheduler injeksi & penyalaan berbasis gigi trigger (M4). Pure logic.
//
// Prinsip: setiap tooth edge membuka "jendela sudut" (theta, theta + lebar]
// menuju gigi berikutnya. Semua sudut event (buka/tutup injektor, mulai dwell,
// spark) yang jatuh di jendela itu dijadwalkan dengan prediksi linear:
//   t_event = t_tooth + intervalPitchUs x (sudut_ke_event / sudut_per_pitch)
// intervalPitchUs berasal dari EMA interval gigi decoder (disediakan caller).
//
// Konvensi crank-only (opsi A, keputusan user):
//  - Satu rev crank = satu set event identik (wasted spark + injeksi batch,
//    squirtsPerCycle=2). Fase siklus 4-tak tidak diketahui dan tidak perlu.
//  - Sudut TDC kompresi silinder k = phaseOffsetDeg + k x (720/cyl), mod 360.
//    Untuk 4-silinder: k=0 -> 0, k=1 -> 180, k=2 -> 0, k=3 -> 180 (mod 360);
//    dua silinder bertumpuk di sudut sama = pasangan wasted/batch yang benar.
//  - Event tepat di sudut gigi (delta ~ 0) TIDAK dijadwalkan lagi di sini:
//    gigi sebelumnya sudah menjadwalkannya (jendela setengah-terbuka) ->
//    tidak ada duplikat, tidak ada event terlewat.
#pragma once
#include <stdint.h>
#include "Ztune.h"
#include "SchedulerTypes.h"

namespace ztune {

// Semua nilai dihitung caller (decoder, tabel, FuelMath, limiter) sehingga
// kelas ini bebas dari EcuConfig dan tabel -> mudah di-test secara isolat.
struct SchedInput {
    bool     synced = false;
    uint8_t  toothIndex = 0;        // 0 = gigi pertama setelah gap
    uint32_t toothUs = 0;           // timestamp edge gigi ini
    float    intervalUs = 0.0f;     // estimasi interval antar pitch gigi (EMA)
    float    rpm = 0.0f;

    // Parameter mesin (dari EcuConfig):
    float    totalTeeth = 12.0f;
    float    missingTeeth = 1.0f;
    float    phaseOffsetDeg = 0.0f; // TDC kompresi silinder 0 vs gigi-0
    uint8_t  cylinders = 1;
    float    injEndBeforeTdcDeg = 60.0f;
    float    advanceDeg = 10.0f;    // BTDC, hasil lookup tabel advance
    float    dwellMs = 3.0f;        // hasil lookup dwell curve
    float    pwMs = 0.0f;           // PW termasuk dead time (FuelMath)
    bool     fuelCut = false;       // rev limiter / quickshifter aktif
};

class EngineScheduler {
public:
    void onTooth(const SchedInput& in, EventQueue& q) {
        if (!in.synced || in.rpm < MIN_SCHED_RPM || in.intervalUs <= 0.0f) return;

        const float dpt = 360.0f / in.totalTeeth;
        const uint8_t teethPerRev = (uint8_t)(in.totalTeeth - in.missingTeeth);
        // Jendela gigi terakhir memuat slot yang hilang: 2 pitch utk 12-1,
        // (missing+1) pitch secara umum.
        const float winDeg = (in.toothIndex == (uint8_t)(teethPerRev - 1))
                             ? (in.missingTeeth + 1.0f) * dpt : dpt;
        const float theta = AngleMath::wrap360((float)in.toothIndex * dpt);

        const float cylStep = 720.0f / (float)in.cylinders;
        const float pwDeg = AngleMath::degForMs(in.pwMs, in.rpm);
        const float dwDeg = AngleMath::degForMs(in.dwellMs, in.rpm);

        for (uint8_t k = 0; k < in.cylinders; ++k) {
            const float tdc      = AngleMath::wrap360(in.phaseOffsetDeg + (float)k * cylStep);
            const float sparkDeg = AngleMath::wrap360(tdc - in.advanceDeg);
            const float dwellDeg = AngleMath::wrap360(sparkDeg - dwDeg);
            const float injClose = AngleMath::wrap360(tdc - in.injEndBeforeTdcDeg);
            const float injOpen  = AngleMath::wrap360(injClose - pwDeg);

            trySched(q, in, theta, winDeg, dwellDeg, EvAction::DwellStart, k);
            trySched(q, in, theta, winDeg, sparkDeg, EvAction::SparkFire, k);
            if (!in.fuelCut) {   // hard cut: hanya potong bahan bakar, coil tetap
                trySched(q, in, theta, winDeg, injOpen,  EvAction::InjOpen,  k);
                trySched(q, in, theta, winDeg, injClose, EvAction::InjClose, k);
            }
        }
    }

    uint32_t scheduled() const { return sched_; }
    void resetStats() { sched_ = 0; }

private:
    static constexpr float MIN_SCHED_RPM = 30.0f; // di bawah ini: engkol diam
    static constexpr float EPS = 0.05f;           // toleransi float (derajat)

    void trySched(EventQueue& q, const SchedInput& in, float theta, float winDeg,
                  float evDeg, EvAction a, uint8_t ch) {
        float d = AngleMath::wrap360(evDeg - theta);
        if (d <= EPS || d > winDeg + EPS) return;  // jendela setengah-terbuka
        // Prediksi waktu: kecepatan sudut uniform -> waktu per PITCH tetap
        // intervalUs (meski jendela gigi terakhir selebar 2 pitch saat gap).
        float dpt = 360.0f / in.totalTeeth;
        float t = (float)in.toothUs + in.intervalUs * (d / dpt);
        if (q.push(Ev{(uint32_t)t, a, ch})) sched_++;
    }

    uint32_t sched_ = 0;
};

} // namespace ztune
