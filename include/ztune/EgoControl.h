// EgoControl - closed loop bahan bakar (M5). Pure logic: tanpa hardware.
//
// Mode sumber umpan balik (EcuConfig::Ego::o2Type):
//  - Wideband   : AFR terukur (kalibrasi linier 2 titik ADC) dibandingkan
//                 target dari tabel AFR 16x16 (rpm x load).
//  - Narrowband : hanya tahu "rich/lean" di sekitar stoich -> koreksi osilasi
//                 di titik switch tegangan (sensor standar motor injeksi).
//
// Algoritma (gaya MegaSquirt, sengaja sederhana & teruji):
//   - Integral step: tiap updateMs, jika error melewati deadband, koreksi
//     digeser +-stepPct arah penghapusan error. Tidak ada P/D -> tidak bisa
//     meledak; clamp +-limitPct mencegah drift liar saat kalibrasi salah.
//   - Gate: hanya aktif di cruise (rpm window, TPS di bawah max, mesin
//     hangat, tanpa fuel cut / ASE). Di luar gate koreksi = 0 (open loop).
//   - sensorDelayMs: setelah gate kembali OK, tunggu dulu (transportasi gas
//     buang + settling sensor, sekaligus warmup pengendali wideband saat
//     di-set besar) sebelum mulai menyesuaikan.
//
// Tanda koreksi mengikuti konvensi korektor lain (WUE/IAT/ASE): nilai %
// yang DIJUMLAHKAN ke PW. Lean (AFR terukur > target) -> butuh lebih
// banyak bahan bakar -> koreksi positif.
#pragma once
#include <stdint.h>
#include "Ztune.h"
#include "InterpTable.h"
#include "EcuConfig.h"

namespace ztune {

// Hasil state terakhir - dipakai engine task (PW) & live telemetry.
struct EgoState {
    float corrPct     = 0.0f;   // % tambahan PW (clamp +-limitPct)
    bool  active      = false;  // gate OK (masih bisa 0 koreksi)
    float targetAfr   = 0.0f;   // target dari tabel (wideband; 0 saat NB)
    float measuredAfr = 0.0f;   // AFR terukur (wideband; 0 saat NB)
};

class EgoControl {
public:
    void configure(const EgoConfig& c) { c_ = c; reset(); }

    void reset() {
        st_ = EgoState{};
        haveGate_ = false;
        tGateMs_ = 0;
        tLastAdjMs_ = 0;
    }

    // Input satu sampel - dipanggil periodik dari engine task (per tooth).
    struct Input {
        uint32_t tMs;         // waktu sekarang (ms; unsigned, aman wrap)
        float rpm;
        float load;           // load aktif (TPS % atau MAP kPa, ikut loadSource)
        float tpsPct;
        float cltC;
        float o2V;            // tegangan O2 mentah (narrowband)
        float o2Afr;          // AFR terukur (wideband)
        bool  o2Fault;        // sensor tidak valid (raw di ujung skala)
        bool  fuelCut;        // rev limiter / quickshifter aktif
        bool  aseActive;      // acceleration enrichment sedang jalan
    };

    const EgoState& update(const Input& in) {
        // ---- Gate kondisi closed loop ----
        bool gate = c_.enabled && c_.o2Type != O2Type::None && !in.o2Fault &&
                    !in.fuelCut && !in.aseActive &&
                    in.rpm >= (float)c_.minRpm && in.rpm <= (float)c_.maxRpm &&
                    in.tpsPct <= c_.maxTpsPct && in.cltC >= c_.minCltC;
        if (!gate) {
            if (haveGate_) {          // keluar gate -> koreksi dibuang
                haveGate_ = false;
                st_ = EgoState{};
            }
            return st_;
        }
        if (!haveGate_) {             // baru masuk gate: mulai masa delay
            haveGate_ = true;
            tGateMs_ = in.tMs;
            tLastAdjMs_ = in.tMs;     // jadwal penyesuaian pertama = delay+update
        }

        st_.active = true;
        if (c_.o2Type == O2Type::Wideband) {
            st_.measuredAfr = in.o2Afr;
            st_.targetAfr = c_.afrTarget.lookup(in.rpm, in.load);
        } else {
            st_.measuredAfr = 0.0f;
            st_.targetAfr = 0.0f;
        }

        // ---- Masa settle: delay transportasi gas / warmup sensor ----
        if (in.tMs - tGateMs_ < (uint32_t)c_.sensorDelayMs) return st_;
        // ---- Periode penyesuaian ----
        if (in.tMs - tLastAdjMs_ < (uint32_t)c_.updateMs) return st_;
        tLastAdjMs_ = in.tMs;

        if (c_.o2Type == O2Type::Wideband) {
            if (st_.measuredAfr > st_.targetAfr + c_.deadbandAfr)
                step(+1.0f);          // lean -> kaya-kan
            else if (st_.measuredAfr < st_.targetAfr - c_.deadbandAfr)
                step(-1.0f);          // rich -> kurangi
        } else {                      // Narrowband: switching sekitar stoich
            if (in.o2V > c_.nbSwitchV + c_.nbHystV)
                step(-1.0f);          // tegangan tinggi = rich -> pangkas
            else if (in.o2V < c_.nbSwitchV - c_.nbHystV)
                step(+1.0f);          // tegangan rendah = lean -> tambah
        }
        return st_;
    }

    const EgoState& state() const { return st_; }

private:
    void step(float dir) {
        st_.corrPct += dir * c_.stepPct;
        if (st_.corrPct > c_.limitPct)  st_.corrPct = c_.limitPct;
        if (st_.corrPct < -c_.limitPct) st_.corrPct = -c_.limitPct;
    }

    EgoConfig c_{};
    EgoState st_{};
    bool haveGate_ = false;
    uint32_t tGateMs_ = 0;
    uint32_t tLastAdjMs_ = 0;
};

} // namespace ztune
