// Tipe & primitif scheduler event (M4). Pure logic: tidak ada include
// Arduino/ESP32, waktu selalu eksplisit dalam mikrodetik -> unit test native.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cmath>
#include "Ztune.h"

namespace ztune {

// ---- Matematika sudut-waktu crank ----
struct AngleMath {
    // Derajat crank yang ditempuh dalam `ms` ms pada `rpm`.
    // 1 ms @ 3000 rpm = 18 derajat (cek: 3000x0.006x1).
    static float degForMs(float ms, float rpm) { return ms * rpm * (360.0f / 60000.0f); }
    // Kebalikannya: waktu (ms) untuk menempuh `deg` derajat.
    static float msForDeg(float deg, float rpm) {
        return (rpm > 0.0f) ? deg * (60000.0f / (rpm * 360.0f)) : 0.0f;
    }
    // Periode satu putaran crank dalam us.
    static float revPeriodUs(float rpm) { return (rpm > 0.0f) ? (60.0e6f / rpm) : 0.0f; }
    // Bungkus sudut ke [0, 360).
    static float wrap360(float deg) {
        float d = std::fmod(deg, 360.0f);
        return (d < 0.0f) ? d + 360.0f : d;
    }
};

// Perbandingan timestamp cyclic: aman terhadap wrap uint32 (71 menit @ 1 MHz).
// Dipakai juga untuk cek "sudah due": (int32_t)(t - now) <= 0.
inline bool tBefore(uint32_t a, uint32_t b) { return (int32_t)(a - b) < 0; }

enum class EvAction : uint8_t {
    InjOpen = 0,    // buka injektor silinder `channel`
    InjClose = 1,   // tutup injektor (akhir squirt)
    DwellStart = 2, // mulai isi coil (driver ON)
    SparkFire = 3,  // lepas coil (spark terjadi)
};

struct Ev {
    uint32_t tUs = 0;
    EvAction action = EvAction::InjOpen;
    uint8_t channel = 0;   // indeks silinder (0..cyl-1)
};

// Queue event terurut naik berdasarkan tUs, kapasitas tetap (tanpa heap/malloc).
// Push memakai insertion sort; event hanya dijadwalkan maksimal satu jendela
// gigi di depan sehingga ukurannya kecil dan bounded.
class EventQueue {
public:
    static constexpr size_t CAP = 32;

    void clear() { n_ = 0; }

    // Penuh -> tolak & hitung (jangan pernah overwrite event yang lebih awal).
    bool push(const Ev& e) {
        if (n_ >= CAP) { dropped_++; return false; }
        size_t i = n_++;
        while (i > 0 && tBefore(e.tUs, buf_[i - 1].tUs)) {
            buf_[i] = buf_[i - 1];
            --i;
        }
        buf_[i] = e;
        return true;
    }

    bool        empty() const { return n_ == 0; }
    size_t      size() const { return n_; }
    const Ev&   peek() const { return buf_[0]; }
    uint32_t    dropped() const { return dropped_; }

    void pop() {
        if (n_ == 0) return;
        n_--;
        for (size_t i = 0; i < n_; ++i) buf_[i] = buf_[i + 1];
    }

private:
    Ev buf_[CAP] = {};
    size_t n_ = 0;
    uint32_t dropped_ = 0;
};

} // namespace ztune
