// Quickshifter (M4). Pure logic: input level sensor gear mentah (raw),
// debounce internal, keluaran fuel-cut terjadwal.
//
// Perilaku:
//  - Level sensor harus stabil >= debounceMs sebelum dianggap sah.
//  - Rising edge stabil saat rpm >= minRpm -> mulai cut selama cutMs.
//  - Selama cut, sinyal baru diabaikan; edge berikutnya butuh sensor
//    kembali lepas dulu (rising edge natural mencegah retrigger).
//  - rpm < minRpm saat edge -> sinyal diabaikan penuh (bukan cut nol ms).
#pragma once
#include <stdint.h>
#include "Ztune.h"

namespace ztune {

class Quickshifter {
public:
    void configure(bool enabled, uint16_t cutMs, uint16_t minRpm, uint16_t debounceMs) {
        enabled_ = enabled;
        cutUs_ = (uint32_t)cutMs * 1000u;
        minRpm_ = (float)minRpm;
        debounceUs_ = (uint32_t)debounceMs * 1000u;
    }

    void reset() {
        stable_ = false; candRaw_ = false; candUs_ = 0;
        inCut_ = false; cutUntilUs_ = 0; shifts_ = 0;
    }

    // raw: level mentah pin sensor gear. Panggil periodik (mis. tiap 1 ms)
    // atau tiap tooth; nowUs = timestamp mikrodetik.
    void onInput(bool raw, float rpm, uint32_t nowUs) {
        if (!enabled_) { stable_ = raw; inCut_ = false; return; }

        // Debounce: kandidat berubah -> catat waktu; stabil >= debounceUs -> sah.
        if (raw != candRaw_) { candRaw_ = raw; candUs_ = nowUs; }
        if (raw != stable_ && (uint32_t)(nowUs - candUs_) >= debounceUs_) {
            bool prev = stable_;
            stable_ = raw;
            if (!prev && stable_) {                       // rising edge stabil
                if (rpm >= minRpm_ && !inCut_) {
                    inCut_ = true;
                    cutUntilUs_ = nowUs + cutUs_;
                    shifts_++;
                }
            }
        }
    }

    // true selama jendela cut aktif; meng-expire sendiri berdasar nowUs.
    bool fuelCut(uint32_t nowUs) {
        if (inCut_ && (int32_t)(cutUntilUs_ - nowUs) <= 0) inCut_ = false;
        return inCut_;
    }

    uint32_t shifts() const { return shifts_; }

private:
    bool enabled_ = false;
    uint32_t cutUs_ = 0;
    float minRpm_ = 0.0f;
    uint32_t debounceUs_ = 0;

    bool stable_ = false;      // level sensor setelah debounce
    bool candRaw_ = false;     // kandidat level mentah
    uint32_t candUs_ = 0;      // sejak kandidat berubah terakhir
    bool inCut_ = false;
    uint32_t cutUntilUs_ = 0;
    uint32_t shifts_ = 0;
};

} // namespace ztune
