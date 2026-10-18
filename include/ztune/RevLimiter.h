// Pembatas RPM hard-cut (M4). Pure logic: rpm masuk via parameter,
// tidak ada akses hardware -> unit test native langsung.
//
// Perilaku (prioritas fitur user):
//  - rpm naik ke hardLimitRpm          -> potong bahan bakar (latch ON)
//  - rpm turun ke < limit-resumeDrop   -> lepas potongan (histeresis anti
//    on/off cepat di sekitar batas)
//  - disabled                          -> tidak pernah memotong
#pragma once
#include <stdint.h>
#include "Ztune.h"

namespace ztune {

class RevLimiter {
public:
    void configure(bool enabled, uint16_t hardLimitRpm, uint16_t resumeDropRpm) {
        enabled_ = enabled;
        limitRpm_ = (float)hardLimitRpm;
        resumeRpm_ = (float)hardLimitRpm - (float)resumeDropRpm;
    }

    void reset() { cutting_ = false; trips_ = 0; }

    // Panggil tiap tooth (rpm instan decoder) atau update periodik.
    void onRpm(float rpm) {
        if (!enabled_) { cutting_ = false; return; }
        if (!cutting_) {
            if (rpm >= limitRpm_) { cutting_ = true; trips_++; }
        } else if (rpm < resumeRpm_) {
            cutting_ = false;
        }
    }

    bool     fuelCut() const { return cutting_; }
    uint32_t trips() const { return trips_; }

private:
    bool enabled_ = false;
    float limitRpm_ = 0.0f;
    float resumeRpm_ = 0.0f;
    bool cutting_ = false;
    uint32_t trips_ = 0;
};

} // namespace ztune
