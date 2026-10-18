// Acceleration Enrichment (ASE) tip-in (M4.1). Pure logic.
//
// Deteksi tip-in: laju TPS (pct/s) melewati ambang -> mulai window ASE.
// Nilai enrich ramp-down linear dari asePct ke 0 selama durationMs:
//   enrich(t) = asePct x (1 - t/duration)
// Tidak ada retrigger selama window aktif; tip-in berikutnya setelah expire.
#pragma once
#include <stdint.h>
#include "Ztune.h"

namespace ztune {

class AccelEnrichment {
public:
    void configure(float tpsRateThreshPctS, float asePct, uint32_t durationMs) {
        thresh_ = tpsRateThreshPctS;
        asePct_ = asePct;
        durationUs_ = (uint32_t)durationMs * 1000u;
    }

    void reset() {
        havePrev_ = false; prevTps_ = 0.0f; prevUs_ = 0;
        active_ = false; startUs_ = 0; trips_ = 0;
    }

    // Panggil tiap sampel TPS (per tooth di engine task).
    void onTps(float tpsPct, uint32_t nowUs) {
        if (!havePrev_) { prevTps_ = tpsPct; prevUs_ = nowUs; havePrev_ = true; return; }
        uint32_t dt = nowUs - prevUs_;              // unsigned: aman wrap
        if (dt < 1000) return;                      // < 1 ms: noise, skip
        float rate = (tpsPct - prevTps_) * 1.0e6f / (float)dt;   // %/s
        prevTps_ = tpsPct;
        prevUs_ = nowUs;
        if (!active_ && rate >= thresh_) {
            active_ = true;
            startUs_ = nowUs;
            trips_++;
        }
    }

    // 0..asePct, ramp-down linear; expire sendiri berdasar nowUs.
    float enrichPct(uint32_t nowUs) {
        if (!active_) return 0.0f;
        uint32_t el = nowUs - startUs_;
        if (el >= durationUs_) { active_ = false; return 0.0f; }
        return asePct_ * (1.0f - (float)el / (float)durationUs_);
    }

    bool     active() const { return active_; }
    uint32_t trips() const { return trips_; }

private:
    float thresh_ = 0.0f;
    float asePct_ = 0.0f;
    uint32_t durationUs_ = 0;

    bool havePrev_ = false;
    float prevTps_ = 0.0f;
    uint32_t prevUs_ = 0;
    bool active_ = false;
    uint32_t startUs_ = 0;
    uint32_t trips_ = 0;
};

} // namespace ztune
