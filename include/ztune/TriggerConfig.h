// Konfigurasi trigger roda gigi missing-tooth (crank-only via conditioner MAX9926).
// ATURAN: tidak ada nilai mesin yang dibekukan di firmware - semuanya lewat
// struct config ini (persist di NVS, diubah via aplikasi tuning).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstdio>

namespace ztune {

struct TriggerConfig {
    uint8_t  totalTeeth;     // jumlah posisi gigi per putaran crank (mis. 12)
    uint8_t  missingTeeth;   // jumlah gigi hilang berurutan (mis. 1)
    float    gapRatio;       // rasio deteksi gap: interval > gapRatio x EMA normal
    float    rpmMax;         // RPM atas valid (di luar ini dianggap glitch)
    float    rpmMin;         // RPM bawah acuan (cranking)
    uint32_t syncTimeoutUs;  // tanpa edge selama ini -> Lost (stall)
    // Kalibrasi fase: sudut crank TDC kompresi silinder 0 diukur dari gigi-0
    // (gigi pertama setelah gap, arah putar positif). Derajat 0..360.
    // Dikalibrasi via aplikasi tuning (timing light) - TIDAK di-hardcode.
    float    phaseOffsetDeg;

    bool validate(char* err, size_t errLen) const {
        auto fail = [&](const char* m) {
            if (err) snprintf(err, errLen, "trigger: %s", m);
            return false;
        };
        if (totalTeeth < 4 || totalTeeth > 60) return fail("totalTeeth 4..60");
        if (missingTeeth < 1 || missingTeeth > 4) return fail("missingTeeth 1..4");
        if (totalTeeth - missingTeeth < 4) return fail("gigi efektif < 4");
        if (gapRatio < 1.05f || gapRatio > 5.0f) return fail("gapRatio 1.05..5.0");
        if (rpmMin < 10.0f || rpmMax > 30000.0f || rpmMax < rpmMin * 2.0f)
            return fail("rentang rpm tidak valid");
        if (syncTimeoutUs < 10000 || syncTimeoutUs > 1000000)
            return fail("syncTimeoutUs 10ms..1s");
        if (phaseOffsetDeg < 0.0f || phaseOffsetDeg >= 360.0f)
            return fail("phaseOffsetDeg 0..360");
        return true;
    }

    static constexpr TriggerConfig defaults() {
        return TriggerConfig{12, 1, 1.5f, 16000.0f, 40.0f, 200000u, 0.0f};
    }
};

} // namespace ztune
