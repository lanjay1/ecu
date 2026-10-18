// Simulator sinyal gigi untuk test native: membangkitkan timestamp edge
// roda gigi missing-tooth pada profil RPM terhadap waktu (integrasi sudut).
// Dipakai test_trigger untuk skenario: sync, akselerasi, jitter, drop-tooth.
#pragma once
#include <cstdint>
#include <cmath>
#include <functional>
#include "ztune/TriggerConfig.h"

namespace ztune_sim {

// PRNG deterministik (xorshift32) agar test bisa direproduksi.
inline uint32_t xs32(uint32_t& s) {
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    return s;
}

class ToothSim {
public:
    explicit ToothSim(const ztune::TriggerConfig& cfg) : cfg_(cfg) {}

    // Bangkitkan edge dari tUsStart..tUsEnd (mikrodetik, mulai sudut 0).
    // rpmFn: rpm instan vs detik. onEdge dipanggil per edge gigi (timestamp).
    // jitterPct: derau +/- pada INTERVAL (bukan waktu absolut).
    // dropEdgeIndex: nomor edge global (0-based) yang dibuang (VR kotor).
    void run(uint32_t tUsStart, uint32_t tUsEnd, float (*rpmFn)(float),
             const std::function<void(uint32_t)>& onEdge,
             float jitterPct = 0.0f, uint32_t seed = 0x9E3779B9u,
             long dropEdgeIndex = -1) {
        const float pitch = 360.0f / (float)cfg_.totalTeeth;
        const float dtUs = 5.0f;
        float angle = 0.0f;
        uint32_t prevT = tUsStart;
        uint32_t lastEdgeT = tUsStart;
        uint32_t rng = seed | 1u;
        long edgeIdx = 0;

        for (uint32_t t = tUsStart + (uint32_t)dtUs; t <= tUsEnd; t += (uint32_t)dtUs) {
            float rp = rpmFn((float)(t - (uint32_t)(dtUs * 0.5f)) * 1e-6f);
            if (rp > 1.0f) {
                float dA = (rp / 60.0f) * 360.0f * (dtUs * 1e-6f);
                float newA = angle + dA;
                int k0 = (int)std::floor(angle / pitch);
                int k1 = (int)std::floor(newA / pitch);
                for (int k = k0 + 1; k <= k1; ++k) {
                    int pos = k % (int)cfg_.totalTeeth;
                    if (pos < 0) pos += (int)cfg_.totalTeeth;
                    if (pos < (int)cfg_.missingTeeth) continue;  // gigi hilang -> tanpa edge
                    // Interpolasi linier waktu penyeberangan sudut (sub-us)
                    float frac = ((float)k * pitch - angle) / dA;
                    uint32_t tE = (uint32_t)((float)prevT + (float)(t - prevT) * frac);
                    if (jitterPct > 0.0f) {
                        float j = ((float)(xs32(rng) % 2001u) - 1000.0f) / 1000.0f
                                  * (jitterPct / 100.0f);
                        tE = lastEdgeT + (uint32_t)((float)(tE - lastEdgeT) * (1.0f + j));
                    }
                    lastEdgeT = tE;
                    if (dropEdgeIndex >= 0 && edgeIdx == dropEdgeIndex) { edgeIdx++; continue; }
                    edgeIdx++;
                    onEdge(tE);
                }
                angle = newA;
            }
            prevT = t;
        }
    }

private:
    const ztune::TriggerConfig& cfg_;
};

} // namespace ztune_sim
