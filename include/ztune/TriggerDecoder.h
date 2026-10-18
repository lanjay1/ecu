// Decoder trigger missing-tooth crank-only. State machine: Hunting -> Synced -> Lost.
// Murni logika + stdint.h (filosofi TritonECU): unit test native tanpa hardware.
//
// Cara pakai: panggil onTooth(nowUs) pada setiap rising edge CKP (hasil
// kondisioner MAX9926 sudah digital), dan update(nowUs) secara periodik
// untuk deteksi stall. Referensi fase = gigi ke-0 tepat setelah gap.
//
// CATATAN DESAIN (crank-only / tanpa cam):
//  - Fase siklus 4-tak (rev 1 vs rev 2) tidak diketahui -> penyalaan wasted
//    spark dan injeksi dibagi 2 squirt per siklus (aturan di EcuConfig).
//  - Keterbatasan inheren missing-tooth: kalau edge gigi yang TEPAT setelah
//    gap hilang (VR kotor), dekoder bisa alias +1 gigi (30 derajat utk 12-1).
//    Mitigasi lengkap butuh cam sensor (opsi B, menyusul) atau plausibility
//    check berbasis konteks. Test drop-tooth di test_trigger memakai gigi
//    di tengah pola (bukan tepat sambungan gap).
#pragma once
#include <stdint.h>
#include "Ztune.h"
#include "TriggerConfig.h"

namespace ztune {

class TriggerDecoder {
public:
    explicit TriggerDecoder(const TriggerConfig& cfg) : cfg_(cfg) { reset(); }

    void reset() {
        state_ = TriggerState::Hunting;
        haveLast_ = false;
        lastToothUs_ = 0;
        intervalEmaUs_ = 0.0f;
        toothIndex_ = 0;
        edgesSinceGap_ = 0;
        rpm_ = 0.0f;
        syncCount_ = 0;
        lostCount_ = 0;
    }

    void setConfig(const TriggerConfig& cfg) { cfg_ = cfg; reset(); }

    // Dipanggil pada setiap rising edge CKP. nowUs = timestamp mikrodetik.
    void onTooth(uint32_t nowUs) {
        if (!haveLast_) {
            lastToothUs_ = nowUs;
            haveLast_ = true;
            edgesSinceGap_ = 1;
            return;
        }
        uint32_t delta = nowUs - lastToothUs_;   // aritmetika unsigned: aman wrap uint32
        if (delta == 0) return;                  // edge ganda / glitch
        lastToothUs_ = nowUs;

        const uint8_t teethPerRev = (uint8_t)(cfg_.totalTeeth - cfg_.missingTeeth);
        const float fDelta = (float)delta;

        // Deteksi gap: interval > gapRatio x EMA interval gigi normal.
        bool isGap = (intervalEmaUs_ > 0.0f) && (fDelta > cfg_.gapRatio * intervalEmaUs_);

        if (isGap) {
            // Interval gap mencakup (missing+1) pitch -> koreksi perhitungan RPM.
            float rpmGap = 60.0e6f * (float)(cfg_.missingTeeth + 1) /
                           (fDelta * (float)cfg_.totalTeeth);
            if (rpmValid(rpmGap)) rpm_ = rpmGap;

            // Verifikasi pola tiap gap: jumlah gigi sejak gap sebelumnya
            // harus persis teethPerRev. Inilah resync otomatis tiap putaran.
            if (edgesSinceGap_ == teethPerRev) {
                if (state_ != TriggerState::Synced) syncCount_++;
                state_ = TriggerState::Synced;
            } else {
                if (state_ == TriggerState::Synced) lostCount_++;
                state_ = TriggerState::Hunting;
            }
            toothIndex_ = 0;         // edge ini = gigi ke-0 (referensi fase)
            edgesSinceGap_ = 1;
            return;                  // EMA tidak dipengaruhi interval gap
        }

        // Interval normal.
        float rpmNow = 60.0e6f / (fDelta * (float)cfg_.totalTeeth);
        if (!rpmValid(rpmNow)) return;   // glitch di luar rentang: abaikan, jangan polusi EMA
        intervalEmaUs_ = (intervalEmaUs_ <= 0.0f)
                         ? fDelta
                         : intervalEmaUs_ + EMA_ALPHA * (fDelta - intervalEmaUs_);
        rpm_ = rpmNow;

        if (edgesSinceGap_ < 0xFFFF) edgesSinceGap_++;
        if (state_ == TriggerState::Synced) {
            toothIndex_ = (uint8_t)(edgesSinceGap_ - 1);
        }
    }

    // Panggil periodik dari loop utama: deteksi stall via timeout.
    void update(uint32_t nowUs) {
        if (state_ == TriggerState::Synced && haveLast_ &&
            (uint32_t)(nowUs - lastToothUs_) > cfg_.syncTimeoutUs) {
            state_ = TriggerState::Lost;
            lostCount_++;
            rpm_ = 0.0f;
            haveLast_ = false;       // mulai hunt segar saat mesin hidup lagi
            edgesSinceGap_ = 0;
        }
    }

    TriggerState state() const { return state_; }
    bool isSynced() const { return state_ == TriggerState::Synced; }
    float rpm() const { return rpm_; }             // instan per-interval (mentah)
    uint8_t toothIndex() const { return toothIndex_; }  // 0..teethPerRev-1 saat Synced
    float degPerTooth() const { return 360.0f / (float)cfg_.totalTeeth; }
    // Estimasi interval antar pitch gigi (EMA) - input prediksi sudut scheduler.
    float intervalUs() const { return intervalEmaUs_; }
    uint32_t lastToothUs() const { return lastToothUs_; }
    uint32_t syncCount() const { return syncCount_; }
    uint32_t lostCount() const { return lostCount_; }

private:
    static constexpr float EMA_ALPHA = 0.30f;   // konstanta algoritma filter (bukan param mesin)

    bool rpmValid(float r) const { return r <= cfg_.rpmMax; }

    TriggerConfig cfg_{};
    TriggerState state_ = TriggerState::Hunting;
    bool haveLast_ = false;
    uint32_t lastToothUs_ = 0;
    float intervalEmaUs_ = 0.0f;
    uint8_t toothIndex_ = 0;
    uint16_t edgesSinceGap_ = 0;
    float rpm_ = 0.0f;
    uint32_t syncCount_ = 0;
    uint32_t lostCount_ = 0;
};

} // namespace ztune
