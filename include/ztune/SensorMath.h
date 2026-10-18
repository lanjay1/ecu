// Matematika sensor input (M4.1). Pure logic: raw ADC masuk, nilai fisik
// keluar - semua kalibrasi via parameter (EcuConfig), tanpa hardware.
//
// Konvensi wiring:
//  - ADC 12-bit, full-scale 3.3 V (konstanta elektronik board, bukan param mesin)
//  - TPS & MAP: kalibrasi linier 2 titik dalam unit ADC raw (menyerap
//    non-linearitas ADC ESP32 pada kisaran kerja sensor)
//  - CLT/IAT: NTC dengan persamaan beta: pull-up ke 3.3V, sensor ke GND
//      raw/ADC_MAX = Rs/(Rp+Rs)  ->  Rs = Rp x raw/(ADC_MAX-raw)
//      1/T = 1/T0 + ln(R/R0)/beta   (T dalam Kelvin)
//  - Batt: pembagi resistif, ratio = (Rtop+Rbot)/Rbot
//  - O2 (M5): tegangan langsung (narrowband 0..1 V) atau output pengendali
//    wideband via pembagi -> kalibrasi linier 2 titik ADC -> AFR
#pragma once
#include <stdint.h>
#include <cmath>

namespace ztune {

struct SensorMath {
    static constexpr float ADC_MAX  = 4095.0f;   // 12-bit
    static constexpr float ADC_FS_V = 3.3f;

    // ---- TPS: 0..100 % dari kalibrasi 2 titik (closed < wot) ----
    static float tpsPct(int raw, int closedAdc, int wotAdc) {
        if (wotAdc <= closedAdc) return 0.0f;    // kalibrasi salah -> inert
        float pct = ((float)raw - (float)closedAdc) * 100.0f /
                    (float)(wotAdc - closedAdc);
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 100.0f) pct = 100.0f;
        return pct;
    }

    // ---- MAP: interpolasi linier 2 titik ADC -> kPa, clamp ke rentang ----
    static float mapKpa(int raw, int zeroAdc, int fullAdc,
                        float kpaZero, float kpaFull) {
        if (fullAdc == zeroAdc) return kpaZero;
        float k = kpaZero + ((float)raw - (float)zeroAdc) *
                  (kpaFull - kpaZero) / (float)(fullAdc - zeroAdc);
        float lo = (kpaZero < kpaFull) ? kpaZero : kpaFull;
        float hi = (kpaZero < kpaFull) ? kpaFull : kpaZero;
        if (k < lo) k = lo;
        if (k > hi) k = hi;
        return k;
    }

    // ---- NTC (CLT/IAT): persamaan beta. Return -100 C saat open/short ----
    static float ntcC(int raw, float pullupOhm, float betaK,
                      float r0Ohm, float t0C) {
        if (raw <= 2 || (float)raw >= ADC_MAX - 2.0f)
            return -100.0f;                      // wiring fault: open/short
        float r = pullupOhm * (float)raw / (ADC_MAX - (float)raw);
        float t0K = t0C + 273.15f;
        float invT = 1.0f / t0K + std::log(r / r0Ohm) / betaK;
        return 1.0f / invT - 273.15f;
    }

    // ---- Tegangan baterai via pembagi ----
    static float battV(int raw, float divRatio) {
        return (float)raw / ADC_MAX * ADC_FS_V * divRatio;
    }

    // ---- O2 (M5) ----
    // Tegangan sensor O2 (narrowband switch 0..1 V; wideband 0..5 V setelah
    // pembagi). Murni konversi full-scale 3.3 V.
    static float o2Volts(int raw) {
        return (float)raw / ADC_MAX * ADC_FS_V;
    }
    // Wideband: AFR dari kalibrasi linier 2 titik ADC -> AFR (menyerap
    // pembagi + karakteristik pengendali). Clamp ke rentang kalibrasi;
    // kalibrasi salah (adcHigh == adcLow) -> inert di afrLow.
    static float o2Afr(int raw, int adcLow, float afrLow, int adcHigh, float afrHigh) {
        if (adcHigh <= adcLow) return afrLow;
        float afr = afrLow + ((float)raw - (float)adcLow) *
                    (afrHigh - afrLow) / (float)(adcHigh - adcLow);
        float lo = (afrLow < afrHigh) ? afrLow : afrHigh;
        float hi = (afrLow < afrHigh) ? afrHigh : afrLow;
        if (afr < lo) afr = lo;
        if (afr > hi) afr = hi;
        return afr;
    }
};

// Filter EMA sederhana (konstanta algoritma alpha, bukan param mesin).
class EmaFilter {
public:
    explicit EmaFilter(float alpha = 0.25f) : alpha_(alpha) {}
    float update(float v) {
        if (n_ == 0) out_ = v;
        else       out_ += alpha_ * (v - out_);
        n_++;
        return out_;
    }
    float    value() const { return out_; }
    bool     valid() const { return n_ > 0; }
    uint32_t samples() const { return n_; }
    void reset() { out_ = 0.0f; n_ = 0; }

private:
    float alpha_;
    float out_ = 0.0f;
    uint32_t n_ = 0;
};

// Snapshot nilai sensor ter-smooth - dibaca engine task & live telemetry.
struct SensorSnapshot {
    float tpsPct = 0.0f;
    float mapKpa = 0.0f;
    float cltC   = 0.0f;
    float iatC   = 0.0f;
    float battV  = 0.0f;
    float o2V    = 0.0f;   // M5: tegangan O2 (NB & debug)
    float o2Afr  = 0.0f;   // M5: AFR terukur (wideband; 0 saat NB)
    bool  cltFault = false;   // NTC open/short
    bool  iatFault = false;
    bool  o2Fault  = false;   // M5: raw di ujung skala / sensor tak terpasang
};

} // namespace ztune
