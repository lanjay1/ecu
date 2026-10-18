// Tabel lookup generik (interpolasi bilinear 2D + kurva 1D) untuk VE,
// advance, dwell, dan kalibrasi sensor. Pure header-only.
#pragma once
#include <stdint.h>

namespace ztune {

namespace detail {
// Cari indeks sel: ax[i] <= v <= ax[i+1], hasil 0..n-2
inline uint8_t cellIndex(float v, const float* ax, uint8_t n) {
    if (v <= ax[0]) return 0;
    uint8_t i = 0;
    while (i < (uint8_t)(n - 2) && v > ax[i + 1]) i++;
    return i;
}
inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
} // namespace detail

template <uint8_t NX, uint8_t NY>
struct Table2D {
    static_assert(NX >= 2 && NY >= 2, "tabel minimal 2x2");
    float xAxis[NX] = {};
    float yAxis[NY] = {};
    float values[NY][NX] = {};

    float lookup(float x, float y) const {
        x = detail::clampf(x, xAxis[0], xAxis[NX - 1]);
        y = detail::clampf(y, yAxis[0], yAxis[NY - 1]);
        uint8_t ix = detail::cellIndex(x, xAxis, NX);
        uint8_t iy = detail::cellIndex(y, yAxis, NY);
        float tx = (x - xAxis[ix]) / (xAxis[ix + 1] - xAxis[ix]);
        float ty = (y - yAxis[iy]) / (yAxis[iy + 1] - yAxis[iy]);
        float a = values[iy][ix]     * (1.0f - tx) + values[iy][ix + 1]     * tx;
        float b = values[iy + 1][ix] * (1.0f - tx) + values[iy + 1][ix + 1] * tx;
        return a * (1.0f - ty) + b * ty;
    }
};

template <uint8_t N>
struct Curve1D {
    static_assert(N >= 2, "kurva minimal 2 titik");
    float x[N] = {};
    float y[N] = {};

    float lookup(float v) const {
        v = detail::clampf(v, x[0], x[N - 1]);
        uint8_t i = detail::cellIndex(v, x, N);
        float t = (v - x[i]) / (x[i + 1] - x[i]);
        return y[i] * (1.0f - t) + y[i + 1] * t;
    }
};

using VeTable16      = Table2D<16, 16>;  // keputusan user: VE 16x16
using AdvanceTable16 = Table2D<16, 16>;  // keputusan user: ignition 16x16
using DwellCurve16   = Curve1D<16>;      // dwell (ms) vs rpm
using WueCurve16     = Curve1D<16>;      // M4.1: koreksi warmup % vs CLT (C)
using IatCurve16     = Curve1D<16>;      // M4.1: koreksi densitas % vs IAT (C)
using DtimeCurve16   = Curve1D<16>;      // M4.2: dead time injektor (ms) vs batt (V)
using AfrTargetTable16 = Table2D<16, 16>; // M5: target AFR (rpm x load) utk closed loop

} // namespace ztune
