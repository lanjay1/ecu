// Mini test framework assert-based (tanpa dependensi) untuk test native.
#pragma once
#include <cstdio>
#include <cmath>

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

#define CHECK_NEAR(a, b, tol) do { \
    double _a = (double)(a), _b = (double)(b), _t = (double)(tol); \
    if (std::fabs(_a - _b) <= _t) { g_pass++; } \
    else { g_fail++; std::printf("  FAIL %s:%d: %.6f vs %.6f (tol %.6f)\n", \
        __FILE__, __LINE__, _a, _b, _t); } \
} while (0)

static int report(const char* name) {
    std::printf("[%s] pass=%d fail=%d -> %s\n", name, g_pass, g_fail,
                g_fail == 0 ? "OK" : "GAGAL");
    return g_fail == 0 ? 0 : 1;
}
