// Test native M4: AngleMath, EventQueue, EngineScheduler, RevLimiter,
// Quickshifter. Semua kasus waktu dihitung manual @3000 rpm (pitch 1666.67us,
// rev 20000us) sehingga nilai expected bersifat analitik, bukan tautologi.
#include "ztune/SchedulerTypes.h"
#include "ztune/EngineScheduler.h"
#include "ztune/RevLimiter.h"
#include "ztune/Quickshifter.h"
#include "test_util.h"
#include <vector>

using namespace ztune;

// Helper hitung/lookup event pada vektor hasil drain queue.
static int countEv(const std::vector<Ev>& v, EvAction a, uint8_t ch) {
    int n = 0;
    for (const auto& e : v) if (e.action == a && e.channel == ch) n++;
    return n;
}
static double firstT(const std::vector<Ev>& v, EvAction a, uint8_t ch) {
    for (const auto& e : v) if (e.action == a && e.channel == ch) return (double)e.tUs;
    return -1.0;
}

// ---- Feeder gigi sintetis 12-1: 11 edge per rev, gap = 2 pitch ----
// Timestamp gigi dihitung presisi dari posisi slot kumulatif (bukan
// akumulasi float per gigi) supaya error < 1 us seperti sensor nyata.
struct Rig {
    static constexpr float RPM  = 3000.0f;
    static constexpr double SLOT = 60.0e6 / (3000.0 * 12.0);  // 1666.6667 us
    static constexpr uint32_t T0 = 1000;

    EventQueue q;
    EngineScheduler sch;
    SchedInput in{};
    std::vector<Ev> evs;
    int slots_ = 0;              // posisi pitch kumulatif sejak T0

    Rig() {
        in.synced = true;
        in.intervalUs = (float)SLOT;
        in.rpm = RPM;
        in.totalTeeth = 12.0f;
        in.missingTeeth = 1.0f;
        in.phaseOffsetDeg = 0.0f;
        in.cylinders = 4;
        in.injEndBeforeTdcDeg = 60.0f;
        in.advanceDeg = 10.0f;
        in.dwellMs = 3.0f;
        in.pwMs = 2.0f;      // 2 ms @3000 rpm = 36 derajat
        in.fuelCut = false;
    }

    void tooth(uint8_t idx) {
        in.toothIndex = idx;
        in.toothUs = T0 + (uint32_t)llround((double)slots_ * SLOT);
        sch.onTooth(in, q);
        while (!q.empty()) { evs.push_back(q.peek()); q.pop(); }
        // ke gigi berikutnya: setelah gigi 10 ada gap 2 pitch
        slots_ += (idx == 10) ? 2 : 1;
    }
    void rev() { for (uint8_t i = 0; i < 11; ++i) tooth(i); }
};

int main() {
    { // ---- AngleMath: nilai analitik ----
        CHECK_NEAR(AngleMath::degForMs(1.0f, 3000.0f), 18.0, 1e-4);
        CHECK_NEAR(AngleMath::degForMs(3.0f, 3000.0f), 54.0, 1e-4);
        CHECK_NEAR(AngleMath::msForDeg(18.0f, 3000.0f), 1.0, 1e-4);
        CHECK_NEAR(AngleMath::msForDeg(54.0f, 3000.0f), 3.0, 1e-4);
        CHECK_NEAR(AngleMath::revPeriodUs(3000.0f), 20000.0, 1e-2);
        CHECK_NEAR(AngleMath::wrap360(-10.0f), 350.0, 1e-5);
        CHECK_NEAR(AngleMath::wrap360(370.0f), 10.0, 1e-5);
        CHECK_NEAR(AngleMath::wrap360(-720.0f), 0.0, 1e-5);
    }
    { // ---- EventQueue: urutan, kapasitas, wrap cyclic ----
        EventQueue q;
        q.push({16, EvAction::SparkFire, 0});
        q.push({4,  EvAction::InjOpen, 0});
        q.push({8,  EvAction::InjOpen, 1});
        CHECK(q.size() == 3);
        CHECK(q.peek().tUs == 4);
        q.pop();
        CHECK(q.peek().tUs == 8);
        q.pop();
        CHECK(q.peek().tUs == 16);
        q.pop();
        CHECK(q.empty());

        EventQueue big;
        for (size_t i = 0; i < EventQueue::CAP; ++i)
            CHECK(big.push({(uint32_t)(1000 + i), EvAction::InjClose, 0}));
        CHECK(!big.push({5, EvAction::InjOpen, 0}));   // penuh -> tolak, bukan overwrite
        CHECK(big.dropped() == 1);
        CHECK(big.peek().tUs == 1000);                 // event terawal utuh

        // Wrap uint32: 0xFFFFFFF0 cyclic sebelum 0x00000010
        EventQueue w;
        w.push({0x00000010u, EvAction::SparkFire, 1});
        w.push({0xFFFFFFF0u, EvAction::DwellStart, 1});
        CHECK(w.peek().tUs == 0xFFFFFFF0u);
        w.pop();
        CHECK(w.peek().tUs == 0x00000010u);
    }
    { // ---- Scheduler 2 rev @3000 rpm, 4-sil, PW 2ms, dwell 3ms, adv 10 ----
        // Ekspektasi per rev (tdc mod 360: c0=0, c1=180, c2=0, c3=180):
        //   InjOpen  : c1/c3 @84 , c0/c2 @264
        //   InjClose : c1/c3 @120, c0/c2 @300
        //   Dwell    : c1/c3 @116, c0/c2 @296
        //   Spark    : c1/c3 @170, c0/c2 @350   (jendela gigi terakhir 2 pitch)
        Rig r;
        r.rev(); r.rev();
        CHECK(r.evs.size() == 32);                     // 16 event/rev x 2
        for (uint8_t c = 0; c < 4; ++c) {
            CHECK(countEv(r.evs, EvAction::InjOpen,  c) == 2);
            CHECK(countEv(r.evs, EvAction::InjClose, c) == 2);
            CHECK(countEv(r.evs, EvAction::DwellStart, c) == 2);
            CHECK(countEv(r.evs, EvAction::SparkFire,  c) == 2);
        }
        // Waktu analitik: t_ev = T0 + sudut/360 x 20000
        CHECK_NEAR(firstT(r.evs, EvAction::InjOpen, 1), 1000.0 + 84.0 / 360.0 * 20000.0, 2.0);
        CHECK_NEAR(firstT(r.evs, EvAction::InjClose, 1), 1000.0 + 120.0 / 360.0 * 20000.0, 2.0);
        CHECK_NEAR(firstT(r.evs, EvAction::DwellStart, 1), 1000.0 + 116.0 / 360.0 * 20000.0, 2.0);
        CHECK_NEAR(firstT(r.evs, EvAction::SparkFire, 1), 1000.0 + 170.0 / 360.0 * 20000.0, 2.0);
        CHECK_NEAR(firstT(r.evs, EvAction::InjOpen, 0), 1000.0 + 264.0 / 360.0 * 20000.0, 2.0);
        CHECK_NEAR(firstT(r.evs, EvAction::DwellStart, 0), 1000.0 + 296.0 / 360.0 * 20000.0, 2.0);
        // Spark @350: lewat gap (2 pitch) -> bukti jendela gigi terakhir benar
        CHECK_NEAR(firstT(r.evs, EvAction::SparkFire, 0), 1000.0 + 350.0 / 360.0 * 20000.0, 2.0);
        // Tidak ada event terjadwal ganda di sudut sama+channel sama
        for (size_t i = 1; i < r.evs.size(); ++i)
            for (size_t j = 0; j < i; ++j)
                if (r.evs[i].action == r.evs[j].action && r.evs[i].channel == r.evs[j].channel)
                    CHECK((int32_t)(r.evs[i].tUs - r.evs[j].tUs) != 0);
    }
    { // ---- Guard scheduler ----
        Rig r;
        r.in.synced = false; r.rev();
        CHECK(r.evs.empty());                          // tidak sync -> tidak jadwal

        Rig r2; r2.in.rpm = 0.0f; r2.rev();
        CHECK(r2.evs.empty());                         // engkol diam -> tidak jadwal
    }
    { // ---- phaseOffset menggeser semua event ----
        Rig r; r.in.phaseOffsetDeg = 30.0f; r.rev();
        // Spark c0: tdc=30 -> spark=20 deg (dari 350 geser +30)
        CHECK_NEAR(firstT(r.evs, EvAction::SparkFire, 0), 1000.0 + 20.0 / 360.0 * 20000.0, 2.0);
    }
    { // ---- fuelCut (rev limiter/QS): injeksi dibuang, spark tetap ----
        Rig r; r.in.fuelCut = true; r.rev(); r.rev();
        CHECK(r.evs.size() == 16);
        int inj = 0, ign = 0;
        for (const auto& e : r.evs)
            (e.action == EvAction::InjOpen || e.action == EvAction::InjClose) ? inj++ : ign++;
        CHECK(inj == 0);
        CHECK(ign == 16);
    }
    { // ---- RevLimiter: latch + histeresis ----
        RevLimiter L;
        L.configure(true, 11000, 300);                 // resume di 10700
        L.onRpm(10500); CHECK(!L.fuelCut());
        L.onRpm(10999); CHECK(!L.fuelCut());
        L.onRpm(11000); CHECK(L.fuelCut());            // tepat batas -> cut
        L.onRpm(10900); CHECK(L.fuelCut());            // belum turun ke resume
        L.onRpm(10701); CHECK(L.fuelCut());
        L.onRpm(10699); CHECK(!L.fuelCut());           // lepas
        CHECK(L.trips() == 1);
        L.onRpm(11000); CHECK(L.fuelCut());            // latch lagi
        CHECK(L.trips() == 2);

        RevLimiter D; D.configure(false, 11000, 300);
        D.onRpm(16000); CHECK(!D.fuelCut());           // disabled -> tak pernah cut
    }
    { // ---- Quickshifter: debounce, cut window, retrigger (skala us!) ----
        Quickshifter Q;
        Q.configure(true, 80 /*ms*/, 4000, 30 /*ms = 30000 us*/);
        Q.reset();
        Q.onInput(true, 6000, 1000000);                // edge mentah -> kandidat
        CHECK(!Q.fuelCut(1000010));                    // 10 us: belum stabil
        Q.onInput(true, 6000, 1029999);                // 29999 us: masih kurang
        CHECK(!Q.fuelCut(1029999));
        Q.onInput(true, 6000, 1030000);                // 30000 us: sah -> cut
        CHECK(Q.fuelCut(1030000));                     // cutUntil = 1110000
        CHECK(Q.fuelCut(1109999));
        CHECK(!Q.fuelCut(1110000));                    // expire tepat 80 ms

        // Masih ditekan -> tidak retrigger (butuh release dulu)
        Q.onInput(true, 6000, 1200000);
        CHECK(!Q.fuelCut(1200001));

        // Lepas (debounce), tekan lagi -> cut kedua
        Q.onInput(false, 6000, 1300000);
        Q.onInput(false, 6000, 1330000);               // stabil false
        Q.onInput(true, 6000, 1400000);                // kandidat baru
        CHECK(!Q.fuelCut(1400001));                    // belum stabil
        Q.onInput(true, 6000, 1430000);                // stabil -> cut kedua
        CHECK(Q.fuelCut(1430000));
        CHECK(Q.shifts() == 2);

        // rpm di bawah minRpm: edge diabaikan penuh
        Quickshifter R; R.configure(true, 80, 4000, 30); R.reset();
        R.onInput(true, 3000, 1000);
        R.onInput(true, 3000, 31000);                  // stabil, tapi rpm rendah
        CHECK(!R.fuelCut(32000));
        R.onInput(false, 3000, 33000);                 // harus lepas dulu
        R.onInput(false, 3000, 63000);                 // stabil false
        R.onInput(true, 6000, 70000);                  // edge sah rpm tinggi
        R.onInput(true, 6000, 100000);                 // stabil -> cut
        CHECK(R.fuelCut(100000));

        // Chattering sensor: toggle tiap 5 ms tak pernah sah (debounce 30 ms)
        Quickshifter C; C.configure(true, 80, 4000, 30); C.reset();
        for (uint32_t t = 0; t < 1000000; t += 5000)
            C.onInput((t / 5000) % 2 == 0, 6000, t);
        CHECK(!C.fuelCut(1000001));
        CHECK(C.shifts() == 0);

        // Disabled: tak pernah cut
        Quickshifter N; N.configure(false, 80, 4000, 30); N.reset();
        N.onInput(true, 6000, 0);
        N.onInput(true, 6000, 100000);
        CHECK(!N.fuelCut(100001));
    }
    return report("scheduler");
}
