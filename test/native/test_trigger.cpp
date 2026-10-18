// Test native TriggerDecoder: sync, akurasi RPM, akselerasi, jitter,
// drop-tooth -> resync, stall -> Lost, pola salah, urutan toothIndex.
#include "ztune/TriggerDecoder.h"
#include "ToothSimulator.h"
#include "test_util.h"
#include <vector>

using namespace ztune;
using ztune_sim::ToothSim;

static float rpmConst1000(float) { return 1000.0f; }
static float rpmConst1500(float) { return 1500.0f; }
static float rpmConst2000(float) { return 2000.0f; }
static float rpmConst3000(float) { return 3000.0f; }
// 1000 -> 6000 rpm, ramp 2500 rpm/s (kasus akselerasi motor realistis)
static float rpmAccel(float t) {
    if (t < 0.5f) return 1000.0f;
    if (t < 2.5f) return 1000.0f + 2500.0f * (t - 0.5f);
    return 6000.0f;
}

static void feed(TriggerDecoder& d, ToothSim& s, uint32_t t0, uint32_t t1,
                 float (*f)(float), float jitterPct = 0.0f, long dropEdge = -1) {
    s.run(t0, t1, f, [&](uint32_t us) { d.onTooth(us); }, jitterPct, 0x9E3779B9u, dropEdge);
}

int main() {
    { // 1. Sinkron & akurasi RPM konstan (target +/-1%)
        TriggerConfig c = TriggerConfig::defaults();
        TriggerDecoder d(c);
        ToothSim s(c);
        feed(d, s, 0, 500000, rpmConst1500);
        CHECK(d.isSynced());
        CHECK_NEAR(d.rpm(), 1500.0, 15.0);
    }
    { // 2. Akselerasi 1000->6000: tetap sync tanpa Lost, RPM mengikuti (+/-2%)
        TriggerConfig c = TriggerConfig::defaults();
        TriggerDecoder d(c);
        ToothSim s(c);
        bool wasSynced = false, lostAfter = false;
        s.run(0, 2600000, rpmAccel, [&](uint32_t us) {
            d.onTooth(us);
            d.update(us);
            if (d.isSynced()) wasSynced = true;
            else if (wasSynced) lostAfter = true;
        });
        CHECK(wasSynced);
        CHECK(!lostAfter);
        CHECK_NEAR(d.rpm(), 6000.0, 120.0);
    }
    { // 3. Jitter 5% (derau VR): tetap sync; RPM mentah per-interval +/-5%
        TriggerConfig c = TriggerConfig::defaults();
        TriggerDecoder d(c);
        ToothSim s(c);
        feed(d, s, 0, 1000000, rpmConst3000, 5.0f);
        CHECK(d.isSynced());
        CHECK_NEAR(d.rpm(), 3000.0, 165.0);
    }
    { // 4. Satu edge hilang (VR kotor, gigi tengah): desink -> resink otomatis
        TriggerConfig c = TriggerConfig::defaults();
        TriggerDecoder d(c);
        ToothSim s(c);
        feed(d, s, 0, 500000, rpmConst2000);
        CHECK(d.isSynced());
        feed(d, s, 500000, 1200000, rpmConst2000, 0.0f, 50);
        CHECK(d.lostCount() >= 1);   // sempat keluar dari Synced
        CHECK(d.isSynced());         // pulih sendiri dalam ~2 putaran
        CHECK_NEAR(d.rpm(), 2000.0, 40.0);
    }
    { // 5. Stall: Lost via timeout, RPM nol, lalu resink saat gigi balik
        TriggerConfig c = TriggerConfig::defaults();
        TriggerDecoder d(c);
        ToothSim s(c);
        feed(d, s, 0, 300000, rpmConst1500);
        CHECK(d.isSynced());
        d.update(300000 + c.syncTimeoutUs + 1000);
        CHECK(d.state() == TriggerState::Lost);
        CHECK(d.rpm() == 0.0f);
        feed(d, s, 400000, 900000, rpmConst1500);
        CHECK(d.isSynced());
    }
    { // 6. Pola salah (roda 12-0 tanpa gap): TIDAK pernah sync (aman)
        TriggerConfig c = TriggerConfig::defaults();
        c.missingTeeth = 0;
        TriggerDecoder d(c);
        ToothSim s(c);
        feed(d, s, 0, 1000000, rpmConst1500);
        CHECK(d.state() == TriggerState::Hunting);
    }
    { // 7. toothIndex berputar 0..10 berurutan setelah sync
        TriggerConfig c = TriggerConfig::defaults();
        TriggerDecoder d(c);
        ToothSim s(c);
        std::vector<uint8_t> ring;
        bool captured = false;
        s.run(0, 500000, rpmConst1500, [&](uint32_t us) {
            d.onTooth(us);
            if (!captured && d.isSynced() && d.toothIndex() == 5) {
                captured = true;
                ring.clear();
            } else if (captured && ring.size() < 22) {
                ring.push_back(d.toothIndex());
            }
        });
        CHECK(captured);
        CHECK(ring.size() >= 16);
        bool ok = true;
        for (size_t i = 1; i < ring.size(); ++i) {
            uint8_t exp = (uint8_t)((ring[i - 1] + 1) % 11);
            if (ring[i] != exp) { ok = false; break; }
        }
        CHECK(ok);
    }
    { // 8. Idle rendah 1000 rpm tetap sync (cranking rendah)
        TriggerConfig c = TriggerConfig::defaults();
        TriggerDecoder d(c);
        ToothSim s(c);
        feed(d, s, 0, 800000, rpmConst1000);
        CHECK(d.isSynced());
        CHECK_NEAR(d.rpm(), 1000.0, 10.0);
    }
    return report("trigger");
}
