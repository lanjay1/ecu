// Test native M4.2: EventExecutor (planAlarm + runDue) - logika yang di MCU
// dijalankan di task dan ISR alarm timer hw. Semua kasus waktu dihitung
// manual, termasuk komparasi cyclic lintas wrap uint32.
#include "ztune/EventExecutor.h"
#include "ztune/EngineScheduler.h"
#include "test_util.h"
#include <vector>

using namespace ztune;

int main() {
    { // ---- planAlarm: queue kosong ----
        EventQueue q;
        AlarmPlan p = EventExecutor::planAlarm(q, false, 0, 1000);
        CHECK(!p.rearm);
        p = EventExecutor::planAlarm(q, true, 1500, 1000);   // alarm jalan, tapi kosong
        CHECK(!p.rearm);
    }
    { // ---- planAlarm: idle + head future ----
        EventQueue q;
        q.push({2000, EvAction::InjOpen, 0});
        AlarmPlan p = EventExecutor::planAlarm(q, false, 0, 1000);
        CHECK(p.rearm);
        CHECK(p.delayUs == 1000);
    }
    { // ---- planAlarm: head due sekarang / terlewat -> delay clamp 1 tick ----
        EventQueue q;
        q.push({2000, EvAction::InjOpen, 0});
        AlarmPlan p = EventExecutor::planAlarm(q, false, 0, 2000);
        CHECK(p.rearm);
        CHECK(p.delayUs == 1);
        p = EventExecutor::planAlarm(q, false, 0, 2100);     // overdue 100 us
        CHECK(p.rearm);
        CHECK(p.delayUs == 1);
    }
    { // ---- planAlarm: alarm berjalan lebih awal -> biarkan ----
        EventQueue q;
        q.push({2000, EvAction::InjOpen, 0});
        CHECK(!EventExecutor::planAlarm(q, true, 1800, 1000).rearm);
        CHECK(!EventExecutor::planAlarm(q, true, 2000, 1000).rearm);   // tepat sama
    }
    { // ---- planAlarm: alarm berjalan lebih lambat -> rearm ke head ----
        EventQueue q;
        q.push({2000, EvAction::InjOpen, 0});
        AlarmPlan p = EventExecutor::planAlarm(q, true, 2500, 1000);
        CHECK(p.rearm);
        CHECK(p.delayUs == 1000);
    }
    { // ---- planAlarm: komparasi cyclic lintas wrap uint32 ----
        EventQueue w;
        w.push({0x00000100u, EvAction::InjOpen, 0});         // head = +512 us (cyclic)
        AlarmPlan p = EventExecutor::planAlarm(w, false, 0, 0xFFFFFF00u);
        CHECK(p.rearm);
        CHECK(p.delayUs == 512);
        p = EventExecutor::planAlarm(w, true, 0x00000050u, 0xFFFFFF00u); // alarm lebih awal
        CHECK(!p.rearm);
        p = EventExecutor::planAlarm(w, true, 0x00000180u, 0xFFFFFF00u); // alarm lebih lambat
        CHECK(p.rearm);
        CHECK(p.delayUs == 512);
    }
    { // ---- runDue: only due, future tidak tersentuh ----
        EventQueue q;
        q.push({100, EvAction::InjOpen, 0});
        q.push({150, EvAction::InjClose, 0});
        q.push({300, EvAction::SparkFire, 1});
        std::vector<Ev> f;
        EventExecutor::runDue(q, 120, [&](const Ev& e) { f.push_back(e); });
        CHECK(f.size() == 1);
        CHECK(f[0].action == EvAction::InjOpen && f[0].tUs == 100);
        CHECK(q.size() == 2);
        EventExecutor::runDue(q, 149, [&](const Ev& e) { f.push_back(e); });
        CHECK(f.size() == 1);                                 // batas: belum due
        EventExecutor::runDue(q, 150, [&](const Ev& e) { f.push_back(e); });
        CHECK(f.size() == 2);
        CHECK(f[1].action == EvAction::InjClose);
        CHECK(q.size() == 1);
    }
    { // ---- runDue: banyak overdue tetap terurut ----
        EventQueue q;
        q.push({200, EvAction::SparkFire, 0});
        q.push({100, EvAction::InjOpen, 1});   // insertion sort -> 100 dulu
        std::vector<Ev> f;
        EventExecutor::runDue(q, 500, [&](const Ev& e) { f.push_back(e); });
        CHECK(f.size() == 2);
        CHECK(f[0].tUs == 100 && f[1].tUs == 200);
        CHECK(q.empty());
    }
    { // ---- runDue: wrap uint32 ----
        // Event 0x50: due 48 us SETELAH wrap melewati 0 (now 0x80).
        // Event 0x100: future 128 us dari 0x80 (belum due).
        EventQueue w;
        w.push({0x00000050u, EvAction::SparkFire, 0});
        w.push({0x00000100u, EvAction::InjClose, 0});
        std::vector<Ev> f;
        EventExecutor::runDue(w, 0x00000080u, [&](const Ev& e) { f.push_back(e); });
        CHECK(f.size() == 1);
        CHECK(f[0].action == EvAction::SparkFire);
        CHECK(w.size() == 1);
        EventExecutor::runDue(w, 0x00000100u, [&](const Ev& e) { f.push_back(e); });
        CHECK(f.size() == 2);
        CHECK(w.empty());
    }
    { // ---- runDue: queue kosong aman ----
        EventQueue e;
        int n = 0;
        EventExecutor::runDue(e, 12345, [&](const Ev&) { n++; });
        CHECK(n == 0);
    }
    { // ---- integrasi: scheduler -> executor simulasi alarm semu ----
        // 4 silinder @3000 rpm (SLOT 1666.67 us), 2 rev gigi. Model runtime
        // M4.2: alarm one-shot menyala TEPAT pada tUs event (jitter ISR 3 us),
        // task menjalankan runDue saat tooth tiba (event due tepat di tooth).
        // Semua event harus dieksekusi pada waktunya, queue akhirnya kosong.
        static constexpr double SLOT = 60.0e6 / (3000.0 * 12.0);
        static constexpr uint32_t T0 = 1000;
        static constexpr uint32_t JITTER = 3;   // simulasi latency ISR (us)
        EventQueue q;
        EngineScheduler sch;
        SchedInput in{};
        std::vector<Ev> fired;
        bool alarmActive = false;
        uint32_t alarmDue = 0;
        uint32_t maxLatencyUs = 0;

        in.synced = true;
        in.intervalUs = (float)SLOT;
        in.rpm = 3000.0f;
        in.totalTeeth = 12.0f;
        in.missingTeeth = 1.0f;
        in.phaseOffsetDeg = 0.0f;
        in.cylinders = 4;
        in.injEndBeforeTdcDeg = 60.0f;
        in.advanceDeg = 10.0f;
        in.dwellMs = 3.0f;
        in.pwMs = 2.0f;
        in.fuelCut = false;

        // ISR alarm semu: eksekusi due pada t (alarmDue + jitter), re-arm.
        auto fireIsr = [&](uint32_t t) {
            EventExecutor::runDue(q, t, [&](const Ev& e) {
                uint32_t lat = t - e.tUs;
                if (lat > maxLatencyUs) maxLatencyUs = lat;
                fired.push_back(e);
            });
            AlarmPlan p = EventExecutor::planAlarm(q, false, 0, t);
            if (p.rearm) { alarmDue = q.peek().tUs; alarmActive = true; }
            else alarmActive = false;
        };
        // Jalankan semua alarm yang menyala sebelum `until` (re-arm berantai).
        auto flushUntil = [&](uint32_t until) {
            for (int k = 0; k < 64 && alarmActive &&
                             (int32_t)(alarmDue + JITTER - until) < 0; ++k)
                fireIsr(alarmDue + JITTER);
        };

        int slots = 0;
        for (int rev = 0; rev < 2; ++rev) {
            for (uint8_t i = 0; i < 11; ++i) {
                in.toothIndex = i;
                uint32_t tUs = T0 + (uint32_t)llround((double)slots * SLOT);
                in.toothUs = tUs;
                flushUntil(tUs);                        // alarm ISR sebelum tooth
                sch.onTooth(in, q);                     // task: jadwal jendela gigi
                AlarmPlan p = EventExecutor::planAlarm(q, alarmActive, alarmDue, tUs);
                if (p.rearm) { alarmDue = q.peek().tUs; alarmActive = true; }
                slots += (i == 10) ? 2 : 1;
            }
        }
        // Buang sisa event jendela terakhir (spill lintas rev)
        uint32_t tEnd = T0 + (uint32_t)llround((double)slots * SLOT);
        flushUntil(tEnd);
        for (int k = 0; k < 64 && alarmActive; ++k) fireIsr(alarmDue + JITTER);

        CHECK(q.empty());
        CHECK(!alarmActive);
        // Setiap kombinasi action x channel dieksekusi tepat 2x (2 rev)
        for (int a = 0; a < 4; ++a)
            for (uint8_t ch = 0; ch < 4; ++ch) {
                int n = 0;
                for (const auto& e : fired)
                    if (e.action == (EvAction)a && e.channel == ch) n++;
                CHECK(n == 2);
            }
        // Latensi eksekusi maksimal = jitter ISR simulasi
        CHECK(maxLatencyUs <= JITTER);
        // Tidak ada drop karena queue tidak penuh pada rpm ini
        CHECK(q.dropped() == 0);
    }

    return report("test_executor");
}
