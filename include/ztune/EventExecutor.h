// EventExecutor (M4.2) - jembatan EventQueue (M4) -> timer hardware (ESP32).
// Pure logic: TIDAK ada include Arduino/ESP32, waktu eksplisit us -> unit
// test native. Keputusan yang diuji:
//   1. runDue()   : pop & eksekusi semua event dengan tUs <= now
//                   (komparasi cyclic (int32_t)(tUs - now) tahan wrap uint32).
//   2. planAlarm(): kapan alarm hw one-shot perlu di-set / di-reset.
// Aturan re-arm (dipakai identik oleh task dan ISR di MCU):
//   - queue kosong                 -> matikan alarm (rearm=false)
//   - head due sekarang / terlewat -> alarm kencang (delay min 1 tick);
//     eksekusinya sendiri lewat runDue() sesaat setelah ini
//   - alarm berjalan menyala LEBIH AWAL dari head baru -> biarkan alarm
//   - alarm berjalan menyala LEBIH LAMBAT dari head baru -> reset ke head
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "SchedulerTypes.h"

namespace ztune {

struct AlarmPlan {
    bool     rearm = false;   // true -> (re)set alarm hw pada delayUs dari now
    uint32_t delayUs = 0;     // offset alarm dari now (di-clamp min 1 tick)
};

struct EventExecutor {
    // Keputusan alarm berikutnya. Di MCU WAJIB dipanggil dalam critical
    // section (bersama EventQueue yang dibagi task <-> ISR).
    static AlarmPlan planAlarm(const EventQueue& q, bool alarmActive,
                               uint32_t alarmDueUs, uint32_t nowUs) {
        AlarmPlan p;
        if (q.empty()) return p;                       // tidak ada event -> idle
        uint32_t head = q.peek().tUs;
        int32_t toHead = (int32_t)(head - nowUs);
        if (toHead < 1) {                              // due / terlewat -> kencang
            p.rearm = true;
            p.delayUs = 1;
            return p;
        }
        if (alarmActive) {
            // alarmDueUs - head <= 0 (cyclic) berarti alarm berjalan menyala
            // lebih awal dari event baru -> tidak perlu reset.
            if ((int32_t)(alarmDueUs - head) <= 0) return p;
        }
        p.rearm = true;
        p.delayUs = (uint32_t)toHead;
        return p;
    }

    // Pop & jalankan semua event yang due (tUs <= now). Callback boleh
    // melakukan apa pun (di MCU: tulis GPIO; di test: catat).
    template <typename Cb>
    static void runDue(EventQueue& q, uint32_t nowUs, Cb cb) {
        while (!q.empty()) {
            const Ev e = q.peek();
            if ((int32_t)(e.tUs - nowUs) > 0) break;   // belum due
            q.pop();
            cb(e);
        }
    }
};

} // namespace ztune
