// ZtuneECU - glue ESP32-S3 (M0-M5)
// Core 1: engine task realtime (decoder trigger -> scheduler inj/ign ->
// eksekusi GPIO tepat waktu VIA TIMER HW + ISR; polling task jadi fallback).
// Core 0: telemetri USB + ZTP over USB CDC + BLE NUS + WiFi TCP (M3.1).
// M5: O2 narrowband/wideband + closed loop EGO.
//
// KEBIJAKAN NO-HARDCODE: nilai di EcuConfig::defaults() hanyalah safe-mode
// placeholder. Mesin terkunci sampai aplikasi tuning menulis config asli
// dan set tuneAccepted=true.
//
// CATATAN M4/M5: g_cfg dibaca engine task dan ditulis ZTP di Core 0 tanpa
// mutex (race jendela sempit, blob POD). g_sen lintas core sudah lewat
// snapshot spinlock (M5). TODO(M6): snapshot atomic/versioning g_cfg saat
// CONFIG_SET.
#include <Arduino.h>
#include <Preferences.h>
#include "ztune/EcuConfig.h"
#include "ztune/TriggerDecoder.h"
#include "ztune/SchedulerTypes.h"
#include "ztune/EngineScheduler.h"
#include "ztune/EventExecutor.h"
#include "ztune/RevLimiter.h"
#include "ztune/Quickshifter.h"
#include "ztune/SensorMath.h"
#include "ztune/AccelEnrichment.h"
#include "ztune/EgoControl.h"
#include "ztune/ZtpTypes.h"
#include "ztune/ZtpFrame.h"
#include "ztune/ZtpDispatcher.h"
#include "device/ZtpBle.h"
#include "device/ZtpWifi.h"

static EcuConfig g_cfg = EcuConfig::defaults();
static TriggerDecoder g_dec(g_cfg.trigger);

// ---- Ring buffer timestamp edge CKP: ISR -> task engine (lock-free SPSC) ----
static constexpr size_t RB_SIZE = 64;
static uint32_t s_rb[RB_SIZE];
static volatile size_t s_head = 0, s_tail = 0;

static void IRAM_ATTR ckpIsr() {
    uint32_t now = (uint32_t)micros();
    size_t h = s_head;
    size_t n = (h + 1) % RB_SIZE;
    if (n != s_tail) {           // penuh -> buang (jangan pernah blok ISR)
        s_rb[h] = now;
        s_head = n;
    }
}

// ---- State M4: queue event + aktuator logika (pure logic, feed timestamp) ----
static EventQueue g_eq;
static EngineScheduler g_sched;
static RevLimiter g_lim;
static Quickshifter g_qs;
static bool g_outputsLive = false;   // true saat output aktif (sync + tune ok)

// ---- State M4.2: timer hw utk eksekusi event presisi (task <-> ISR) ----
// EventQueue dibagi task (push/clear) dan ISR alarm (pop). Semua akses queue
// dan alarm ada di critical section (spinlock); EventQueue sendiri tetap pure
// sehingga keputusan plan/run diuji native via EventExecutor.
static hw_timer_t* g_evtTimer = nullptr;
static portMUX_TYPE g_evtMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_alarmActive = false;   // alarm one-shot sedang berjalan
static volatile uint32_t g_alarmDueUs = 0;    // tUs absolut saat alarm menyala

// ---- State M4.1: sensor + ASE + nilai terakhir untuk live telemetry ----
// g_sen ditulis loop (Core 0) via readSensors() dan dibaca engine task
// (Core 1) lewat SNAPSHOT spinlock: field konsisten satu versi snapshot
// (M5 - menuntaskan TODO atomic). EMA hanya disentuh loop, di luar lock.
static SensorSnapshot g_sen;
static portMUX_TYPE g_senMux = portMUX_INITIALIZER_UNLOCKED;
static SensorSnapshot snapSen() {
    portENTER_CRITICAL(&g_senMux);
    SensorSnapshot s = g_sen;
    portEXIT_CRITICAL(&g_senMux);
    return s;
}
static EmaFilter g_tpsEma(0.25f), g_mapEma(0.25f), g_cltEma(0.10f),
                 g_iatEma(0.10f), g_battEma(0.20f), g_o2Ema(0.30f);
static AccelEnrichment g_ase;
static EgoControl g_ego;                 // M5: closed loop (engine task saja)
static volatile float g_lastPwMs = 0.0f;
static volatile float g_lastAdvDeg = 0.0f;
// Telemetri closed loop (ditulis engine task, dibaca Core 0 utk live frame)
static volatile float g_lastEgoPct = 0.0f;
static volatile float g_lastAfrMeas = 0.0f;
static volatile float g_lastAfrTgt  = 0.0f;
static volatile bool  g_lastEgoAct = false;

// Baca seluruh channel ADC + konversi + smoothing. Dipanggil dari loop().
// Build snapshot lokal dulu, publish via spinlock -> engine task melihat
// satu versi konsisten.
static void readSensors() {
    SensorSnapshot sen;
    int tpsRaw  = analogRead(g_cfg.io.tpsGpio);
    int mapRaw  = analogRead(g_cfg.io.mapGpio);
    int cltRaw  = analogRead(g_cfg.io.cltGpio);
    int iatRaw  = analogRead(g_cfg.io.iatGpio);
    int battRaw = analogRead(g_cfg.io.battGpio);

    sen.tpsPct = g_tpsEma.update(SensorMath::tpsPct(
        tpsRaw, g_cfg.sensors.tpsClosedAdc, g_cfg.sensors.tpsWotAdc));
    sen.mapKpa = g_mapEma.update(SensorMath::mapKpa(
        mapRaw, g_cfg.sensors.mapZeroAdc, g_cfg.sensors.mapFullAdc,
        g_cfg.sensors.mapZeroKpa, g_cfg.sensors.mapFullKpa));

    // NTC: fault (open/short) -> fallback ke T0 (nilai aman utk motor)
    float clt = SensorMath::ntcC(cltRaw, g_cfg.sensors.cltPullupOhm,
                                 g_cfg.sensors.cltBetaK, g_cfg.sensors.cltR0Ohm,
                                 g_cfg.sensors.cltT0C);
    sen.cltFault = (clt <= -99.0f);
    if (!sen.cltFault) sen.cltC = g_cltEma.update(clt);

    float iat = SensorMath::ntcC(iatRaw, g_cfg.sensors.iatPullupOhm,
                                 g_cfg.sensors.iatBetaK, g_cfg.sensors.iatR0Ohm,
                                 g_cfg.sensors.iatT0C);
    sen.iatFault = (iat <= -99.0f);
    if (!sen.iatFault) sen.iatC = g_iatEma.update(iat);

    sen.battV = g_battEma.update(SensorMath::battV(battRaw,
                                                   g_cfg.sensors.battDivRatio));

    // O2 (M5): skip bila tipe None (pin boleh mengambang); fault bila raw
    // menempel ujung skala (sensor mati/pengendali WB belum siap).
    if (g_cfg.ego.o2Type != O2Type::None) {
        int o2Raw = analogRead(g_cfg.io.o2Gpio);
        sen.o2Fault = (o2Raw <= 2 || o2Raw >= 4093);
        sen.o2V = g_o2Ema.update(SensorMath::o2Volts(o2Raw));
        if (g_cfg.ego.o2Type == O2Type::Wideband)
            sen.o2Afr = SensorMath::o2Afr(o2Raw, g_cfg.ego.wbAdcLow,
                                          g_cfg.ego.wbAfrLow, g_cfg.ego.wbAdcHigh,
                                          g_cfg.ego.wbAfrHigh);
    } else {
        sen.o2Fault = true;
    }

    portENTER_CRITICAL(&g_senMux);
    g_sen = sen;
    portEXIT_CRITICAL(&g_senMux);
}

static inline void IRAM_ATTR injWrite(uint8_t ch, bool on) {
    if (ch < g_cfg.engine.cylinders && ch < 6)
        digitalWrite(g_cfg.io.injGpio[ch], on ? HIGH : LOW);
}
static inline void IRAM_ATTR ignWrite(uint8_t ch, bool on) {
    if (ch < g_cfg.engine.cylinders && ch < 6)
        digitalWrite(g_cfg.io.ignGpio[ch], on ? HIGH : LOW);
}

// Safety: hilang sync / safe mode -> semua aktuator OFF.
static void outputsOff() {
    for (uint8_t c = 0; c < g_cfg.engine.cylinders && c < 6; ++c) {
        injWrite(c, false);
        ignWrite(c, false);
    }
    g_outputsLive = false;
}

// Tulis GPIO satu event (dipakai task DAN ISR alarm) + tandai output hidup.
static inline void IRAM_ATTR evtWrite(const Ev& e) {
    switch (e.action) {
        case EvAction::InjOpen:    injWrite(e.channel, true);  break;
        case EvAction::InjClose:   injWrite(e.channel, false); break;
        case EvAction::DwellStart: ignWrite(e.channel, true);  break;
        case EvAction::SparkFire:  ignWrite(e.channel, false); break;
    }
    g_outputsLive = true;
}

// (Re)set alarm one-shot ke tUs absolut. WAJIB dipanggil dalam critical
// section. Counter timer di-reset tiap arm: offset = due - now (1 us/tick).
static inline void armAlarmLocked(uint32_t dueUs, uint32_t nowUs) {
    uint32_t d = dueUs - nowUs;
    if (d < 1) d = 1;                     // clamp min 1 tick
    timerAlarmDisable(g_evtTimer);
    timerWrite(g_evtTimer, 0);
    timerAlarmWrite(g_evtTimer, d, false);   // one-shot (autoreload=false)
    timerAlarmEnable(g_evtTimer);
    g_alarmDueUs = dueUs;
    g_alarmActive = true;
}

// ISR alarm hw (1 us/tick): eksekusi event yang due lalu re-arm berikutnya.
// Latency ISR ~2-4 us -> presisi sudut jauh lebih baik dari polling task
// (~80 us + jitter scheduler). micros() = esp_timer_get_time, aman di ISR.
static void IRAM_ATTR onEvtAlarm() {
    portENTER_CRITICAL_ISR(&g_evtMux);
    uint32_t now = (uint32_t)micros();
    EventExecutor::runDue(g_eq, now, evtWrite);
    AlarmPlan p = EventExecutor::planAlarm(g_eq, false, 0, now);
    if (p.rearm) armAlarmLocked(g_eq.peek().tUs, now);
    else { g_alarmActive = false; timerAlarmDisable(g_evtTimer); }
    portEXIT_CRITICAL_ISR(&g_evtMux);
}

// Safety: hilang sync / safe mode -> matikan alarm + queue + semua aktuator.
static void killEventsLocked() {
    g_eq.clear();
    g_alarmActive = false;
    timerAlarmDisable(g_evtTimer);
}

// Susun input scheduler dari decoder + sensor + tabel + FuelMath + cut.
// M4.1: load nyata (TPS alpha-N atau MAP), koreksi WUE/IAT/ASE pada PW.
// M5: closed loop EGO - state diupdate per tooth, koreksi % masuk PW.
static SchedInput makeSchedInput(uint32_t toothUs, bool fuelCut) {
    SensorSnapshot sen = snapSen();
    SchedInput in;
    in.synced = true;
    in.toothIndex = g_dec.toothIndex();
    in.toothUs = toothUs;
    in.intervalUs = g_dec.intervalUs();
    in.rpm = g_dec.rpm();

    in.totalTeeth = (float)g_cfg.trigger.totalTeeth;
    in.missingTeeth = (float)g_cfg.trigger.missingTeeth;
    in.phaseOffsetDeg = g_cfg.trigger.phaseOffsetDeg;
    in.cylinders = g_cfg.engine.cylinders;
    in.injEndBeforeTdcDeg = g_cfg.fuel.injEndBeforeTdcDeg;

    float load = (g_cfg.fuel.loadSource == LoadSource::MAP)
                 ? sen.mapKpa : sen.tpsPct;
    in.advanceDeg = g_cfg.ign.advance.lookup(in.rpm, load);
    in.dwellMs = g_cfg.ign.dwell.lookup(in.rpm);
    float ve = g_cfg.fuel.ve.lookup(in.rpm, load);
    float reqFuel = FuelMath::reqFuelMs(g_cfg.engine.dispPerCylCc,
                                        g_cfg.engine.stoichAfr,
                                        g_cfg.fuel.injectorCcPerMin);
    // Dead time dinamis dari kurva vs tegangan baterai (M4.2): PW eksekusi =
    // PW aliran + dead time. Volt rendah (cranking) -> dead time membesar.
    float dtMs = g_cfg.fuel.dtime.lookup(sen.battV);
    float pwBase = FuelMath::pulseWidthMs(reqFuel, ve,
                                          g_cfg.engine.squirtsPerCycle, dtMs);
    // ASE: deteksi tip-in per tooth (rate TPS dihitung dari sampel berurutan)
    g_ase.onTps(sen.tpsPct, toothUs);
    float wue = g_cfg.fuel.wue.lookup(sen.cltC);
    float iat = g_cfg.fuel.iatCorr.lookup(sen.iatC);
    float ase = g_ase.enrichPct(toothUs);
    // Closed loop EGO (M5): gate di dalam (rpm/TPS/CLT/fuelCut/ASE/fault).
    EgoControl::Input ei;
    ei.tMs = toothUs / 1000u;
    ei.rpm = in.rpm;
    ei.load = load;
    ei.tpsPct = sen.tpsPct;
    ei.cltC = sen.cltC;
    ei.o2V = sen.o2V;
    ei.o2Afr = sen.o2Afr;
    ei.o2Fault = sen.o2Fault;
    ei.fuelCut = fuelCut;
    ei.aseActive = g_ase.active() || ase > 0.0f;
    const EgoState& es = g_ego.update(ei);
    in.pwMs = FuelMath::correctedPwMs(pwBase, wue, iat, ase, es.corrPct);
    in.fuelCut = fuelCut;

    g_lastPwMs = in.pwMs;          // untuk LIVE_FRAME / telemetri
    g_lastAdvDeg = in.advanceDeg;
    g_lastEgoPct  = es.corrPct;
    g_lastAfrMeas = es.measuredAfr;
    g_lastAfrTgt  = es.targetAfr;
    g_lastEgoAct  = es.active;
    return in;
}

// ---- Task realtime Core 1: tooth -> scheduler -> eksekusi event ----
static void engineTask(void*) {
    for (;;) {
        while (s_tail != s_head) {
            uint32_t ts = s_rb[s_tail];
            s_tail = (s_tail + 1) % RB_SIZE;

            g_dec.onTooth(ts);

            // Safe mode atau tidak sync -> output terkunci, event basi dibuang.
            if (!g_cfg.tuneAccepted || !g_dec.isSynced()) {
                g_ego.reset();   // M5: koreksi closed loop dibuang saat tidak sync
                if (g_outputsLive) {
                    portENTER_CRITICAL(&g_evtMux);
                    killEventsLocked();
                    portEXIT_CRITICAL(&g_evtMux);
                    outputsOff();
                }
                g_lim.onRpm(g_dec.rpm());
                continue;
            }

            // Quickshifter: sampling pin tiap tooth (>= ribuan sampel/detik,
            // jauh di atas kebutuhan debounce milidetik).
            g_qs.onInput(digitalRead(g_cfg.io.qsInputGpio) == HIGH,
                         g_dec.rpm(), ts);

            // Rev limiter + quickshifter -> keputusan fuel cut gabungan.
            g_lim.onRpm(g_dec.rpm());
            bool cut = g_lim.fuelCut() || g_qs.fuelCut(ts);

            // M4.2: jadwal event lalu keputusan alarm hw (critical: queue
            // dibagi dengan ISR alarm di core yang sama / spinlock lintas core).
            SchedInput in = makeSchedInput(ts, cut);
            portENTER_CRITICAL(&g_evtMux);
            g_sched.onTooth(in, g_eq);
            EventExecutor::runDue(g_eq, ts, evtWrite);
            AlarmPlan p = EventExecutor::planAlarm(g_eq, g_alarmActive,
                                                   g_alarmDueUs, ts);
            if (p.rearm) armAlarmLocked(g_eq.peek().tUs, ts);
            portEXIT_CRITICAL(&g_evtMux);
        }

        // M4.2: eksekusi presisi dilakukan alarm hw di ISR. Task cukup:
        // fallback tangkap event due (ISR telat/blok lama) + re-arm, lalu
        // tidur 1 tick. Tanpa spin halus -> beban Core 1 turun drastis.
        vTaskDelay(1);
        uint32_t now = (uint32_t)micros();
        portENTER_CRITICAL(&g_evtMux);
        EventExecutor::runDue(g_eq, now, evtWrite);
        AlarmPlan fp = EventExecutor::planAlarm(g_eq, g_alarmActive,
                                                g_alarmDueUs, now);
        if (fp.rearm) armAlarmLocked(g_eq.peek().tUs, now);
        portEXIT_CRITICAL(&g_evtMux);

        // Deteksi stall (timeout tanpa edge) -> Lost -> output OFF.
        g_dec.update((uint32_t)micros());
        if (!g_dec.isSynced()) g_ego.reset();   // M5: stall juga buang koreksi
        if (g_outputsLive && !g_dec.isSynced()) {
            portENTER_CRITICAL(&g_evtMux);
            killEventsLocked();
            portEXIT_CRITICAL(&g_evtMux);
            outputsOff();
        }
    }
}

// ---- Persist config NVS: blob + CRC32 + schema gate ----
static void saveConfig(const EcuConfig& c) {
    Preferences p;
    if (!p.begin("ztune", false)) return;
    p.putBytes("ecuCfg", &c, sizeof(EcuConfig));
    p.putULong("ecuCfgCrc", crc32(&c, sizeof(EcuConfig)));
    p.end();
}

static bool loadConfig() {
    Preferences p;
    if (!p.begin("ztune", true)) return false;
    EcuConfig c;
    size_t got = p.getBytes("ecuCfg", &c, sizeof(EcuConfig));
    uint32_t crc = p.getULong("ecuCfgCrc", 0);
    p.end();
    if (got != sizeof(EcuConfig)) return false;
    if (crc32(&c, sizeof(EcuConfig)) != crc) return false;
    if (c.schemaVersion != ZTUNE_SCHEMA_VERSION) return false;
    char err[128];
    if (!c.validate(err, sizeof(err))) return false;
    g_cfg = c;
    return true;
}

// ---- ZTP v1: multi-transport USB CDC + BLE NUS + WiFi TCP (M3.1) ----
// Live frame keluar di transport yang meminta LIVE_START (satu aktif).
enum class Iface : uint8_t { Usb = 0, Ble = 1, Wifi = 2 };
static Iface g_liveIf = Iface::Usb;

static bool g_liveActive = false;
static ztp::ZtpCtx g_ztp;
static ztp::StreamDecoder g_ztpDecUsb, g_ztpDecBle, g_ztpDecWifi;
static ztp::ZtpBleTransport g_bleT;
static ztp::ZtpWifiTransport g_wifiT;
static uint8_t g_rsp[ztp::HEADER_SIZE + ztp::MAX_PAYLOAD + ztp::CRC_SIZE];
static uint32_t g_tLastLiveMs = 0;

static void ztpSend(Iface ifc, const uint8_t* buf, size_t n) {
    switch (ifc) {
        case Iface::Usb:  Serial.write(buf, n); break;
        case Iface::Ble:  g_bleT.send(buf, n); break;
        case Iface::Wifi: g_wifiT.send(buf, n); break;
    }
}

static void handleFrame(Iface ifc, uint8_t type, const uint8_t* p, size_t len) {
    size_t n = ztp::dispatch(g_ztp, type, p, len, g_rsp, sizeof(g_rsp));
    if (n) ztpSend(ifc, g_rsp, n);
    if (type == (uint8_t)ztp::Type::LIVE_START) g_liveIf = ifc;
}

static void onUsbFrame(uint8_t type, const uint8_t* p, size_t len) {
    handleFrame(Iface::Usb, type, p, len);
}
static void onUsbError(uint8_t code) {
    size_t n = ztp::err(g_rsp, sizeof(g_rsp), (ztp::Err)code, 0, "frame rusak");
    if (n) ztpSend(Iface::Usb, g_rsp, n);
}

void setup() {
    Serial.begin(115200);
    if (!loadConfig()) {
        g_cfg = EcuConfig::defaults();
        saveConfig(g_cfg);       // boot pertama: tulis default safe-mode ke NVS
    }
    g_dec.setConfig(g_cfg.trigger);

    analogReadResolution(12);    // 0..4095 @ 3.3V (konvensi SensorMath)
    g_ase.configure(g_cfg.fuel.aseTpsRatePctPerS, g_cfg.fuel.asePct,
                    g_cfg.fuel.aseDurationMs);
    g_ego.configure(g_cfg.ego);   // M5: closed loop EGO

    g_ztp.cfg = &g_cfg;
    g_ztp.liveActive = &g_liveActive;
    g_ztp.liveRateMs = 100;
    g_ztp.onPersist = saveConfig;    // CONFIG_SET / TABLE_WRITE -> langsung persist NVS

    // Transport tuning (M3.1): BLE NUS + WiFi STA/TCP, sesuai EcuConfig.
    if (g_cfg.comms.bleEnable) {
        g_bleT.setName(g_cfg.comms.bleName);
        g_bleT.begin(g_ztpDecBle,
                     [](uint8_t t, const uint8_t* p, size_t n) {
                         handleFrame(Iface::Ble, t, p, n);
                     },
                     [](uint8_t code) {
                         size_t n = ztp::err(g_rsp, sizeof(g_rsp), (ztp::Err)code, 0,
                                             "frame rusak");
                         if (n) ztpSend(Iface::Ble, g_rsp, n);
                     });
    }
    if (g_cfg.comms.wifiEnable) {
        g_wifiT.begin(g_ztpDecWifi, g_cfg.comms.wifiSsid, g_cfg.comms.wifiPass,
                      g_cfg.comms.wifiTcpPort,
                      [](uint8_t t, const uint8_t* p, size_t n) {
                          handleFrame(Iface::Wifi, t, p, n);
                      },
                      [](uint8_t code) {
                          size_t n = ztp::err(g_rsp, sizeof(g_rsp), (ztp::Err)code, 0,
                                              "frame rusak");
                          if (n) ztpSend(Iface::Wifi, g_rsp, n);
                      });
    }

    // GPIO: CKP input; injektor/coil output LOW; sensor gear QS input.
    pinMode(g_cfg.io.ckpGpio, INPUT_PULLUP);
    for (uint8_t c = 0; c < g_cfg.engine.cylinders && c < 6; ++c) {
        pinMode(g_cfg.io.injGpio[c], OUTPUT);
        digitalWrite(g_cfg.io.injGpio[c], LOW);
        pinMode(g_cfg.io.ignGpio[c], OUTPUT);
        digitalWrite(g_cfg.io.ignGpio[c], LOW);
    }
    pinMode(g_cfg.io.qsInputGpio, INPUT_PULLDOWN);   // sensor gear aktif HIGH

    g_lim.configure(g_cfg.revLimit.enabled,
                    g_cfg.revLimit.hardLimitRpm,
                    g_cfg.revLimit.resumeDropRpm);
    g_qs.configure(g_cfg.quickShift.enabled,
                   g_cfg.quickShift.cutMs,
                   g_cfg.quickShift.minRpm,
                   g_cfg.quickShift.debounceMs);

    // M4.2: timer hw eksekusi event - 80 MHz/80 = 1 MHz (1 tick = 1 us),
    // alarm one-shot per event terdekat; di-arm saat event pertama dijadwalkan.
    g_evtTimer = timerBegin(2, 80, true);
    timerAttachInterrupt(g_evtTimer, &onEvtAlarm, true);
    timerAlarmDisable(g_evtTimer);

    // Task realtime di Core 1 (Core 0 untuk USB/BT/WiFi stack)
    xTaskCreatePinnedToCore(engineTask, "eng", 4096, nullptr, 10, nullptr, 1);

    Serial.printf("\n[ZtuneECU %s] schema=%u safeMode=%s exec=hw-timer ztp=%s%s\n",
                  ZTUNE_FW_VERSION, ZTUNE_SCHEMA_VERSION,
                  g_cfg.tuneAccepted ? "OFF" : "ON",
                  g_cfg.comms.bleEnable ? "BLE" : "",
                  g_cfg.comms.wifiEnable ? "+WiFi" : "");
}

void loop() {
    // RX ZTP per transport: USB CDC + WiFi TCP pump (BLE via callback NimBLE)
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        g_ztpDecUsb.feed(&b, 1, onUsbFrame, onUsbError);
    }
    if (g_cfg.comms.wifiEnable) g_wifiT.pump();

    // LIVE_FRAME streaming keluar di transport yang meminta LIVE_START
    uint32_t nowMs = millis();
    if (g_liveActive && nowMs - g_tLastLiveMs >= g_ztp.liveRateMs) {
        g_tLastLiveMs = nowMs;
        readSensors();           // isi snapshot sensor utk live view
        SensorSnapshot snap = snapSen();
        ztp::LiveSensors sen;
        sen.tpsPct = snap.tpsPct;   sen.mapKpa = snap.mapKpa;
        sen.cltC = snap.cltC;       sen.iatC = snap.iatC;
        sen.battV = snap.battV;
        ztp::EgoState egoT;      // telemetri closed loop (lintas core, float)
        egoT.corrPct = g_lastEgoPct;
        egoT.measuredAfr = g_lastAfrMeas;
        egoT.targetAfr = g_lastAfrTgt;
        egoT.active = g_lastEgoAct;
        ztp::LiveFrameV2 lf = ztp::buildLiveFrame(g_cfg, g_dec, nowMs, sen,
                                                  g_lastPwMs, g_lastAdvDeg, egoT);
        size_t n = ztp::encodeFrame(g_rsp, sizeof(g_rsp),
                                    (uint8_t)ztp::Type::LIVE_FRAME, &lf, sizeof(lf));
        if (n) ztpSend(g_liveIf, g_rsp, n);
    }

    // Telemetri teks hanya saat live stream mati (port sama: mode teks vs biner)
    static uint32_t tLast = 0;
    if (!g_liveActive && nowMs - tLast >= 500) {
        tLast = nowMs;
        readSensors();
        SensorSnapshot snap = snapSen();
        static const char* ST[] = {"Hunting", "Synced", "Lost"};
        Serial.printf("rpm=%.0f %s tps=%.1f%% map=%.1fkPa clt=%.0fC iat=%.0fC batt=%.1fV "
                      "pw=%.2fms adv=%.1f lim=%d qs=%d afr=%.1f tgt=%.1f ego=%.0f%%%s%s\n",
                      g_dec.rpm(), ST[(int)g_dec.state()],
                      snap.tpsPct, snap.mapKpa, snap.cltC, snap.iatC, snap.battV,
                      g_lastPwMs, g_lastAdvDeg,
                      (int)g_lim.fuelCut(), (int)g_qs.fuelCut(micros()),
                      g_lastAfrMeas, g_lastAfrTgt, g_lastEgoPct,
                      g_lastEgoAct ? " CL" : "",
                      g_cfg.tuneAccepted ? "" : "  [SAFE MODE]");
    }
}
