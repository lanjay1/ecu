# ZtuneECU

Firmware ECU motor berbasis **ESP32-S3**, arsitektur bersih terinspirasi
[TritonECU](https://github.com/alpauna/TritonECU) (pola dual-core + pure-logic
yang bisa di-test native) dan referensi decoder rusEFI (kondisioner VR **MAX9926**).

## Keputusan desain (final, dari owner)

| Item | Keputusan |
|------|-----------|
| Trigger | **Opsi A: tanpa cam** - crank-only 12-1 (configurable), MAX9926, wasted spark |
| Fuel | MegaSquirt style: `PW = reqFuel x VE%`, `reqFuel = disp x stoich x 500 / (inj cc/min)` |
| Tabel | VE & advance **16x16**, load source MAP/TPS (configurable) |
| Komunikasi | **ZTP v2** (protokol binary native) via USB + WiFi + BLE - tanpa web, tanpa AP hotspot |
| Fitur prioritas | Quickshifter + rev limiter (M4) - **selesai** |
| O2 | **M5 selesai**: narrowband (switch) & wideband (linier 2 titik) + closed loop EGO |
| Transmisi elektrik | DIHAPUS (tidak ada di ZtuneECU) |

## Kebijakan NO-HARDCODE

Tidak ada satu pun parameter mesin yang dibekukan di kode. Semua lewat
`EcuConfig` (persist NVS, blob + CRC32 + schemaVersion). Nilai default hanyalah
**safe-mode placeholder**: firmware hanya menjalankan decoder + telemetri sampai
aplikasi tuning menulis config asli dan set `tuneAccepted=true`.

## Scheduler M4 (injeksi & penyalaan)

Pure logic (`EngineScheduler.h` + `SchedulerTypes.h`), diuji native dengan
nilai analitik @3000 rpm. Prinsip kerja per gigi trigger:

```
Jendela gigi: (theta_tooth, theta_next]  (gigi terakhir = 2 pitch, memuat gap)
Prediksi waktu: t_event = t_tooth + intervalPitch x (sudut_ke_event / sudut_per_pitch)
Sudut event per silinder k:
  TDC k      = phaseOffsetDeg + k x (720/cyl)   mod 360  (crank-only: tiap rev identik)
  Spark      = TDC k - advance                            (tabel 16x16)
  DwellStart = Spark - degForMs(dwellMs, rpm)             (kurva 1D)
  InjClose   = TDC k - injEndBeforeTdcDeg                 (konvensi MegaSquirt)
  InjOpen    = InjClose - degForMs(PW, rpm)               (PW = reqFuel x VE% + deadtime)
```

- Event yang jatuh di jendela gigi masuk `EventQueue` (terurut, kapasitas 32,
  komparasi cyclic tahan wrap uint32). Eksekusinya via **timer hardware** (M4.2,
  lihat bawah): alarm one-shot per event, ISR menulis GPIO; polling task hanya
  fallback.
- **Rev limiter**: hard fuel-cut di `hardLimitRpm`, resume setelah turun
  `resumeDropRpm` (histeresis). Spark tetap jalan (hanya bahan bakar dipotong).
- **Quickshifter**: sensor gear (debounce `debounceMs`) -> fuel cut `cutMs`
  per upshift saat rpm >= `minRpm`; edge baru butuh sensor lepas dulu.
- Safety: safe mode / lost sync / stall -> semua output OFF + queue dibuang.
- Fase crank-only: satu rev = satu set event identik (wasted spark + injeksi
  batch); dua silinder bertumpuk di sudut sama = pasangan wasted yang benar.

## Closed loop bahan bakar - O2 & EGO (M5)

Pure logic (`EgoControl.h` + konversi O2 di `SensorMath.h`), semua via
`EcuConfig.ego` (schema v6):

```
Sumber umpan balik (ego.o2Type):
  Narrowband : tegangan switch stoich (nbSwitchV +- nbHystV) - osilasi di stoich
  Wideband   : AFR = linier 2 titik ADC (wbAdcLow/High -> wbAfrLow/High),
               dibandingkan target dari tabel AFR 16x16 (rpm x load)
Koreksi (integral step, gaya MegaSquirt):
  tiap updateMs, error melewati deadbandAfr -> corr += -stepPct arah hapus error
  clamp +-limitPct; corr dijumlahkan ke PW seperti WUE/IAT/ASE (clamp 0.2x..3.0x)
Gate (di luar kondisi ini koreksi = 0 / open loop):
  enabled + o2Type != None + !o2Fault + !fuelCut(limiter/QS) + !ASE aktif
  + rpm di [minRpm..maxRpm] + TPS <= maxTpsPct (cruise) + CLT >= minCltC
sensorDelayMs: setelah gate OK, tunggu dulu (transportasi gas + settle sensor;
  set besar ~20 s juga meng-cover warmup pengendali wideband)
```

- Safety tambahan: lost sync / stall / safe mode -> state EGO di-reset
  (koreksi lama tidak dibawa saat re-sync).
- `SensorSnapshot` lintas core kini lewat **snapshot spinlock** (field
  konsisten satu versi) - menuntaskan TODO atomic dari M4.1.
- ZTP naik ke **v2**: `PROTO_VER=2`, `LiveFrameV2` (+`afrMeasured`,
  `afrTarget`, `egoPct`, `egoActive`), tabel target AFR via
  `TABLE_READ/WRITE TableId::AFR_TARGET`, `MAX_PAYLOAD` 8192 (blob config
  tumbuh), dan `HELLO.caps` kini mencerminkan `comms` (bit1 BLE, bit2 WiFi).

## Sensor & koreksi bahan bakar (M4.1)

Pure logic (`SensorMath.h` + `AccelEnrichment.h`), semua kalibrasi via
`EcuConfig.sensors` (schema v4):

```
TPS  : kalibrasi 2 titik ADC raw (closed/WOT) -> 0..100 %  (alpha-N)
MAP  : linier 2 titik ADC -> kPa                              (speed-density)
CLT  : NTC persamaan beta (pull-up, R0, beta, T0) -> C; open/short -> fault
IAT  : idem CLT
BATT : pembagi resistif -> volt
```

- Semua channel dihaluskan EMA (alpha per channel), sensor WAJIB di **ADC1
  (GPIO 1..10)** karena ADC2 bentrok dengan WiFi.
- **Koreksi PW** (gaya MegaSquirt, dijumlahkan sebagai %, clamp 0.2x..3.0x):
  `PW = PW_base x (1 + WUE% + IAT% + ASE%)`
  - WUE: kurva 16 titik vs suhu coolant (default: +45% di -10 C -> 0% di 80 C)
  - IAT: kurva 16 titik koreksi densitas (+-8%)
  - ASE: tip-in TPS >= ambang (%/s) -> enrich ramp-down linear selama durasi
- Load untuk VE mengikuti `fuel.loadSource`: TPS (alpha-N) atau MAP kPa.
- LIVE_FRAME kini membawa TPS/MAP/CLT/IAT/batt + PW & advance terakhir nyata.

## Timer hardware & dead time (M4.2)

**Eksekusi event presisi (menggantikan polling spin ~80 us):**

- `EventExecutor.h` (pure, diuji native): `runDue()` pop event due (cyclic,
  tahan wrap) + `planAlarm()` keputusan re-arm — dipakai identik oleh task
  dan ISR, sehingga logika yang sama teruji di `test_executor`.
- MCU: `hw_timer` 1 us/tick (80 MHz/80), **alarm one-shot per event terdekat**;
  ISR `IRAM_ATTR` menulis GPIO lalu re-arm event berikutnya. Latency ISR
  ~2-4 us -> presisi sudut jauh lebih baik dari polling task.
- Semua akses `EventQueue` + alarm di critical section (spinlock) karena
  dibagi task (push/clear) <-> ISR (pop).
- Task engine cukup `vTaskDelay(1)` + fallback `runDue` (jitter-proof),
  beban Core 1 turun drastis; alarm menyala tepat pada tUs event.

**Dead time injektor = KURVA vs tegangan baterai (bukan skalar):**

```
PW_eksekusi = PW_aliran + dtime(battV)      # dtime: kurva 16 titik 6..21 V
```

- Injektor membuka lebih lambat saat volt rendah (cranking 6 V ~ 2x dead time
  12 V); kurva khas high-Z default: 6 V=1.60 ms, 12 V=0.82 ms, 21 V=0.70 ms,
  di luar rentang di-clamp.
- Dihitung ulang per gigi dari `sensors.battV` (EMA) -> PW masuk scheduler.
- Validasi: sumbu x naik ketat 0..30 V, y 0..5 ms; schema config sekarang
  **v5** (blob lama ditolak -> safe-mode, wajib re-tune via app).

## ZTP v1 (protokol - status placeholder lean)

```
Frame: [0x5A 'Z'][0x54 'T'][ver=0x01][type u8][len u16 LE][payload][CRC32 u32 LE]
CRC mencakup ver..payload -> magic murni titik resync (tahan noise/chunking BLE).
```

| Type | Arah | Isi |
|------|------|-----|
| 0x01 HELLO | fw->host | HelloInfo (protoVer, schemaVersion, caps, fwVersion) |
| 0x02/0x03 CONFIG_GET/SET | dua arah | payload = blob `EcuConfig` (validate + persist otomatis) |
| 0x04/0x05 TABLE_READ/WRITE | dua arah | VE=1, Advance=2, Dwell=3; `[axisRpm 16][axisLoad 16][values 256]` |
| 0x06/0x07 LIVE_START/STOP | host->fw | rateMs u16 (min 10 ms) |
| 0x08 LIVE_FRAME | fw->host | LiveFrameV1 packed ~44 B: rpm, state, tooth, VE, PW, advance |
| 0x0A PING / 0x0B PONG | dua arah | echo |
| 0xFF ERROR | fw->host | [code u8][reqType u8][pesan] |

Konvensi: sukses = echo type request; gagal = ERROR. Stream decoder tahan
frame terpotong, byte sampah, dan CRC korup (resync otomatis).

### Transport (M3.1 - real)

| Interface | Mode | Detail |
|-----------|------|--------|
| USB CDC | aktif default | Serial USB, satu decoder |
| **BLE** | `comms.bleEnable` (default on) | NimBLE-Arduino, **Nordic UART Service** (RX write / TX notify), MTU 247 -> chunk 244 B/notify, re-advertise otomatis saat disconnect |
| **WiFi** | `comms.wifiEnable` (default off) | STA + TCP server (`comms.wifiTcpPort`, default 6666), satu klien tuning app, auto-reconnect |

SSID/password/nama BLE disimpan di `EcuConfig.comms` (schema v3) - tidak ada
kredensial yang di-hardcode. LIVE_FRAME keluar di transport yang meminta
LIVE_START. Semua transport menaikkan StreamDecoder identik -> framing tahan
chunking BLE/TCP apa pun (diuji 200 trial fuzz pecahan 1-7 B + garbage).

## Struktur

```
include/ztune/     logika murni header-only (stdint.h) -> test native tanpa hardware
  Ztune.h          tipe umum, CRC32
  TriggerConfig.h  konfig trigger + validasi
  TriggerDecoder.h state machine Hunting->Synced->Lost, gap EMA, RPM, resync
  InterpTable.h    tabel bilinear 2D + kurva 1D (VE, advance, dwell)
  FuelMath.h       reqFuel (MegaSquirt) + pulse width + dead time
  EcuConfig.h      agregat seluruh config + validate() + safe-mode defaults
  SchedulerTypes.h M4: AngleMath + EvAction/Ev + EventQueue (cyclic, sorted)
  EngineScheduler.h M4: jadwal inj/ign per gigi (prediksi sudut linear)
  EventExecutor.h  M4.2: runDue + planAlarm (keputusan alarm hw, pure)
  RevLimiter.h     M4: hard fuel-cut + histeresis resume
  Quickshifter.h   M4: debounce sensor gear + fuel cut window per shift
  SensorMath.h     M4.1: TPS/MAP/NTC beta/Batt + EmaFilter + snapshot
  AccelEnrichment.h M4.1: deteksi tip-in TPS + ramp-down ASE
  ZtpTypes.h       ZTP: type/err/table id + payload struct packed
  ZtpFrame.h       ZTP: encode frame + stream decoder
  ZtpDispatcher.h  ZTP: dispatcher request->response + buildLiveFrame
src/main.cpp       glue ESP32-S3: ISR -> engine task Core 1; ZTP USB+BLE+WiFi; NVS
src/device/        transport real: ZtpBle.h (NimBLE NUS), ZtpWifi.h (STA+TCP)
sim/               ToothSimulator: bangkitkan sinyal gigi untuk test
test/native/       test suite (make test): trigger, fuel, config, ztp, scheduler
```

## Build & Test

```bash
make test          # test native: g++ -std=c++17, tanpa hardware
pio run -e ztune_esp32s3   # firmware ESP32-S3 (PlatformIO)
pio run -t upload && pio device monitor
```

## Status milestone

| M | Isi | Status |
|---|-----|--------|
| M0 | Skeleton + EcuConfig + NVS persist + safe-mode gate | DONE |
| M1 | Trigger decoder + tooth simulator + test native | DONE |
| M2 | FuelMath MegaSquirt + tabel 16x16 + advance/dwell + validasi | DONE |
| M3 | ZTP v1 frame+dispatcher+LIVE over USB (BLE/WiFi stub M3.1) | DONE |
| M3.1 | Transport BLE (NUS) + WiFi (STA/TCP) real + config Comms v3 | DONE |
| M4 | Scheduler injeksi/penyalaan + rev limiter + quickshifter | DONE |
| M4.1 | Sensor ADC TPS/MAP/CLT/IAT/Batt + WUE/ASE/IAT koreksi PW | DONE |
| M4.2 | Eksekusi event via hw timer (alarm one-shot ISR) + kurva dead-time vs batt | DONE |
| M5 | O2 narrowband + wideband, closed loop EGO, tabel target AFR, ZTP v2 | DONE |

## Catatan teknis decoder crank-only

- Referensi fase = gigi ke-0 tepat setelah gap; verifikasi pola tiap putaran
  (jumlah gigi sejak gap harus persis `totalTeeth - missingTeeth`) -> resync otomatis.
- Tanpa cam, fase siklus 4-tak tidak diketahui -> penyalaan wasted spark,
  injeksi dibagi 2 squirt per siklus (setengah muatan per 360 derajat).
- Keterbatasan inheren missing-tooth: hilangnya edge gigi yang tepat menempel
  gap bisa alias +1 gigi. Mitigasi penuh = cam sensor (opsi B) / plausibility check.
- Schema config sekarang **v6** (GPIO per silinder + `phaseOffsetDeg` +
  `injEndBeforeTdcDeg` + revLimit/quickShift + `comms` BLE/WiFi + `sensors`
  kalibrasi + kurva WUE/IAT/ASE + kurva dead-time vs batt + `ego` closed loop
  dgn tabel target AFR). Blob NVS schema lama otomatis ditolak -> kembali
  safe-mode.
- Keterbatasan framing magic-based: garbage yang kebetulan membentuk 'ZT'
  tepat sebelum frame bisa memakan magic frame itu (frame hilang). Host cukup
  retry; CRC tetap menjamin tidak ada frame palsu yang dieksekusi.
