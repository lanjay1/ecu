// Test native ZTP v2: frame codec, stream decoder (robustness), dispatcher
// (HELLO/PING/CONFIG/TABLE/LIVE/ERROR), dan builder LIVE_FRAME.
// M5: LiveFrameV2 (field O2/EGO), tabel AFR_TARGET, caps per comms.
#include "ztune/ZtpFrame.h"
#include "ztune/ZtpDispatcher.h"
#include "test_util.h"
#include <vector>
#include <cstring>
#include <utility>

using namespace ztune;
using namespace ztune::ztp;

static std::vector<uint8_t> enc(uint8_t type, const void* p, size_t n) {
    std::vector<uint8_t> v(HEADER_SIZE + n + CRC_SIZE);
    size_t k = encodeFrame(v.data(), v.size(), type, p, n);
    v.resize(k);
    return v;
}

// Kumpulkan hasil stream decoder
struct Rx {
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> frames;
    std::vector<uint8_t> errors;
    void feed(StreamDecoder& d, const uint8_t* b, size_t n) {
        d.feed(b, n,
            [this](uint8_t t, const uint8_t* p, size_t l) {
                frames.push_back({t, std::vector<uint8_t>(p, p + l)});
            },
            [this](uint8_t e) { errors.push_back(e); });
    }
    void feed(StreamDecoder& d, const std::vector<uint8_t>& v) { feed(d, v.data(), v.size()); }
};

static ZtpCtx makeCtx(EcuConfig& cfg, bool& live, int& pc) {
    ZtpCtx c;
    c.cfg = &cfg;
    c.liveActive = &live;
    c.liveRateMs = 100;
    c.onPersist = [&](const EcuConfig&) { pc++; };
    return c;
}

int main() {
    // ===== frame codec & stream decoder =====
    { // 1. Round trip + dua frame dalam satu feed
        StreamDecoder d; Rx r;
        auto a = enc((uint8_t)Type::PING, "hai", 3);
        auto b = enc((uint8_t)Type::PING, nullptr, 0);
        std::vector<uint8_t> s = a;
        s.insert(s.end(), b.begin(), b.end());
        r.feed(d, s);
        CHECK(r.frames.size() == 2);
        CHECK(r.frames[0].first == (uint8_t)Type::PING);
        CHECK(r.frames[0].second.size() == 3);
        CHECK(r.errors.empty());
    }
    { // 2. Frame terpotong antar-feed
        StreamDecoder d; Rx r;
        auto a = enc((uint8_t)Type::HELLO, "abc", 3);
        r.feed(d, a.data(), 4);        CHECK(r.frames.empty());
        r.feed(d, a.data() + 4, 2);    CHECK(r.frames.empty());
        r.feed(d, a.data() + 6, a.size() - 6);
        CHECK(r.frames.size() == 1);
    }
    { // 3. Byte sampah sebelum frame -> diskip otomatis
        StreamDecoder d; Rx r;
        std::vector<uint8_t> s = {0x00, 0x12, 0x5A, 0x99, 0x7F};
        auto a = enc((uint8_t)Type::PING, nullptr, 0);
        s.insert(s.end(), a.begin(), a.end());
        r.feed(d, s);
        CHECK(r.frames.size() == 1);
        CHECK(r.errors.empty());
    }
    { // 4. CRC korup -> error, frame berikutnya tetap terdekode
        StreamDecoder d; Rx r;
        auto a = enc((uint8_t)Type::PING, nullptr, 0);
        auto bad = a;
        bad[bad.size() - 1] ^= 0xFF;
        auto b = enc((uint8_t)Type::PING, "ok", 2);
        std::vector<uint8_t> s = bad;
        s.insert(s.end(), b.begin(), b.end());
        r.feed(d, s);
        CHECK(r.errors.size() == 1 && r.errors[0] == (uint8_t)Err::BAD_CRC);
        CHECK(r.frames.size() == 1);
    }
    { // 5. Versi protokol beda -> error
        StreamDecoder d; Rx r;
        auto a = enc((uint8_t)Type::PING, nullptr, 0);
        a[2] = 0x09;
        r.feed(d, a);
        CHECK(r.errors.size() == 1 && r.errors[0] == (uint8_t)Err::BAD_VERSION);
    }
    { // 6. Len melebihi MAX -> error, resync ke frame berikutnya
        StreamDecoder d; Rx r;
        std::vector<uint8_t> bad = {0x5A, 0x54, 0x02, 0x0A, 0xFF, 0xFF};
        auto b = enc((uint8_t)Type::PING, nullptr, 0);
        std::vector<uint8_t> s = bad;
        s.insert(s.end(), b.begin(), b.end());
        r.feed(d, s);
        CHECK(r.errors.size() == 1 && r.errors[0] == (uint8_t)Err::BAD_PAYLOAD);
        CHECK(r.frames.size() == 1);
    }
    { // 7. Invarian: blob EcuConfig muat dalam MAX_PAYLOAD
        CHECK(sizeof(EcuConfig) <= MAX_PAYLOAD);
        CHECK(sizeof(EcuConfig) > 4096);   // schema v6 tumbuh dgn EgoConfig
        CHECK(sizeof(LiveFrameV2) == 55);  // 42 (v1) + 3 float + 1 u8, packed
    }

    // ===== dispatcher =====
    { // 8. PING -> PONG echo
        EcuConfig cfg; bool live = false; int pc = 0;
        ZtpCtx ctx = makeCtx(cfg, live, pc);
        uint8_t out[512];
        const char* msg = "halo";
        size_t n = dispatch(ctx, (uint8_t)Type::PING, (const uint8_t*)msg, 4, out, sizeof(out));
        CHECK(n == HEADER_SIZE + 4 + CRC_SIZE);
        CHECK(out[3] == (uint8_t)Type::PONG);
        CHECK(memcmp(out + HEADER_SIZE, msg, 4) == 0);
    }
    { // 9. HELLO: proto + schema + fw version
        EcuConfig cfg; bool live = false; int pc = 0;
        ZtpCtx ctx = makeCtx(cfg, live, pc);
        uint8_t out[256];
        size_t n = dispatch(ctx, (uint8_t)Type::HELLO, nullptr, 0, out, sizeof(out));
        CHECK(n == HEADER_SIZE + sizeof(HelloInfo) + CRC_SIZE);
        HelloInfo h;
        memcpy(&h, out + HEADER_SIZE, sizeof(h));
        CHECK(h.protoVer == PROTO_VER);
        CHECK(h.schemaVersion == ZTUNE_SCHEMA_VERSION);
        CHECK(strncmp(h.fwVersion, ZTUNE_FW_VERSION, sizeof(h.fwVersion)) == 0);
    }
    { // 10. CONFIG_GET -> blob utuh; CONFIG_SET valid -> applied + persist dipanggil
        EcuConfig cfg = EcuConfig::defaults(); bool live = false; int pc = 0;
        ZtpCtx ctx = makeCtx(cfg, live, pc);
        uint8_t out[HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE];

        size_t n = dispatch(ctx, (uint8_t)Type::CONFIG_GET, nullptr, 0, out, sizeof(out));
        CHECK(n == HEADER_SIZE + sizeof(EcuConfig) + CRC_SIZE);
        EcuConfig got;
        memcpy(&got, out + HEADER_SIZE, sizeof(got));
        CHECK(got.trigger.totalTeeth == 12);

        got.trigger.totalTeeth = 13;   // tetap valid
        size_t m = dispatch(ctx, (uint8_t)Type::CONFIG_SET, (uint8_t*)&got,
                            sizeof(got), out, sizeof(out));
        CHECK(m == HEADER_SIZE + 4 + CRC_SIZE);   // ACK: echo schemaVersion
        CHECK(out[3] == (uint8_t)Type::CONFIG_SET);
        CHECK(pc == 1);
        CHECK(cfg.trigger.totalTeeth == 13);
    }
    { // 11. CONFIG_SET invalid -> ERROR VALIDATION, config tidak berubah
        EcuConfig cfg = EcuConfig::defaults(); bool live = false; int pc = 0;
        ZtpCtx ctx = makeCtx(cfg, live, pc);
        uint8_t out[HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE];
        EcuConfig bad = EcuConfig::defaults();
        bad.engine.cylinders = 0;
        size_t m = dispatch(ctx, (uint8_t)Type::CONFIG_SET, (uint8_t*)&bad,
                            sizeof(bad), out, sizeof(out));
        CHECK(m > 0);
        CHECK(out[3] == (uint8_t)Type::ERROR);
        CHECK(out[HEADER_SIZE] == (uint8_t)Err::VALIDATION);
        CHECK(out[HEADER_SIZE + 1] == (uint8_t)Type::CONFIG_SET);
        CHECK(pc == 0);
        CHECK(cfg.engine.cylinders == 1);
    }
    { // 12. TABLE_WRITE VE -> nilai berubah; TABLE_READ baca balik
        EcuConfig cfg = EcuConfig::defaults(); bool live = false; int pc = 0;
        ZtpCtx ctx = makeCtx(cfg, live, pc);
        uint8_t out[HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE];

        std::vector<uint8_t> w(1 + 288 * 4);
        w[0] = (uint8_t)TableId::VE;
        float* f = (float*)(w.data() + 1);
        memcpy(f, cfg.fuel.ve.xAxis, 16 * 4);
        memcpy(f + 16, cfg.fuel.ve.yAxis, 16 * 4);
        memcpy(f + 32, cfg.fuel.ve.values, 256 * 4);
        f[32 + 5 * 16 + 3] = 77.5f;   // values[5][3]

        size_t m = dispatch(ctx, (uint8_t)Type::TABLE_WRITE, w.data(), w.size(),
                            out, sizeof(out));
        CHECK(m == HEADER_SIZE + 1 + CRC_SIZE);
        CHECK(out[3] == (uint8_t)Type::TABLE_WRITE);
        CHECK(pc == 1);
        CHECK_NEAR(cfg.fuel.ve.values[5][3], 77.5, 1e-6);

        uint8_t rid = (uint8_t)TableId::VE;
        size_t n = dispatch(ctx, (uint8_t)Type::TABLE_READ, &rid, 1, out, sizeof(out));
        CHECK(n == HEADER_SIZE + 1 + 288 * 4 + CRC_SIZE);
        const float* g = (const float*)(out + HEADER_SIZE + 1);
        CHECK_NEAR(g[32 + 5 * 16 + 3], 77.5, 1e-3);
    }
    { // 13. TABLE_WRITE size salah / id salah -> ERROR
        EcuConfig cfg; bool live = false; int pc = 0;
        ZtpCtx ctx = makeCtx(cfg, live, pc);
        uint8_t out[256];

        uint8_t w1[4] = {(uint8_t)TableId::VE, 0, 0, 0};
        size_t a = dispatch(ctx, (uint8_t)Type::TABLE_WRITE, w1, 4, out, sizeof(out));
        CHECK(a > 0 && out[3] == (uint8_t)Type::ERROR);
        CHECK(out[HEADER_SIZE] == (uint8_t)Err::BAD_PAYLOAD);

        uint8_t w2[1 + 32 * 4] = {9};   // id 9 tak dikenal
        size_t b = dispatch(ctx, (uint8_t)Type::TABLE_WRITE, w2, sizeof(w2), out, sizeof(out));
        CHECK(b > 0 && out[3] == (uint8_t)Type::ERROR);
        CHECK(out[HEADER_SIZE] == (uint8_t)Err::BAD_TABLE_ID);
    }
    { // 14. LIVE_START/STOP mengubah flag + rate
        EcuConfig cfg; bool live = false; int pc = 0;
        ZtpCtx ctx = makeCtx(cfg, live, pc);
        uint8_t out[128];
        uint8_t rate[2] = {0x64, 0x00};   // 100 ms LE
        size_t n = dispatch(ctx, (uint8_t)Type::LIVE_START, rate, 2, out, sizeof(out));
        CHECK(n == HEADER_SIZE + 2 + CRC_SIZE);
        CHECK(live);
        CHECK(ctx.liveRateMs == 100);
        dispatch(ctx, (uint8_t)Type::LIVE_STOP, nullptr, 0, out, sizeof(out));
        CHECK(!live);
    }
    { // 15. Type tidak dikenal -> ERROR BAD_TYPE
        EcuConfig cfg; bool live = false; int pc = 0;
        ZtpCtx ctx = makeCtx(cfg, live, pc);
        uint8_t out[128];
        size_t n = dispatch(ctx, 0x55, nullptr, 0, out, sizeof(out));
        CHECK(n > 0);
        CHECK(out[3] == (uint8_t)Type::ERROR);
        CHECK(out[HEADER_SIZE] == (uint8_t)Err::BAD_TYPE);
    }
    { // 16. buildLiveFrame sanity (M4.1: snapshot sensor)
        EcuConfig cfg = EcuConfig::defaults();
        TriggerDecoder dec(cfg.trigger);
        LiveSensors sen;
        sen.tpsPct = 55.0f;
        LiveFrameV2 lf = buildLiveFrame(cfg, dec, 1234, sen, 3.2f, 12.5f);
        CHECK(lf.tMs == 1234);
        CHECK(lf.rpm == 0.0f);
        CHECK(lf.trigState == (uint8_t)TriggerState::Hunting);
        CHECK_NEAR(lf.tpsPct, 55.0, 1e-6);
        CHECK_NEAR(lf.pwMs, 3.2, 1e-6);
        CHECK(lf.mapKpa == 0.0f);
        CHECK_NEAR(lf.advanceDeg, 12.5, 1e-6);
        // Load source MAP -> VE lookup pakai mapKpa
        cfg.fuel.loadSource = LoadSource::MAP;
        LiveSensors sen2; sen2.mapKpa = 95.0f;
        LiveFrameV2 lf2 = buildLiveFrame(cfg, dec, 1234, sen2, 0.0f, 0.0f);
        CHECK_NEAR(lf2.vePct, 40.0, 1e-6);   // VE default 40 di sumbu apa pun
        CHECK_NEAR(lf2.mapKpa, 95.0, 1e-6);
        // M5: field O2/EGO - default EgoState kosong -> nol & tidak aktif
        CHECK(lf.afrMeasured == 0.0f && lf.afrTarget == 0.0f);
        CHECK(lf.egoPct == 0.0f && lf.egoActive == 0);
        // Dengan state EGO terisi
        EgoState es;
        es.corrPct = -3.5f; es.active = true;
        es.targetAfr = 14.7f; es.measuredAfr = 15.2f;
        LiveFrameV2 lf3 = buildLiveFrame(cfg, dec, 1234, sen, 3.2f, 12.5f, es);
        CHECK_NEAR(lf3.egoPct, -3.5, 1e-6);
        CHECK(lf3.egoActive == 1);
        CHECK_NEAR(lf3.afrTarget, 14.7, 1e-6);
        CHECK_NEAR(lf3.afrMeasured, 15.2, 1e-6);
    }
    { // 16b. M5: HELLO caps mengikuti comms + TABLE AFR_TARGET round-trip
        EcuConfig cfg = EcuConfig::defaults(); bool live = false; int pc = 0;
        ZtpCtx ctx = makeCtx(cfg, live, pc);
        uint8_t out[HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE];
        // Default: BLE on, WiFi off -> caps = bit0|bit1
        size_t n = dispatch(ctx, (uint8_t)Type::HELLO, nullptr, 0, out, sizeof(out));
        HelloInfo h;
        memcpy(&h, out + HEADER_SIZE, sizeof(h));
        CHECK((h.caps & 0x1) && (h.caps & 0x2) && !(h.caps & 0x4));
        cfg.comms.wifiEnable = true;
        cfg.comms.wifiSsid[0] = 'x';   // ssid wajib isi agar config tetap valid
        n = dispatch(ctx, (uint8_t)Type::HELLO, nullptr, 0, out, sizeof(out));
        memcpy(&h, out + HEADER_SIZE, sizeof(h));
        CHECK(h.caps == 0x7);

        // TABLE_READ AFR_TARGET: axis + nilai default terbawa
        uint8_t req = (uint8_t)TableId::AFR_TARGET;
        n = dispatch(ctx, (uint8_t)Type::TABLE_READ, &req, 1, out, sizeof(out));
        CHECK(n == HEADER_SIZE + 1 + 288 * 4 + CRC_SIZE);
        CHECK(out[HEADER_SIZE] == (uint8_t)TableId::AFR_TARGET);
        const float* f = (const float*)(out + HEADER_SIZE + 1);
        CHECK_NEAR(f[1], 14500.0f / 15.0f + 500.0f, 1e-3);   // axisX[1]
        CHECK_NEAR(f[32], 14.7f, 1e-6);                      // values[0][0]

        // TABLE_WRITE AFR_TARGET: ubah nilai -> terbaca balik + persist
        std::vector<uint8_t> wr(1 + 288 * 4);
        wr[0] = (uint8_t)TableId::AFR_TARGET;
        float* g = (float*)(wr.data() + 1);
        memcpy(g, f, 288 * 4);
        g[32 + 5 * 16 + 3] = 13.9f;                          // values[5][3]
        n = dispatch(ctx, (uint8_t)Type::TABLE_WRITE, wr.data(), wr.size(),
                     out, sizeof(out));
        CHECK(n > 0);
        CHECK(out[3] == (uint8_t)Type::TABLE_WRITE);   // sukses: echo type
        CHECK(pc == 1);
        CHECK_NEAR(cfg.ego.afrTarget.values[5][3], 13.9f, 1e-6);
        // config tetap valid setelah tabel ditulis
        char emsg[96];
        CHECK(cfg.validate(emsg, sizeof(emsg)));
    }
    { // 17. End-to-end: stream request -> decode -> dispatch -> decode response
        EcuConfig cfg = EcuConfig::defaults(); bool live = false; int pc = 0;
        ZtpCtx ctx = makeCtx(cfg, live, pc);
        uint8_t rsp[HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE];

        StreamDecoder drx; Rx rr;
        rr.feed(drx, enc((uint8_t)Type::HELLO, nullptr, 0));
        CHECK(rr.frames.size() == 1);
        size_t n = dispatch(ctx, rr.frames[0].first, rr.frames[0].second.data(),
                            rr.frames[0].second.size(), rsp, sizeof(rsp));
        CHECK(n > 0);

        StreamDecoder ds; Rx rs;
        rs.feed(ds, rsp, n);
        CHECK(rs.frames.size() == 1);
        CHECK(rs.frames[0].first == (uint8_t)Type::HELLO);
        HelloInfo h;
        memcpy(&h, rs.frames[0].second.data(), sizeof(h));
        CHECK(h.protoVer == PROTO_VER);
    }
    { // 18. M3.1: fuzz chunked split - frame dipecah di titik acak (simulasi
      //     BLE notify / TCP segment / USB chunking). PRNG deterministik.
        uint32_t seed = 0xC0FFEE01;   // xorshift32 deterministik
        auto rnd = [&seed]() {
            seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
            return seed;
        };
        auto a = enc((uint8_t)Type::PING, "chunky-1234567890", 18);
        auto b = enc((uint8_t)Type::HELLO, nullptr, 0);
        EcuConfig cfgTmp = EcuConfig::defaults();
        auto c = enc((uint8_t)Type::CONFIG_GET, &cfgTmp, sizeof(cfgTmp));

        for (int trial = 0; trial < 200; ++trial) {
            std::vector<uint8_t> s = a;
            s.insert(s.end(), b.begin(), b.end());
            s.insert(s.end(), c.begin(), c.end());

            // Garbage IDLE-LINE: bertumpuk di depan stream, byte "tumpul"
            // (0x01..0x50 - bukan magic/parsial magic). Garbage yang membentuk
            // 'ZT' bisa memakan magic frame pertama (keterbatasan desain
            // framing magic-based, host retry) - bukan skenario yang diuji.
            size_t garbage = rnd() % 5;
            for (size_t g = 0; g < garbage; ++g)
                s.insert(s.begin(), (uint8_t)(0x01 + rnd() % 0x50));

            StreamDecoder d; Rx r;
            size_t off = 0;
            while (off < s.size()) {
                size_t chunk = 1 + (rnd() % 7);   // potongan 1..7 byte
                if (chunk > s.size() - off) chunk = s.size() - off;
                r.feed(d, s.data() + off, chunk);
                off += chunk;
            }
            // 3 frame harus terbaca utuh (garbage diabaikan; garbage yang
            // tidak sengaja membentuk header valid -> boleh menambah error
            // path, jadi cek minimal 3 frame PING/HELLO/CONFIG_GET masuk)
            int nPing = 0, nHello = 0, nCfg = 0;
            for (auto& f : r.frames) {
                if (f.first == (uint8_t)Type::PING) nPing++;
                if (f.first == (uint8_t)Type::HELLO) nHello++;
                if (f.first == (uint8_t)Type::CONFIG_GET) nCfg++;
            }
            CHECK(nPing == 1);
            CHECK(nHello == 1);
            CHECK(nCfg == 1);
            // HELLO payload kosong utuh (tidak tercampur garbage)
            bool helloEmpty = false;
            for (auto& f : r.frames)
                if (f.first == (uint8_t)Type::HELLO && f.second.empty()) helloEmpty = true;
            CHECK(helloEmpty);
        }
    }
    { // 19. M3.1: dua frame back-to-back di satu chunk TCP tanpa jeda
        StreamDecoder d; Rx r;
        auto a = enc((uint8_t)Type::PING, "aa", 2);
        auto b = enc((uint8_t)Type::PING, "bb", 2);
        std::vector<uint8_t> s = a;
        s.insert(s.end(), b.begin(), b.end());
        r.feed(d, s);            // satu chunk penuh (kasus TCP aggregate)
        CHECK(r.frames.size() == 2);
        CHECK(r.frames[0].second.size() == 2 && r.frames[1].second.size() == 2);
        CHECK(r.frames[0].second[0] == 'a' && r.frames[1].second[0] == 'b');
        CHECK(r.errors.empty());
    }
    return report("ztp");
}
