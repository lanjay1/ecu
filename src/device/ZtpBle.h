// Transport BLE ZTP v1 (M3.1) - implementasi real via NimBLE-Arduino.
//
// Desain: Nordic UART Service (NUS) - standar de facto yang didukung semua
// library tuning app (BLE UART client):
//   Service : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//   RX (host->fw, WRITE)      : 6E400002-...
//   TX (fw->host, NOTIFY)     : 6E400003-...
//
// Byte dari RX langsung di-feed ke StreamDecoder (framing magic-based tahan
// chunking apa pun); response dikirim per-notify dengan chunk = MTU-3
// (negosiasi MTU 247 -> 244 B/notify, fallback aman 20 B bila client tak
// renegotiate). Kontrak begin() diperluas: transport yang men-feed decoder,
// frame hasil decode diteruskan ke callback milik main.cpp.
//
// CATATAN: onWrite NimBLE jalan di task BLE stack -> CONFIG_SET memodifikasi
// g_cfg lintas core (jendela sempit). Sama seperti USB: TODO(M5) snapshot/
// versioning config atomik.
#pragma once
#include <NimBLEDevice.h>
#include "ztune/ZtpFrame.h"

namespace ztune {
namespace ztp {

class ZtpBleTransport {
public:
    using FrameCb = std::function<void(uint8_t, const uint8_t*, size_t)>;
    using ErrCb   = std::function<void(uint8_t)>;
    static constexpr uint16_t MAX_CHUNK = 244;   // MTU 247 - 3

    // Panggil setName() SEBELUM begin() (nama dari EcuConfig.comms).
    void setName(const char* n) { name_ = n; }

    void begin(StreamDecoder& dec, FrameCb onFrame, ErrCb onErr) {
        dec_ = &dec;
        onFrame_ = std::move(onFrame);
        onErr_ = std::move(onErr);
        s_inst = this;

        NimBLEDevice::init(name_ ? name_ : "ZtuneECU");
        NimBLEDevice::setMTU(247);

        NimBLEServer* srv = NimBLEDevice::createServer();
        srv->setCallbacks(new SrvCb());

        NimBLEService* svc = srv->createService(NUS_SVC);
        txChar_ = svc->createCharacteristic(NUS_TX, NIMBLE_PROPERTY::NOTIFY);
        NimBLECharacteristic* rx = svc->createCharacteristic(
            NUS_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
        rx->setCallbacks(new RxCb());
        svc->start();

        NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
        adv->addServiceUUID(NUS_SVC);
        adv->start();     // auto-restart saat disconnect ditangani SrvCb
    }

    bool connected() const { return conn_; }

    // Kirim stream (satu atau beberapa frame ZTP) dipecah per MTU-3.
    void send(const uint8_t* data, size_t len) {
        if (!conn_ || !txChar_) return;
        uint16_t chunk = (mtu_ > 3) ? (uint16_t)(mtu_ - 3) : 20;
        if (chunk > MAX_CHUNK) chunk = MAX_CHUNK;
        size_t off = 0;
        while (off < len) {
            size_t n = (len - off > (size_t)chunk) ? (size_t)chunk : (len - off);
            txChar_->setValue(const_cast<uint8_t*>(data + off), n);
            txChar_->notify();
            off += n;
        }
    }

private:
    // UUID NUS standar (string -> NimBLEUUID)
    static constexpr const char* NUS_SVC = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
    static constexpr const char* NUS_RX  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
    static constexpr const char* NUS_TX  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

    class SrvCb : public NimBLEServerCallbacks {
        void onConnect(NimBLEServer*) override { s_inst->conn_ = true; }
        void onDisconnect(NimBLEServer*) override {
            s_inst->conn_ = false;
            s_inst->mtu_ = 23;                       // kembali ke MTU default
            NimBLEDevice::getAdvertising()->start(); // siap koneksi berikutnya
        }
        void onMTUChange(uint16_t mtu, ble_gap_conn_desc*) override {
            s_inst->mtu_ = mtu;                      // MTU negosiasi client
        }
    };

    class RxCb : public NimBLECharacteristicCallbacks {
        void onWrite(NimBLECharacteristic* ch) override {
            if (!s_inst || !s_inst->dec_) return;
            std::string v = ch->getValue();
            if (v.empty()) return;
            s_inst->dec_->feed((const uint8_t*)v.data(), v.size(),
                               s_inst->onFrame_, s_inst->onErr_);
        }
    };

    StreamDecoder* dec_ = nullptr;
    FrameCb onFrame_;
    ErrCb onErr_;
    NimBLECharacteristic* txChar_ = nullptr;
    const char* name_ = "ZtuneECU";
    volatile bool conn_ = false;
    volatile uint16_t mtu_ = 23;
    inline static ZtpBleTransport* s_inst = nullptr;   // satu instance saja
};

} // namespace ztp
} // namespace ztune
