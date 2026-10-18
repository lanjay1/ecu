// Transport WiFi ZTP v1 (M3.1) - WiFi STA (client) + TCP server.
//
// SSID/password/port dari EcuConfig.comms (no-hardcode). Non-blocking:
// pump() dipanggil dari loop() Core 0 -> accept client, baca byte, feed ke
// StreamDecoder milik transport ini; response keluar via send().
// Satu client tuning app pada satu waktu (klien baru menggantikan lama).
//
// CATATAN: handler frame jalan di konteks loop() -> race g_cfg sama seperti
// USB (TODO(M5) snapshot config atomik).
#pragma once
#include <WiFi.h>
#include "ztune/ZtpFrame.h"

namespace ztune {
namespace ztp {

class ZtpWifiTransport {
public:
    using FrameCb = std::function<void(uint8_t, const uint8_t*, size_t)>;
    using ErrCb   = std::function<void(uint8_t)>;

    void begin(StreamDecoder& dec, const char* ssid, const char* pass,
               uint16_t port, FrameCb onFrame, ErrCb onErr) {
        dec_ = &dec;
        onFrame_ = std::move(onFrame);
        onErr_ = std::move(onErr);
        port_ = port;

        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(true);
        WiFi.begin(ssid, pass);
        srv_ = new WiFiServer(port_);
        srv_->begin();
    }

    // Dipanggil periodik dari loop(): accept + RX pump.
    void pump() {
        if (!srv_) return;
        if (!cli_ || !cli_.connected()) {
            WiFiClient c = srv_->available();
            if (c) cli_ = c;                     // klien baru menggantikan lama
        }
        if (!cli_ || !cli_.connected()) return;
        uint8_t buf[64];
        while (cli_.available()) {
            int n = cli_.read(buf, sizeof(buf));
            if (n <= 0) break;
            dec_->feed(buf, (size_t)n, onFrame_, onErr_);
        }
    }

    bool connected() const {
        return WiFi.status() == WL_CONNECTED && cli_ && cli_.connected();
    }

    void send(const uint8_t* data, size_t len) {
        if (connected() && len) cli_.write(data, len);
    }

private:
    StreamDecoder* dec_ = nullptr;
    FrameCb onFrame_;
    ErrCb onErr_;
    WiFiServer* srv_ = nullptr;
    WiFiClient cli_;
    uint16_t port_ = 6666;
};

} // namespace ztp
} // namespace ztune
