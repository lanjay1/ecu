// ZTP v1 - encode/decode frame + decoder aliran byte.
// Frame: [0x5A][0x54][ver][type][len u16 LE][payload][crc32 u32 LE]
// CRC32 mencakup ver..payload (magic di luar) -> magic murni jadi titik resync.
// StreamDecoder tahan: frame terpotong antar-feed, byte sampah, CRC korup
// (buang frame, resync ke magic berikutnya).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <functional>
#include "Ztune.h"
#include "ZtpTypes.h"

namespace ztune {
namespace ztp {

// Encode frame lengkap. Return total byte, 0 jika len/cap tidak valid.
inline size_t encodeFrame(uint8_t* out, size_t cap, uint8_t type,
                          const void* payload, size_t len) {
    if (len > MAX_PAYLOAD) return 0;
    size_t total = HEADER_SIZE + len + CRC_SIZE;
    if (cap < total) return 0;
    out[0] = MAGIC0;
    out[1] = MAGIC1;
    out[2] = PROTO_VER;
    out[3] = type;
    out[4] = (uint8_t)(len & 0xFF);
    out[5] = (uint8_t)(len >> 8);
    if (len) memcpy(out + HEADER_SIZE, payload, len);
    uint32_t crc = crc32(out + 2, HEADER_SIZE - 2 + len);   // ver..payload
    memcpy(out + HEADER_SIZE + len, &crc, 4);               // LE host
    return total;
}

// Decoder aliran byte -> frame. Callback dipanggil sinkron di dalam feed().
class StreamDecoder {
public:
    void reset() {
        state_ = St::Hunt;
        bufN_ = 0;
        needLen_ = 0;
        expectM1_ = false;
    }

    void feed(const uint8_t* data, size_t n,
              const std::function<void(uint8_t, const uint8_t*, size_t)>& onFrame,
              const std::function<void(uint8_t)>& onError) {
        for (size_t i = 0; i < n; ++i) step(data[i], onFrame, onError);
    }

private:
    enum class St : uint8_t { Hunt, Collect };

    void step(uint8_t b,
              const std::function<void(uint8_t, const uint8_t*, size_t)>& onFrame,
              const std::function<void(uint8_t)>& onError) {
        if (state_ == St::Hunt) {
            if (b == MAGIC0) {
                buf_[0] = b;
                bufN_ = 1;
                expectM1_ = true;
                state_ = St::Collect;
            }
            return;
        }
        // Collect: byte kedua harus MAGIC1
        if (expectM1_) {
            if (b == MAGIC1) {
                buf_[1] = b;
                bufN_ = 2;
                expectM1_ = false;
                return;
            }
            // 'Z' tunggal palsu: byte ini bisa jadi magic baru
            if (b == MAGIC0) { buf_[0] = MAGIC0; bufN_ = 1; expectM1_ = true; }
            else { state_ = St::Hunt; bufN_ = 0; }
            return;
        }
        if (bufN_ >= sizeof(buf_)) {
            reset();
            if (onError) onError((uint8_t)Err::BAD_PAYLOAD);
            return;
        }
        buf_[bufN_++] = b;

        if (bufN_ == HEADER_SIZE) {
            if (buf_[2] != PROTO_VER) {
                reset();
                if (onError) onError((uint8_t)Err::BAD_VERSION);
                return;
            }
            needLen_ = (size_t)buf_[4] | ((size_t)buf_[5] << 8);
            if (needLen_ > MAX_PAYLOAD) {
                reset();
                if (onError) onError((uint8_t)Err::BAD_PAYLOAD);
                return;
            }
        }
        size_t need = HEADER_SIZE + needLen_ + CRC_SIZE;
        if (bufN_ == need) {
            uint32_t crcStored;
            memcpy(&crcStored, buf_ + HEADER_SIZE + needLen_, 4);
            uint32_t crcCalc = crc32(buf_ + 2, HEADER_SIZE - 2 + needLen_);
            if (crcStored == crcCalc) {
                if (onFrame) onFrame(buf_[3], buf_ + HEADER_SIZE, needLen_);
            } else {
                if (onError) onError((uint8_t)Err::BAD_CRC);
            }
            reset();
        }
    }

    St state_ = St::Hunt;
    bool expectM1_ = false;
    size_t bufN_ = 0;
    size_t needLen_ = 0;
    uint8_t buf_[HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE];
};

} // namespace ztp
} // namespace ztune
