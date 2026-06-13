// FluidNC/src/EDM/Can/CanFrame.h
#pragma once
#include <cstdint>
#include <cstring>

namespace EDM {

// A CAN-FD frame. Standard 11-bit IDs only (per PSU contract).
struct CanFrame {
    uint16_t id   = 0;        // 11-bit standard ID
    uint8_t  len  = 0;        // valid bytes in data[] (0..64)
    bool     fd   = true;     // CAN-FD frame
    bool     brs  = true;     // bit-rate switch (data phase at 5 Mbps)
    uint8_t  data[64] = {};

    CanFrame() = default;
    CanFrame(uint16_t id_, uint8_t len_) : id(id_), len(len_) {}
};

}  // namespace EDM
