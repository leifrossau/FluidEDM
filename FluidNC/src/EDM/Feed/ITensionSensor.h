// FluidNC/src/EDM/Feed/ITensionSensor.h
//
// Minimal sensor interface so the WireFeed facade can be driven by either the
// on-target HX711 (Hx711.cpp) or a host stub. PURE header (no ESP32 deps), but
// the only concrete implementation today is ESP32-coupled, so it is not added
// to the native test build.
#pragma once
#include <cstdint>

namespace EDM { namespace feed {

class ITensionSensor {
public:
    virtual ~ITensionSensor() = default;
    // Non-blocking: true when a fresh conversion is available to read.
    virtual bool    dataReady() const = 0;
    // Reads one 24-bit signed sample (sign-extended). Assumes dataReady().
    virtual int32_t readRaw()         = 0;
};

}}  // namespace EDM::feed
