// FluidNC/src/EDM/Feed/Hx711Calibration.h
#pragma once
#include <cstdint>

namespace EDM { namespace feed {

inline int32_t hx711SignExtend(uint32_t v24) {
    return (v24 & 0x800000u) ? int32_t(v24 | 0xFF000000u) : int32_t(v24 & 0x00FFFFFFu);
}

struct Hx711Calibration {
    int32_t offset       = 0;
    float   counts_per_N = 1000.0f;
    float toNewtons(int32_t raw) const { return (float(raw) - float(offset)) / counts_per_N; }
    Hx711Calibration withCalibration(int32_t newOffset, float newCountsPerN) const {
        return Hx711Calibration{ newOffset, newCountsPerN };
    }
};

}}  // namespace EDM::feed
