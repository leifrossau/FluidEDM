// FluidNC/src/EDM/Servo/ModeTable.h
#pragma once
#include <cstdint>
#include "EDM/Psu/Protocol.h"

namespace EDM { namespace servo {

enum class EdmMode : uint8_t { Idle = 0, Ignite = 3, Finish = 2, Rough = 1 };

struct ModeParams {
    uint8_t  mode_id            = 0;
    uint16_t freq_max_kHz       = 0;
    uint16_t on_time_max_ns     = 0;
    uint16_t off_time_min_ns    = 0;
    uint16_t peak_I_setpoint_dA = 0;
    uint16_t peak_I_limit_hw_dA = 0;
    uint8_t  polarity           = 0;
    uint16_t flags              = 0;
    uint16_t gap_V_arc_mV       = 0;
    uint16_t gap_V_short_mV     = 0;
    uint16_t ignition_timeout_us = 0;
    uint16_t ign_delay_nom_ns   = 10000;
};

class ModeTable {
public:
    ModeParams rough, finish, ignite;
    uint16_t   ignite_max = 99;
    uint16_t   finish_max = 499;
    bool       adaptive_setpoint = false;

    EdmMode    modeForSword(uint16_t s) const;
    const ModeParams& params(EdmMode m) const;
    EDM::psu::SetModeBounds build(EdmMode m, uint16_t s_word, uint16_t seq) const;
    EdmMode    lowerEnergy(EdmMode m) const;
};

}}  // namespace EDM::servo
