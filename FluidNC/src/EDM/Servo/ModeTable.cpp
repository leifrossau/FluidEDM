// FluidNC/src/EDM/Servo/ModeTable.cpp
#include "EDM/Servo/ModeTable.h"

namespace EDM { namespace servo {

EdmMode ModeTable::modeForSword(uint16_t s) const {
    if (s == 0)             return EdmMode::Idle;
    if (s <= ignite_max)    return EdmMode::Ignite;
    if (s <= finish_max)    return EdmMode::Finish;
    return EdmMode::Rough;
}

const ModeParams& ModeTable::params(EdmMode m) const {
    switch (m) {
        case EdmMode::Rough:  return rough;
        case EdmMode::Finish: return finish;
        default:              return ignite;
    }
}

EDM::psu::SetModeBounds ModeTable::build(EdmMode m, uint16_t s_word, uint16_t seq) const {
    const ModeParams& p = params(m);
    EDM::psu::SetModeBounds b;
    b.mode_id            = p.mode_id;
    b.seq                = seq;
    b.freq_max_kHz       = p.freq_max_kHz;
    b.on_time_max_ns     = p.on_time_max_ns;
    b.off_time_min_ns    = p.off_time_min_ns;
    b.peak_I_setpoint_dA = p.peak_I_setpoint_dA;
    b.peak_I_limit_hw_dA = p.peak_I_limit_hw_dA;
    b.polarity           = p.polarity;
    b.flags              = p.flags;
    b.gap_V_arc_mV       = p.gap_V_arc_mV;
    b.gap_V_short_mV     = p.gap_V_short_mV;
    b.ignition_timeout_us= p.ignition_timeout_us;

    if (adaptive_setpoint && m != EdmMode::Idle) {
        uint16_t lo = 1, hi = 1000;
        if (m == EdmMode::Ignite)      { lo = 1;              hi = ignite_max; }
        else if (m == EdmMode::Finish) { lo = ignite_max + 1; hi = finish_max; }
        else                           { lo = finish_max + 1; hi = 1000; }
        if (s_word < lo) s_word = lo;
        if (s_word > hi) s_word = hi;
        float t = (hi > lo) ? float(s_word - lo) / float(hi - lo) : 0.0f;
        uint32_t base = p.peak_I_setpoint_dA;
        uint32_t scaled = uint32_t(base * (0.5f + 0.5f * t));
        if (scaled > p.peak_I_limit_hw_dA) scaled = p.peak_I_limit_hw_dA;
        b.peak_I_setpoint_dA = uint16_t(scaled);
    }
    return b;
}

EdmMode ModeTable::lowerEnergy(EdmMode m) const {
    switch (m) {
        case EdmMode::Rough:  return EdmMode::Finish;
        case EdmMode::Finish: return EdmMode::Ignite;
        default:              return EdmMode::Ignite;
    }
}

}}  // namespace EDM::servo
