// FluidNC/src/EDM/Psu/Protocol.cpp
#include "EDM/Psu/Protocol.h"
#include "EDM/Psu/Endian.h"
#include <cstring>

namespace EDM { namespace psu {

CanFrame encodeSetModeBounds(const SetModeBounds& m) {
    CanFrame f(ID_SET_MODE_BOUNDS, 24);
    uint8_t* d = f.data;
    le::put_u8 (d + 0,  m.mode_id);
    le::put_u16(d + 1,  m.seq);
    le::put_u16(d + 3,  m.freq_max_kHz);
    le::put_u16(d + 5,  m.on_time_max_ns);
    le::put_u16(d + 7,  m.off_time_min_ns);
    le::put_u16(d + 9,  m.peak_I_setpoint_dA);
    le::put_u16(d + 11, m.peak_I_limit_hw_dA);
    le::put_u8 (d + 15, m.polarity);          // d+13,14 reserved padding (0)
    le::put_u16(d + 16, m.flags);
    le::put_u16(d + 18, m.gap_V_arc_mV);
    le::put_u16(d + 20, m.gap_V_short_mV);
    le::put_u16(d + 22, m.ignition_timeout_us);
    return f;
}

CanFrame encodeControl(Control cmd, uint8_t arg) {
    CanFrame f(ID_CONTROL, 2);
    f.data[0] = uint8_t(cmd);
    f.data[1] = arg;
    return f;
}

bool decodeStatsAgg(const CanFrame& f, StatsAgg& s) {
    if (f.id != ID_STATS_AGG || f.len < 38) return false;
    const uint8_t* d = f.data;
    s.window_id               = le::get_u32(d + 0);
    s.n_normal                = le::get_u16(d + 4);
    s.n_arc                   = le::get_u16(d + 6);
    s.n_short                 = le::get_u16(d + 8);
    s.n_open                  = le::get_u16(d + 10);
    s.ignition_delay_mean_ns  = le::get_u16(d + 12);
    s.ignition_delay_stddev_ns= le::get_u16(d + 14);
    s.peak_I_mean_dA          = le::get_u16(d + 16);
    s.peak_I_max_dA           = le::get_u16(d + 18);
    s.gap_V_recovery_mean_ns  = le::get_u16(d + 20);
    s.energy_delivered_uJ     = le::get_u32(d + 22);
    s.temp_GaN_dC             = le::get_i16(d + 26);
    s.temp_L_dC               = le::get_i16(d + 28);
    s.dc_link_V_dV            = le::get_u16(d + 30);
    s.dc_link_I_avg_dA        = le::get_u16(d + 32);
    s.state                   = le::get_u8 (d + 34);
    s.mode_id_active          = le::get_u8 (d + 35);
    s.flags                   = le::get_u16(d + 36);
    return true;
}

bool decodeAckModeBounds(const CanFrame& f, AckModeBounds& a) {
    if (f.id != ID_ACK_MODE_BOUNDS || f.len < 13) return false;
    const uint8_t* d = f.data;
    a.seq                = le::get_u16(d + 0);
    a.freq_max_kHz       = le::get_u16(d + 2);
    a.on_time_max_ns     = le::get_u16(d + 4);
    a.off_time_min_ns    = le::get_u16(d + 6);
    a.peak_I_setpoint_dA = le::get_u16(d + 8);
    a.peak_I_limit_hw_dA = le::get_u16(d + 10);
    a.status             = le::get_u8 (d + 12);
    return true;
}

}}  // namespace EDM::psu
