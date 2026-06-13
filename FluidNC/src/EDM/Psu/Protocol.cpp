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

bool decodeWireBreak(const CanFrame& f, WireBreakImminent& w) {
    if (f.id != ID_WIRE_BREAK || f.len < 14) return false;
    const uint8_t* d = f.data;
    w.severity                 = le::get_u8 (d + 0);
    w.cause_flags              = le::get_u8 (d + 1);
    w.recent_short_count       = le::get_u16(d + 2);
    w.recent_arc_count         = le::get_u16(d + 4);
    w.ignition_delay_var_ns2   = le::get_u32(d + 6);
    w.timestamp_ms_since_start = le::get_u32(d + 10);
    return true;
}

bool decodeFault(const CanFrame& f, Fault& flt) {
    if (f.id != ID_FAULT || f.len < 2) return false;
    flt.fault_code = f.data[0];
    flt.severity   = f.data[1];
    std::memset(flt.detail, 0, sizeof(flt.detail));
    uint8_t n = f.len > 8 ? 6 : uint8_t(f.len - 2);
    if (n > 0) std::memcpy(flt.detail, f.data + 2, n);  // guard: f.len may be exactly 2
    return true;
}

bool decodeArcBurst(const CanFrame& f, ArcBurst& a) {
    if (f.id != ID_ARC_BURST || f.len < 2) return false;
    a.consecutive_arcs = le::get_u16(f.data + 0);
    return true;
}

bool decodeInfo(const CanFrame& f, char* out, size_t out_cap) {
    if (f.id != ID_INFO || out_cap == 0) return false;
    size_t n = f.len < (out_cap - 1) ? f.len : (out_cap - 1);
    std::memcpy(out, f.data, n);
    out[n] = '\0';
    return true;
}

bool decodePsuStatus(const CanFrame& f, PsuStatus& st) {
    if (f.id != ID_PSU_STATUS || f.len < 13) return false;
    const uint8_t* d = f.data;
    st.state            = le::get_u8 (d + 0);
    st.fpga_version     = le::get_u16(d + 1);
    st.mcu_version      = le::get_u16(d + 3);
    st.protocol_version = le::get_u16(d + 5);
    st.uptime_s         = le::get_u32(d + 7);
    st.fault_count      = le::get_u16(d + 11);
    return true;
}

}}  // namespace EDM::psu
