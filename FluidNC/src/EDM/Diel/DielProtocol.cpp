// FluidNC/src/EDM/Diel/DielProtocol.cpp
#include "EDM/Diel/DielProtocol.h"
#include "EDM/Psu/Endian.h"
#include <cstring>

namespace EDM { namespace diel {

CanFrame encodeSetDiel(const SetDiel& s) {
    CanFrame f(ID_SET_DIEL, 9);
    uint8_t* d = f.data;
    le::put_u16(d + 0, s.seq);
    le::put_u8 (d + 2, s.pump_on);
    le::put_u8 (d + 3, s.flush_level);
    le::put_i16(d + 4, s.temp_setpoint_dC);
    le::put_u8 (d + 6, s.deioniser_enable);
    le::put_u16(d + 7, s.flags);
    return f;
}

bool decodeAckDiel(const CanFrame& f, AckDiel& a) {
    if (f.id != ID_ACK_DIEL || f.len < 7) return false;
    const uint8_t* d = f.data;
    a.seq = le::get_u16(d + 0); a.pump_on = d[2]; a.flush_level = d[3];
    a.temp_setpoint_dC = le::get_i16(d + 4); a.status = d[6];
    return true;
}

bool decodeDielStats(const CanFrame& f, DielStats& s) {
    if (f.id != ID_DIEL_STATS || f.len < 20) return false;
    const uint8_t* d = f.data;
    s.window_id       = le::get_u32(d + 0);
    s.pump_on         = d[4]; s.flush_level = d[5];
    s.flush_mbar      = le::get_u16(d + 6);
    s.flow_clpm       = le::get_u16(d + 8);
    s.temp_dC         = le::get_i16(d + 10);
    s.temp_set_dC     = le::get_i16(d + 12);
    s.conductivity_uS = le::get_u16(d + 14);
    s.level_pct       = d[16]; s.filter_pct = d[17];
    s.flags           = le::get_u16(d + 18);
    return true;
}

bool decodeDielFault(const CanFrame& f, DielFault& flt) {
    if (f.id != ID_DIEL_FAULT || f.len < 2) return false;
    flt.fault_code = f.data[0]; flt.severity = f.data[1];
    std::memset(flt.detail, 0, sizeof(flt.detail));
    uint8_t n = f.len > 8 ? 6 : uint8_t(f.len - 2);
    if (n > 0) std::memcpy(flt.detail, f.data + 2, n);
    return true;
}

bool decodeDielStatus(const CanFrame& f, DielStatus& st) {
    if (f.id != ID_DIEL_STATUS || f.len < 9) return false;
    const uint8_t* d = f.data;
    st.state = d[0]; st.fw_version = le::get_u16(d + 1);
    st.protocol_version = le::get_u16(d + 3); st.uptime_s = le::get_u32(d + 5);
    return true;
}

}}  // namespace EDM::diel
