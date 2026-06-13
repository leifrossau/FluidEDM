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

}}  // namespace EDM::psu
