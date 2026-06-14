// FluidNC/src/EDM/Diel/DielProtocol.h
#pragma once
#include <cstdint>
#include "EDM/Can/CanFrame.h"

namespace EDM { namespace diel {

enum Id : uint16_t {
    ID_SET_DIEL    = 0x710,  // FluidNC -> module
    ID_ACK_DIEL    = 0x711,  // module -> FluidNC
    ID_DIEL_STATS  = 0x720,  // module -> FluidNC, ~10 Hz
    ID_DIEL_FAULT  = 0x730,
    ID_DIEL_STATUS = 0x740,  // module -> FluidNC, 100 ms heartbeat
};
constexpr uint16_t kProtocolVersion = 1;

struct SetDiel {
    uint16_t seq = 0;
    uint8_t  pump_on = 0;
    uint8_t  flush_level = 0;          // 0=off,1=low,2=med,3=high
    int16_t  temp_setpoint_dC = 220;   // 0.1 C
    uint8_t  deioniser_enable = 1;
    uint16_t flags = 0;
};
struct AckDiel {
    uint16_t seq = 0; uint8_t pump_on = 0; uint8_t flush_level = 0;
    int16_t temp_setpoint_dC = 0; uint8_t status = 0;
};
struct DielStats {
    uint32_t window_id = 0;
    uint8_t  pump_on = 0, flush_level = 0;
    uint16_t flush_mbar = 0, flow_clpm = 0;   // cL/min (650 = 6.50 L/min)
    int16_t  temp_dC = 0, temp_set_dC = 0;
    uint16_t conductivity_uS = 0;
    uint8_t  level_pct = 0, filter_pct = 0;
    uint16_t flags = 0;                        // bit0 chiller,1 low_flow,2 low_level,3 high_cond,4 filter_clog
};
struct DielFault { uint8_t fault_code = 0, severity = 0; uint8_t detail[6] = {}; };
struct DielStatus { uint8_t state = 0; uint16_t fw_version = 0, protocol_version = 0; uint32_t uptime_s = 0; };

CanFrame encodeSetDiel(const SetDiel& s);
bool decodeAckDiel(const CanFrame& f, AckDiel& out);
bool decodeDielStats(const CanFrame& f, DielStats& out);
bool decodeDielFault(const CanFrame& f, DielFault& out);
bool decodeDielStatus(const CanFrame& f, DielStatus& out);

}}  // namespace EDM::diel
