// FluidNC/src/EDM/Psu/Protocol.h
#pragma once
#include <cstdint>
#include "EDM/Can/CanFrame.h"

namespace EDM { namespace psu {

// ---- CAN-FD standard IDs (PSU spec §8.1) ----
enum Id : uint16_t {
    ID_SET_MODE_BOUNDS = 0x010,  // FluidNC -> PSU
    ID_ACK_MODE_BOUNDS = 0x011,  // PSU -> FluidNC
    ID_CONTROL         = 0x020,  // FluidNC -> PSU (start/stop/reboot/clear)
    ID_STATS_AGG       = 0x100,  // PSU -> FluidNC, 1 kHz
    ID_WIRE_BREAK      = 0x200,  // PSU -> FluidNC, on-demand
    ID_FAULT           = 0x201,
    ID_ARC_BURST       = 0x202,
    ID_INFO            = 0x210,
    ID_PSU_STATUS      = 0x300,  // PSU -> FluidNC, 100 ms heartbeat
};

// Protocol version this firmware implements (must match PSU PSU_STATUS).
constexpr uint16_t kProtocolVersion = 1;

// ---- Control sub-commands (payload byte 0 of ID_CONTROL) ----
enum Control : uint8_t {
    CTRL_START_CUT   = 1,
    CTRL_STOP_CUT    = 2,
    CTRL_CLEAR_FAULT = 3,
    CTRL_REBOOT      = 4,
    CTRL_POLARITY    = 5,  // payload byte 1 = polarity (0/1); only valid when PSU idle
};

// ---- §8.2 SET_MODE_BOUNDS ----
struct SetModeBounds {
    uint8_t  mode_id            = 0;  // 0=idle 1=rough 2=finish 3=ignite 4=bench >=16 user
    uint16_t seq                = 0;
    uint16_t freq_max_kHz       = 0;
    uint16_t on_time_max_ns     = 0;
    uint16_t off_time_min_ns    = 0;
    uint16_t peak_I_setpoint_dA = 0;  // 0.1 A
    uint16_t peak_I_limit_hw_dA = 0;  // 0.1 A
    uint8_t  polarity           = 0;  // 0=workpiece+ 1=wire+
    uint16_t flags              = 0;  // bit0 adaptive, bit1 photodiode_req, bit2 anti_arc, bit3 wirebreak_pred
    uint16_t gap_V_arc_mV       = 0;
    uint16_t gap_V_short_mV     = 0;
    uint16_t ignition_timeout_us = 0;
};

// ---- §8.2 ACK_MODE_BOUNDS (echoes seq + clamped values + status) ----
struct AckModeBounds {
    uint16_t seq                = 0;
    uint16_t freq_max_kHz       = 0;
    uint16_t on_time_max_ns     = 0;
    uint16_t off_time_min_ns    = 0;
    uint16_t peak_I_setpoint_dA = 0;
    uint16_t peak_I_limit_hw_dA = 0;
    uint8_t  status             = 0;  // 0=ok, nonzero=clamped/rejected code
};

// ---- §8.3 STATS_AGG (1 kHz) ----
struct StatsAgg {
    uint32_t window_id              = 0;
    uint16_t n_normal               = 0;
    uint16_t n_arc                  = 0;
    uint16_t n_short                = 0;
    uint16_t n_open                 = 0;
    uint16_t ignition_delay_mean_ns = 0;
    uint16_t ignition_delay_stddev_ns = 0;
    uint16_t peak_I_mean_dA         = 0;
    uint16_t peak_I_max_dA          = 0;
    uint16_t gap_V_recovery_mean_ns = 0;
    uint32_t energy_delivered_uJ    = 0;
    int16_t  temp_GaN_dC            = 0;  // 0.1 C
    int16_t  temp_L_dC              = 0;  // 0.1 C
    uint16_t dc_link_V_dV           = 0;  // 0.1 V
    uint16_t dc_link_I_avg_dA       = 0;  // 0.1 A
    uint8_t  state                  = 0;  // 0=running 1=paused 2=fault
    uint8_t  mode_id_active         = 0;
    uint16_t flags                  = 0;  // hw_trip_recent, photodiode_disagree, watchdog_warn
};

// ---- §8.4 WIRE_BREAK_IMMINENT ----
struct WireBreakImminent {
    uint8_t  severity                  = 0;  // 1 warn, 2 elevated, 3 critical
    uint8_t  cause_flags               = 0;  // delay_var|rise_slope|recovery|thermal
    uint16_t recent_short_count        = 0;
    uint16_t recent_arc_count          = 0;
    uint32_t ignition_delay_var_ns2    = 0;
    uint32_t timestamp_ms_since_start  = 0;
};

// ---- §8.5 FAULT ----
struct Fault {
    uint8_t fault_code = 0;
    uint8_t severity   = 0;
    uint8_t detail[6]  = {};
};

// ---- §8.5 ARC_BURST ----
struct ArcBurst {
    uint16_t consecutive_arcs = 0;
};

// ---- §8.6 PSU_STATUS heartbeat (100 ms) ----
struct PsuStatus {
    uint8_t  state            = 0;
    uint16_t fpga_version     = 0;
    uint16_t mcu_version      = 0;
    uint16_t protocol_version = 0;
    uint32_t uptime_s         = 0;
    uint16_t fault_count      = 0;
};

// ---- encode (FluidNC -> PSU) ----
CanFrame encodeSetModeBounds(const SetModeBounds& m);
CanFrame encodeControl(Control cmd, uint8_t arg = 0);

// ---- decode (PSU -> FluidNC). Return false if id/len mismatch. ----
bool decodeAckModeBounds(const CanFrame& f, AckModeBounds& out);
bool decodeStatsAgg(const CanFrame& f, StatsAgg& out);
bool decodeWireBreak(const CanFrame& f, WireBreakImminent& out);
bool decodeFault(const CanFrame& f, Fault& out);
bool decodeArcBurst(const CanFrame& f, ArcBurst& out);
bool decodeInfo(const CanFrame& f, char* out, size_t out_cap);  // null-terminates
bool decodePsuStatus(const CanFrame& f, PsuStatus& out);

}}  // namespace EDM::psu
