// FluidNC/src/EDM/Servo/EdmReport.h
#pragma once
#include <cstdint>
namespace EDM {
struct EdmReport {
    uint32_t window_id = 0;
    uint8_t  psu_state = 0;
    bool     connected = false, protocol_ok = false;
    uint16_t last_ack_status = 0;
    uint8_t  controller_state = 0;
    uint8_t  fault_reason = 0;
    uint8_t  servo_state = 0;
    uint8_t  active_mode_id = 0;
    uint16_t s_word = 0;
    uint16_t n_normal = 0, n_arc = 0, n_short = 0, n_open = 0, total_pulses = 0;
    int16_t  open_ratio_milli = 0, short_ratio_milli = 0, arc_ratio_milli = 0;
    int32_t  v_cmd_um_s = 0;
    int16_t  g_milli = 0, g_target_milli = 0, e_filtered_milli = 0, integrator_milli = 0;
    uint8_t  feed_cap_pct = 100, wire_break_sev = 0;
    uint16_t peak_I_mean_dA = 0, peak_I_max_dA = 0;
    uint32_t energy_delivered_uJ = 0;
    int16_t  temp_GaN_dC = 0, temp_L_dC = 0;
    uint16_t dc_link_V_dV = 0;
    uint16_t flags = 0;

    // ---- dielectric (coolant) telemetry (sub-project D) ----
    bool     diel_present = false;
    uint8_t  diel_pump_on = 0, diel_flush_level = 0, diel_level_pct = 0, diel_filter_pct = 0;
    uint16_t diel_flush_mbar = 0, diel_flow_clpm = 0, diel_conductivity_uS = 0;
    int16_t  diel_temp_dC = 0, diel_temp_set_dC = 0;
    uint16_t diel_flags = 0;   // bit0 chiller,1 low_flow,2 low_level,3 high_cond,4 filter_clog
};
}
