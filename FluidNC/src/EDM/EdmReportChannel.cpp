// FluidNC/src/EDM/EdmReportChannel.cpp
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Verified by the ESP32 firmware build (P4).
//
// JSONencoder is the FluidNC string/JSON encoder (src/JSONEncoder.h). Constructed
// with a Channel* and a json_tag, its output is wrapped in [MSG:JSON:...] lines
// (see JSONencoder::flush / line). member() only accepts const char*, std::string
// and int32_t, so every numeric report field is widened/cast to int32_t below.

#include "EdmReportChannel.h"

#include "Channel.h"
#include "JSONEncoder.h"

// Compact periodic status: controller/servo state, gap, pulse ratios, command
// velocity and the headline health flags. Keep this small -- it is intended to
// be emitted at the report rate.
void report_edm_stats(Channel& channel, const EDM::EdmReport& r) {
    JSONencoder j(&channel, "EDM");
    j.begin();
    j.member("controller_state", static_cast<int32_t>(r.controller_state));
    j.member("fault_reason", static_cast<int32_t>(r.fault_reason));
    j.member("servo_state", static_cast<int32_t>(r.servo_state));
    j.member("v_cmd_um_s", static_cast<int32_t>(r.v_cmd_um_s));
    j.member("g_milli", static_cast<int32_t>(r.g_milli));
    j.member("g_target_milli", static_cast<int32_t>(r.g_target_milli));
    j.member("open_ratio_milli", static_cast<int32_t>(r.open_ratio_milli));
    j.member("short_ratio_milli", static_cast<int32_t>(r.short_ratio_milli));
    j.member("arc_ratio_milli", static_cast<int32_t>(r.arc_ratio_milli));
    j.member("n_normal", static_cast<int32_t>(r.n_normal));
    j.member("n_arc", static_cast<int32_t>(r.n_arc));
    j.member("n_short", static_cast<int32_t>(r.n_short));
    j.member("n_open", static_cast<int32_t>(r.n_open));
    j.member("feed_cap_pct", static_cast<int32_t>(r.feed_cap_pct));
    j.member("wire_break_sev", static_cast<int32_t>(r.wire_break_sev));
    j.member("connected", static_cast<int32_t>(r.connected ? 1 : 0));
    j.member("protocol_ok", static_cast<int32_t>(r.protocol_ok ? 1 : 0));
    j.member("window_id", static_cast<int32_t>(r.window_id));
    j.end();
}

// Full dump: every field of EdmReport, for diagnostics ($EDM/Status in P4).
void edm_status_dump(Channel& channel, const EDM::EdmReport& r) {
    JSONencoder j(&channel, "EDM");
    j.begin();
    j.member("window_id", static_cast<int32_t>(r.window_id));
    j.member("psu_state", static_cast<int32_t>(r.psu_state));
    j.member("connected", static_cast<int32_t>(r.connected ? 1 : 0));
    j.member("protocol_ok", static_cast<int32_t>(r.protocol_ok ? 1 : 0));
    j.member("last_ack_status", static_cast<int32_t>(r.last_ack_status));
    j.member("controller_state", static_cast<int32_t>(r.controller_state));
    j.member("fault_reason", static_cast<int32_t>(r.fault_reason));
    j.member("servo_state", static_cast<int32_t>(r.servo_state));
    j.member("active_mode_id", static_cast<int32_t>(r.active_mode_id));
    j.member("s_word", static_cast<int32_t>(r.s_word));
    j.member("n_normal", static_cast<int32_t>(r.n_normal));
    j.member("n_arc", static_cast<int32_t>(r.n_arc));
    j.member("n_short", static_cast<int32_t>(r.n_short));
    j.member("n_open", static_cast<int32_t>(r.n_open));
    j.member("total_pulses", static_cast<int32_t>(r.total_pulses));
    j.member("open_ratio_milli", static_cast<int32_t>(r.open_ratio_milli));
    j.member("short_ratio_milli", static_cast<int32_t>(r.short_ratio_milli));
    j.member("arc_ratio_milli", static_cast<int32_t>(r.arc_ratio_milli));
    j.member("v_cmd_um_s", static_cast<int32_t>(r.v_cmd_um_s));
    j.member("g_milli", static_cast<int32_t>(r.g_milli));
    j.member("g_target_milli", static_cast<int32_t>(r.g_target_milli));
    j.member("e_filtered_milli", static_cast<int32_t>(r.e_filtered_milli));
    j.member("integrator_milli", static_cast<int32_t>(r.integrator_milli));
    j.member("feed_cap_pct", static_cast<int32_t>(r.feed_cap_pct));
    j.member("wire_break_sev", static_cast<int32_t>(r.wire_break_sev));
    j.member("peak_I_mean_dA", static_cast<int32_t>(r.peak_I_mean_dA));
    j.member("peak_I_max_dA", static_cast<int32_t>(r.peak_I_max_dA));
    j.member("energy_delivered_uJ", static_cast<int32_t>(r.energy_delivered_uJ));
    j.member("temp_GaN_dC", static_cast<int32_t>(r.temp_GaN_dC));
    j.member("temp_L_dC", static_cast<int32_t>(r.temp_L_dC));
    j.member("dc_link_V_dV", static_cast<int32_t>(r.dc_link_V_dV));
    j.member("flags", static_cast<int32_t>(r.flags));
    j.end();
}
