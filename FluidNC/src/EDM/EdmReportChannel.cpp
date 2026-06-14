// FluidNC/src/EDM/EdmReportChannel.cpp
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Verified by the ESP32 firmware build (P4).
//
// JSONencoder is the FluidNC string/JSON encoder (src/JSONEncoder.h). Constructed
// with a Channel* and a json_tag, its output is wrapped in [MSG:JSON:<tag>] lines.
//
// IMPORTANT — number vs. string: JSONencoder::member() ALWAYS quotes its value
// (even the int32_t overload routes through std::to_string + quoted()), so it can
// only produce JSON string values. The WebUI (embedded/edm-webui/index.html)
// reads several keys as bare numbers (e.g. rep.tension_N.toFixed(1),
// rep.progress_pct||0) and rep.fault as string|null. To emit valid bare-number /
// null literals while still using the house JSONencoder, numeric keys go through
// begin_member()+verbatim() (verbatim() writes raw, unquoted characters); genuine
// string keys (state names, the non-null fault) use member(). This keeps the
// emitted object shape identical to what the WebUI's MockMachine produces.

#include "EdmReportChannel.h"

#include "Channel.h"
#include "JSONEncoder.h"

#include "EDM/Control/EdmController.h"  // EdmState
#include "EDM/Servo/GapServo.h"         // ServoState
#include "EDM/Feed/WireFeed.h"          // tension fields (optional)
#include "EDM/EdmSpindle.h"             // Spindles::EdmSpindle (auto-report hook)

#include "Spindles/Spindle.h"           // global Spindles::Spindle* spindle

#include <array>
#include <cstdio>
#include <cstring>
#include <string>

namespace {

// EdmState (EDM/Control/EdmController.h):
//   Idle, Armed, TouchOff, Cutting, Hold, BreakRelief, StallFault, Fault
constexpr std::array<const char*, 8> kControllerStateNames = {
    "Idle", "Armed", "TouchOff", "Cutting", "Hold", "BreakRelief", "StallFault", "Fault"
};

// ServoState (EDM/Servo/GapServo.h):
//   Hold, Track, Retract, ArcHold, BreakRelief
constexpr std::array<const char*, 5> kServoStateNames = {
    "Hold", "Track", "Retract", "ArcHold", "BreakRelief"
};

// FaultReason (EDM/Servo/FaultReason.h): index 0 == None (-> JSON null).
// Keep in lock-step with the enum (appended-only); the static_assert below trips
// the build if a new FaultReason is added without a name here.
constexpr std::array<const char*, 10> kFaultNames = {
    "None", "AckTimeout", "ProtocolMismatch", "TouchOffNoContact",
    "ServoStall", "HeartbeatLost", "PsuFault", "SensorDisagree",
    "DielectricNotReady", "DielectricLost"
};
static_assert(kFaultNames.size() == static_cast<size_t>(FaultReason::DielectricLost) + 1,
              "kFaultNames is out of sync with enum FaultReason");

// "tag": <int>  (bare number, no quotes)
void num(JSONencoder& j, const char* tag, int32_t v) {
    j.begin_member(tag);
    j.verbatim(std::to_string(v));
}

// "tag": <real.d>  formatted from a deci-unit fixed-point value (v == real * 10).
void num_deci(JSONencoder& j, const char* tag, int32_t deci) {
    char buf[24];
    int32_t whole = deci / 10;
    int32_t frac  = deci % 10;
    if (frac < 0) {
        frac = -frac;
    }
    const char* sign = (deci < 0 && whole == 0) ? "-" : "";  // keep -0.5 negative
    std::snprintf(buf, sizeof(buf), "%s%ld.%ld", sign, static_cast<long>(whole), static_cast<long>(frac));
    j.begin_member(tag);
    j.verbatim(buf);
}

// "tag": <v.d>  formatted from a float (one decimal place).
void num_f1(JSONencoder& j, const char* tag, float v) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1f", static_cast<double>(v));
    j.begin_member(tag);
    j.verbatim(buf);
}

// "tag": <v.dd>  formatted by dividing an integer by `div` (a power of ten).
// Mirrors num_deci/num_f1: writes a bare (unquoted) decimal literal via verbatim()
// so the WebUI reads it as a number. `decimals` matches the number of zeros in
// `div` so the printed precision lines up with the fixed-point resolution
// (e.g. div=1000 -> 3 decimals for mbar->bar; div=100 -> 2 decimals for cL->L).
void num_scaled(JSONencoder& j, const char* tag, int32_t v, int32_t div, int decimals) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, static_cast<double>(v) / static_cast<double>(div));
    j.begin_member(tag);
    j.verbatim(buf);
}

}  // namespace

const char* edm_controller_state_name(uint8_t s) {
    return (s < kControllerStateNames.size()) ? kControllerStateNames[s] : "Idle";
}

const char* edm_servo_state_name(uint8_t s) {
    return (s < kServoStateNames.size()) ? kServoStateNames[s] : "Hold";
}

const char* edm_fault_name(uint8_t f) {
    if (f == 0 || f >= kFaultNames.size()) {
        return nullptr;  // None -> JSON null
    }
    return kFaultNames[f];
}

// Emit the WireFeed-sourced tension keys. When no feed is configured we still emit
// the keys (0) so the WebUI's rep.tension_N.toFixed() never sees undefined.
static void emit_tension(JSONencoder& j, const EDM::feed::WireFeed* feed) {
    if (feed) {
        num_f1(j, "tension_N", feed->tensionN());
        num_f1(j, "tension_set_N", feed->tensionSetpointN());
        num(j, "tension_collapse", feed->tensionCollapsed() ? 1 : 0);
    } else {
        // TODO(P4): no WireFeed configured -> tension not measured yet (emit 0s).
        num_f1(j, "tension_N", 0.0f);
        num_f1(j, "tension_set_N", 0.0f);
        num(j, "tension_collapse", 0);
    }
}

// Emit the controller/servo/fault names plus the EdmReport-derived numeric keys.
static void emit_core(JSONencoder& j, const EDM::EdmReport& r) {
    num(j, "window_id", static_cast<int32_t>(r.window_id));
    j.member("controller_state", edm_controller_state_name(r.controller_state));  // string
    j.member("servo_state", edm_servo_state_name(r.servo_state));                 // string

    num(j, "v_cmd_um_s", static_cast<int32_t>(r.v_cmd_um_s));

    num(j, "n_normal", static_cast<int32_t>(r.n_normal));
    num(j, "n_arc", static_cast<int32_t>(r.n_arc));
    num(j, "n_short", static_cast<int32_t>(r.n_short));
    num(j, "n_open", static_cast<int32_t>(r.n_open));
    num(j, "total_pulses", static_cast<int32_t>(r.total_pulses));

    num(j, "feed_cap_pct", static_cast<int32_t>(r.feed_cap_pct));
    num(j, "wire_break_sev", static_cast<int32_t>(r.wire_break_sev));

    num(j, "energy_uj", static_cast<int32_t>(r.energy_delivered_uJ));
    num_deci(j, "dc_link_v", r.dc_link_V_dV);    // dV -> V
    num_deci(j, "peak_i_a", r.peak_I_mean_dA);   // dA -> A
    num_deci(j, "temp_gan_c", r.temp_GaN_dC);    // dC -> C

    num(j, "mode_id", static_cast<int32_t>(r.active_mode_id));
    num(j, "connected", r.connected ? 1 : 0);

    // fault: string when faulted, bare null literal otherwise.
    const char* fault = edm_fault_name(r.fault_reason);
    if (fault) {
        j.member("fault", fault);
    } else {
        j.begin_member("fault");
        j.verbatim("null");
    }

    // TODO(P4): progress_pct / feed_mm_min / x / y / u / v are produced by
    // EdmMotion + the planner, not by EdmReport. Emit 0 placeholders so the keys
    // exist; wire real values when EdmMotion publishes a pose/progress snapshot.
    num(j, "progress_pct", 0);   // TODO(P4): from EdmMotion progress
    num(j, "feed_mm_min", 0);    // TODO(P4): from planner feed rate
    num(j, "x", 0);              // TODO(P4): from EdmMotion pose
    num(j, "y", 0);              // TODO(P4)
    num(j, "u", 0);              // TODO(P4)
    num(j, "v", 0);              // TODO(P4)
}

// Emit the dielectric (coolant) telemetry as a nested object, matching the
// frozen WebUI contract (embedded/edm-webui consumes rep.dielectric.*):
//   present, pump_on, flush_level, flush_bar(float), flow_lpm(float),
//   temp_c(float), temp_set(float), conductivity_us, level_pct, filter_pct, flags.
// The EdmReport diel_* fields are populated by EdmController::tick() from the
// attached IDielLink; when no module is present they stay 0 and present==false,
// so the keys always exist (the WebUI never sees undefined).
static void emit_dielectric(JSONencoder& j, const EDM::EdmReport& r) {
    j.begin_member_object("dielectric");
    num(j, "present", r.diel_present ? 1 : 0);
    num(j, "pump_on", static_cast<int32_t>(r.diel_pump_on));
    num(j, "flush_level", static_cast<int32_t>(r.diel_flush_level));
    num_scaled(j, "flush_bar", static_cast<int32_t>(r.diel_flush_mbar), 1000, 3);  // mbar -> bar
    num_scaled(j, "flow_lpm", static_cast<int32_t>(r.diel_flow_clpm), 100, 2);      // cL/min -> L/min
    num_deci(j, "temp_c", r.diel_temp_dC);          // dC -> C (one decimal)
    num_deci(j, "temp_set", r.diel_temp_set_dC);    // dC -> C (one decimal)
    num(j, "conductivity_us", static_cast<int32_t>(r.diel_conductivity_uS));
    num(j, "level_pct", static_cast<int32_t>(r.diel_level_pct));
    num(j, "filter_pct", static_cast<int32_t>(r.diel_filter_pct));
    num(j, "flags", static_cast<int32_t>(r.diel_flags));
    j.end_object();
}

// Compact periodic telemetry pushed at report_hz. Small by design.
//
// ENVELOPE NOTE (TODO(P4)): the WebUI routes on the line prefix
// `[MSG:JSON:edm_update]`. FluidNC's JSONencoder takes the json_tag
// ("edm_update") but the base Channel::out()/WSChannel path currently DROPS the
// tag (only UartChannel prepends "[" + tag, and even there beginJSON is a no-op),
// so the tag does not reliably reach the wire. We pass the tag to JSONencoder (so
// the bench-side envelope wiring can pick it up) AND embed a redundant "type"
// member inside the object so a consumer can route on the payload regardless of
// the envelope. Reconciling encoder tag -> [MSG:JSON:<tag>] for the WS path is a
// focused follow-up in the messaging layer, intentionally not done here.
void report_edm_stats(Channel& channel, const EDM::EdmReport& r, const EDM::feed::WireFeed* feed) {
    JSONencoder j(&channel, "edm_update");
    j.begin();
    j.member("type", "edm_update");  // in-payload discriminator (see ENVELOPE NOTE)
    emit_core(j, r);
    emit_tension(j, feed);
    emit_dielectric(j, r);
    j.end();
}

// Periodic push from Channel::autoReport(). No-op unless EDM is the active spindle.
void edm_auto_report(Channel& channel) {
    if (!spindle || std::strcmp(spindle->name(), "EDM") != 0) {
        return;
    }
    auto* edm = static_cast<Spindles::EdmSpindle*>(spindle);
    auto* ctl = edm->controller();
    if (!ctl) {
        return;
    }
    report_edm_stats(channel, ctl->snapshot(), edm->wireFeed());
}

// Full diagnostic dump: the WebUI keys plus the raw EdmReport internals that the
// UI does not need but are useful for `$EDM/Status` on a terminal.
void edm_status_dump(Channel& channel, const EDM::EdmReport& r, const EDM::feed::WireFeed* feed) {
    JSONencoder j(&channel, "edm_full_status");
    j.begin();
    j.member("type", "edm_full_status");  // in-payload discriminator (see ENVELOPE NOTE)
    emit_core(j, r);
    emit_tension(j, feed);
    emit_dielectric(j, r);

    // Raw / extra fields (fixed-point ints kept as-is for diagnostics).
    num(j, "psu_state", static_cast<int32_t>(r.psu_state));
    num(j, "protocol_ok", r.protocol_ok ? 1 : 0);
    num(j, "last_ack_status", static_cast<int32_t>(r.last_ack_status));
    num(j, "fault_reason", static_cast<int32_t>(r.fault_reason));
    num(j, "s_word", static_cast<int32_t>(r.s_word));
    num(j, "open_ratio_milli", static_cast<int32_t>(r.open_ratio_milli));
    num(j, "short_ratio_milli", static_cast<int32_t>(r.short_ratio_milli));
    num(j, "arc_ratio_milli", static_cast<int32_t>(r.arc_ratio_milli));
    num(j, "g_milli", static_cast<int32_t>(r.g_milli));
    num(j, "g_target_milli", static_cast<int32_t>(r.g_target_milli));
    num(j, "e_filtered_milli", static_cast<int32_t>(r.e_filtered_milli));
    num(j, "integrator_milli", static_cast<int32_t>(r.integrator_milli));
    num(j, "peak_I_max_dA", static_cast<int32_t>(r.peak_I_max_dA));
    num(j, "temp_L_dC", static_cast<int32_t>(r.temp_L_dC));
    num(j, "flags", static_cast<int32_t>(r.flags));
    j.end();
}
