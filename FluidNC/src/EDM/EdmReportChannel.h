// FluidNC/src/EDM/EdmReportChannel.h
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Uses the FluidNC Channel / JSONencoder APIs; verified by the ESP32 build (P4).
//
// Emits an EDM::EdmReport over a Channel as an encapsulated [MSG:JSON:...] line
// that the embedded WebUI (embedded/edm-webui/index.html) consumes. The compact
// push uses the json_tag "edm_update"; the full dump uses "edm_full_status".
#pragma once

#include "EDM/Servo/EdmReport.h"

class Channel;

namespace EDM { namespace feed { class WireFeed; } }

// Compact periodic telemetry: [MSG:JSON:edm_update]{...}. Keys match the frozen
// WebUI contract. `feed` is optional (null when no wire feed configured) and
// supplies the live tension / collapse fields that EdmReport does not carry.
void report_edm_stats(Channel& channel, const EDM::EdmReport& r, const EDM::feed::WireFeed* feed = nullptr);

// Full dump of every EdmReport field plus the WebUI keys: [MSG:JSON:edm_full_status].
// Backs the `$EDM/Status` command.
void edm_status_dump(Channel& channel, const EDM::EdmReport& r, const EDM::feed::WireFeed* feed = nullptr);

// Periodic push hook. Call from Channel::autoReport(): if the active spindle is
// the EDM spindle, emits one report_edm_stats() line for `channel` using the
// controller's latest snapshot. No-op when EDM is not the active spindle, so it is
// safe to call unconditionally from the shared autoReport path. Rate is governed
// by the caller's existing _reportInterval gating.
void edm_auto_report(Channel& channel);

// String lookups for the uint8 state codes (exposed for tests/reuse). Return a
// stable static string; never null.
const char* edm_controller_state_name(uint8_t controller_state);
const char* edm_servo_state_name(uint8_t servo_state);
const char* edm_fault_name(uint8_t fault_reason);  // returns nullptr when None (-> JSON null)

// JSON KEYS EMITTED (edm_update), mapped from EdmReport:
//   window_id, controller_state(str), servo_state(str), v_cmd_um_s, n_normal,
//   n_arc, n_short, n_open, total_pulses, feed_cap_pct, wire_break_sev,
//   energy_uj, dc_link_v(x10->str), peak_i_a(x10->str), temp_gan_c(x10->str),
//   mode_id, connected, fault(str|null)
// From WireFeed (when present): tension_N, tension_set_N, tension_collapse
// Nested "dielectric" object (from EdmReport diel_* fields, populated by
//   EdmController from the attached IDielLink): present, pump_on, flush_level,
//   flush_bar(mbar/1000->float), flow_lpm(clpm/100->float), temp_c(dC/10->float),
//   temp_set(dC/10->float), conductivity_us, level_pct, filter_pct, flags
// TODO(P4) — NOT yet in EdmReport / not wired through, emitted as 0 with a note:
//   progress_pct, feed_mm_min, x, y, u, v  (come from EdmMotion / the planner)
//   tension_N/tension_set_N fall back to 0 when no WireFeed is configured.
