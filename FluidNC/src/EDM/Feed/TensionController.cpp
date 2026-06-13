// FluidNC/src/EDM/Feed/TensionController.cpp
#include "EDM/Feed/TensionController.h"
#include <cmath>

namespace EDM { namespace feed {

namespace { inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v);} }

TensionOutput TensionController::step(const TensionState& s, const TensionInput& in) const {
    const TensionConfig& c = _c;
    TensionState ns = s;
    const float   dt  = float(in.dt_ms) / 1000.0f;
    const uint8_t sev = in.severity > 3 ? 3 : in.severity;

    float feed_mm_min = (in.commanded_feed_mm_min >= 0.0f) ? in.commanded_feed_mm_min
                                                           : c.default_feed_mm_min;
    float feed_sps = (feed_mm_min / 60.0f) * c.feed_steps_per_mm;
    if (!in.cutting) feed_sps = 0.0f;
    feed_sps *= in.feed_cap_mult;
    feed_sps  = clampf(feed_sps, 0.0f, c.feed_max_sps);

    float sp_eff = c.setpoint_N * c.sev_tension_scale[sev];

    // Seed the EMA on the first valid sample so the controller sees the true
    // measurement immediately rather than ramping up from zero. Without seeding,
    // the warm-up transient injects a spurious error that integrates and biases
    // the very first control ticks.
    if (in.meas_valid) {
        ns.meas_f = s.meas_primed ? c.tens_alpha * in.measured_N + (1.0f - c.tens_alpha) * s.meas_f
                                  : in.measured_N;
        ns.meas_primed = true;
    }

    float ff_sps = c.ff_ratio * feed_sps;

    // The filtered error drives both the deadband test and the P/I terms, so the
    // controller acts on a single, consistent error signal.
    float e        = sp_eff - ns.meas_f;
    ns.e_f         = e;
    bool active    = in.cutting && in.meas_valid && sev < 3 && !s.collapse_latched;
    bool in_db     = std::fabs(e) < c.tens_deadband_N;
    if (c.Ki > 0.0f && !in_db && active) {
        ns.integ += e * dt;
        float imax = c.pi_clamp_sps / c.Ki;
        ns.integ  = clampf(ns.integ, -imax, imax);
    }
    float pi_sps = in_db ? 0.0f : (c.Kp * e + c.Ki * ns.integ);
    pi_sps = clampf(pi_sps, -c.pi_clamp_sps, c.pi_clamp_sps);

    float tension_sps = ff_sps * c.sev_tension_scale[sev] + pi_sps;

    if (in.cutting && in.meas_valid) {
        bool loaded = in.measured_N > c.loaded_frac * sp_eff;
        if (loaded) {
            ns.since_loaded_ms = 0;
        } else {
            uint32_t acc = s.since_loaded_ms + in.dt_ms;
            ns.since_loaded_ms = acc < s.since_loaded_ms ? 0xFFFFFFFFu : acc;
        }
        bool was_loaded_recently = loaded || ns.since_loaded_ms <= c.loaded_recent_ms;
        bool below = (in.measured_N < c.collapse_N) && (in.measured_N < c.collapse_frac * sp_eff);
        ns.collapse_accum_ms = (below && was_loaded_recently) ? (s.collapse_accum_ms + in.dt_ms) : 0;
        if (ns.collapse_accum_ms >= c.collapse_ms) ns.collapse_latched = true;
    }
    bool collapse = ns.collapse_latched;

    if (sev >= 3 || collapse) {
        feed_sps    = 0.0f;
        tension_sps = c.tension_hold_sps;
        ns.integ   *= c.integ_bleed;
    }

    tension_sps = clampf(tension_sps, c.tension_min_sps, c.tension_max_sps);

    TensionOutput out;
    out.next                = ns;
    out.feed_steps_per_s    = int32_t(std::lround(feed_sps));
    out.tension_steps_per_s = int32_t(std::lround(tension_sps));
    out.ff_sps              = ff_sps;
    out.pi_sps              = pi_sps;
    out.e_filtered          = ns.e_f;
    out.integrator          = ns.integ;
    out.meas_f_N            = ns.meas_f;
    out.tension_collapse    = collapse;
    return out;
}

}}  // namespace EDM::feed
