// FluidNC/src/EDM/Servo/GapServo.cpp
#include "EDM/Servo/GapServo.h"
#include <cmath>

namespace EDM { namespace servo {

namespace {
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}

GapServoOutput GapServo::step(const GapServoState& s, const GapServoInput& in) const {
    const ServoConfig& c = _c;
    GapServoState ns = s;

    float e_raw = 0.0f, e_delay = 0.0f, g = 0.0f;
    if (in.valid) {
        e_delay = clampf(in.ign_delay_norm, -1.0f, 1.0f);
        e_raw   = c.w_open  * (in.open_ratio  - c.open_ref)
                - c.w_short * (in.short_ratio - c.short_ref)
                + c.w_delay * e_delay;
        ns.e_f  = c.alpha * e_raw + (1.0f - c.alpha) * s.e_f;
        g       = c.w_open * in.open_ratio - c.w_short * in.short_ratio + c.w_delay * e_delay;
    } else {
        ns.e_f  = s.e_f;
    }
    float g_target = c.w_open * c.open_ref - c.w_short * c.short_ref;

    if (in.valid) {
        if (in.short_ratio > c.short_ref * 2.0f) {
            if (ns.consec_short < 255) ns.consec_short++;
        } else {
            ns.consec_short = 0;
        }
    }

    float v_pi = ns.last_v;
    if (in.valid) {
        if (++ns.decim_count >= c.pi_decimation) {
            ns.decim_count = 0;
            bool in_db = std::fabs(ns.e_f) < c.deadband;
            if (c.Ki > 0.0f && !in_db) {
                ns.integ += ns.e_f * c.dt_pi;
                float imax = c.v_feed_max / c.Ki;
                ns.integ = clampf(ns.integ, -imax, imax);
            }
            float v = in_db ? 0.0f : (c.Kp * ns.e_f + c.Ki * ns.integ);
            v = clampf(v, -c.v_retract, c.v_feed_max);
            v_pi = v;
            ns.last_v = v_pi;
        }
    }

    float v;
    ServoState st;
    if (in.in_touch_off) {
        v = c.v_touch;            st = ServoState::Track;
    } else if (!in.valid) {
        v = 0.0f;                 st = ServoState::Hold;
    } else if (in.force_break_relief) {
        v = -c.v_retract; ns.integ *= c.integ_bleed; st = ServoState::BreakRelief;
    } else if (in.short_ratio > c.short_retract ||
               ns.consec_short >= c.consec_short_limit) {
        v = -c.v_retract; ns.integ *= c.integ_bleed; st = ServoState::Retract;
    } else if (in.arc_ratio > c.arc_hold) {
        v = 0.0f;                 st = ServoState::ArcHold;
    } else if (in.arc_ratio > c.arc_brake) {
        float taper = c.v_feed_max * (1.0f - (in.arc_ratio - c.arc_brake) / (1.0f - c.arc_brake));
        v = v_pi < taper ? v_pi : taper;   st = ServoState::Track;
    } else {
        v = v_pi; st = (std::fabs(ns.e_f) < c.deadband) ? ServoState::Hold : ServoState::Track;
    }

    if (v > 0.0f) v *= in.feed_cap_mult;
    v = clampf(v, -c.v_retract, c.v_feed_max);

    ns.state = st;
    GapServoOutput out;
    out.next        = ns;
    out.v_cmd_mm_min= v;
    out.v_s_um_s    = int32_t(std::lround(v * 1000.0f / 60.0f));
    out.state       = st;
    out.g_telemetry = g;
    out.g_target    = g_target;
    out.e_filtered  = ns.e_f;
    out.integrator  = ns.integ;
    return out;
}

}}  // namespace EDM::servo
