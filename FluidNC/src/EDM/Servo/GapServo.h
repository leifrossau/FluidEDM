// FluidNC/src/EDM/Servo/GapServo.h
#pragma once
#include <cstdint>

namespace EDM { namespace servo {

enum class ServoState : uint8_t { Hold, Track, Retract, ArcHold, BreakRelief };

struct ServoConfig {
    float    short_ref      = 0.08f;
    float    open_ref       = 0.15f;
    float    w_short        = 1.0f;
    float    w_open         = 0.5f;
    float    w_delay        = 0.3f;
    float    alpha          = 0.2f;
    float    Kp             = 8.0f;
    float    Ki             = 0.0f;
    float    deadband       = 0.03f;
    float    v_feed_max     = 4.0f;
    float    v_retract      = 6.0f;
    float    v_touch        = 0.5f;
    float    short_retract  = 0.30f;
    float    arc_brake      = 0.20f;
    float    arc_hold       = 0.40f;
    float    integ_bleed    = 0.5f;
    uint16_t n_min          = 20;
    uint8_t  consec_short_limit = 3;
    uint8_t  pi_decimation  = 5;
    float    dt_pi          = 0.005f;
};

struct GapServoInput {
    bool     valid          = false;
    float    open_ratio     = 0.0f;
    float    short_ratio    = 0.0f;
    float    arc_ratio      = 0.0f;
    float    normal_ratio   = 0.0f;
    float    ign_delay_norm = 0.0f;
    uint16_t total_pulses   = 0;
    float    feed_cap_mult  = 1.0f;
    bool     force_break_relief = false;
    bool     in_touch_off   = false;
};

struct GapServoState {
    ServoState state        = ServoState::Hold;
    float      e_f          = 0.0f;
    float      integ        = 0.0f;
    uint8_t    consec_short = 0;
    uint8_t    decim_count  = 0;
    float      last_v       = 0.0f;
};

struct GapServoOutput {
    GapServoState next;
    int32_t    v_s_um_s     = 0;
    float      v_cmd_mm_min = 0.0f;
    ServoState state        = ServoState::Hold;
    float      g_telemetry  = 0.0f;
    float      g_target     = 0.0f;
    float      e_filtered   = 0.0f;
    float      integrator   = 0.0f;
};

class GapServo {
public:
    explicit GapServo(const ServoConfig& c) : _c(c) {}
    GapServoOutput step(const GapServoState& s, const GapServoInput& in) const;
    const ServoConfig& config() const { return _c; }
    static GapServoState reset() { return GapServoState{}; }
private:
    ServoConfig _c;
};

}}  // namespace EDM::servo
