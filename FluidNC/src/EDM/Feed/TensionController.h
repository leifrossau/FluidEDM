// FluidNC/src/EDM/Feed/TensionController.h
#pragma once
#include <cstdint>

namespace EDM { namespace feed {

struct TensionConfig {
    float    default_feed_mm_min = 60.0f;
    float    feed_steps_per_mm   = 100.0f;
    float    feed_max_sps        = 4000.0f;
    float    ff_ratio            = 1.0f;
    float    setpoint_N          = 5.0f;
    float    Kp                  = 40.0f;
    float    Ki                  = 8.0f;
    float    tens_alpha          = 0.30f;
    float    tens_deadband_N     = 0.25f;
    float    pi_clamp_sps        = 400.0f;
    float    tension_min_sps     = 0.0f;
    float    tension_max_sps     = 4000.0f;
    float    tension_hold_sps    = 200.0f;
    float    integ_bleed         = 0.5f;
    float    sev_tension_scale[4]= { 1.0f, 0.85f, 0.60f, 0.0f };
    float    collapse_N          = 0.50f;
    float    collapse_frac       = 0.20f;
    float    loaded_frac         = 0.50f;
    uint32_t loaded_recent_ms    = 200;
    uint32_t collapse_ms         = 30;
};

struct TensionInput {
    float    measured_N           = 0.0f;
    bool     meas_valid           = false;
    float    feed_cap_mult        = 1.0f;
    uint8_t  severity             = 0;
    bool     cutting              = false;
    float    commanded_feed_mm_min= -1.0f;
    uint32_t dt_ms                = 12;
};

struct TensionState {
    float    meas_f               = 0.0f;
    float    e_f                  = 0.0f;
    float    integ                = 0.0f;
    uint32_t collapse_accum_ms    = 0;
    uint32_t since_loaded_ms      = 0;
    bool     collapse_latched     = false;
    bool     meas_primed          = false;
};

struct TensionOutput {
    TensionState next;
    int32_t  feed_steps_per_s    = 0;
    int32_t  tension_steps_per_s = 0;
    float    ff_sps              = 0.0f;
    float    pi_sps              = 0.0f;
    float    e_filtered          = 0.0f;
    float    integrator          = 0.0f;
    float    meas_f_N            = 0.0f;
    bool     tension_collapse    = false;
};

class TensionController {
public:
    explicit TensionController(const TensionConfig& c) : _c(c) {}
    TensionOutput step(const TensionState& s, const TensionInput& in) const;
    const TensionConfig& config() const { return _c; }
    // Runtime setpoint override (used by the `$EDM/Tension=<N>` command in P4).
    // Additive accessor; does not change step() behavior for a fixed setpoint.
    void  setSetpointN(float n) { _c.setpoint_N = n; }
    float setpointN() const     { return _c.setpoint_N; }
    static TensionState reset() { return TensionState{}; }
private:
    TensionConfig _c;
};

}}  // namespace EDM::feed
