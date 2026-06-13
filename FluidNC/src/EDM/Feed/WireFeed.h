// FluidNC/src/EDM/Feed/WireFeed.h
//
// ESP32-COUPLED HARDWARE (NOT host-testable; NOT in the native test build_src_filter).
// Uses FluidNC Pin/Config + the ESP32 HX711/StepGen; verified by the ESP32 firmware
// build + on-target bench (P4). The control math (TensionController) is host-tested.
//
// WireFeed is the facade that wires the host-tested TensionController to the
// on-target hardware: it reads the HX711 load cell, pulls feed-cap / wire-break
// severity / state from the EdmController, runs one TensionController::step() per
// tick, and drives the feed + tension stepper StepGens (plus the feed DISABLE line
// as a true e-stop on collapse / severe wire break).
#pragma once

#include "TensionController.h"
#include "Hx711.h"
#include "StepGen.h"
#include "Configuration/Configurable.h"

#include "Pin.h"

#include <cstdint>
#include <vector>

namespace EDM {
class EdmController;
}

namespace EDM { namespace feed {

// --- config sub-sections (each a nested YAML section, FluidNC pointer-section idiom) ---

struct MotorCfg : public Configuration::Configurable {
    Pin   step_pin;
    Pin   dir_pin;
    Pin   disable_pin;
    float steps_per_mm       = 100.0f;
    float default_feed_mm_min = 60.0f;  // feed motor only
    float min_sps            = 0.0f;    // tension motor only
    float max_sps            = 4000.0f;
    float hold_sps           = 200.0f;  // tension motor only

    void group(Configuration::HandlerBase& handler) override;
};

struct TensionCtrlCfg : public Configuration::Configurable {
    float ff_ratio     = 1.0f;
    float setpoint_N   = 5.0f;
    float kp           = 40.0f;
    float ki           = 8.0f;
    float alpha        = 0.30f;
    float deadband_N   = 0.25f;
    float pi_clamp_sps = 400.0f;
    float integ_bleed  = 0.5f;

    void group(Configuration::HandlerBase& handler) override;
};

struct CollapseCfg : public Configuration::Configurable {
    float    threshold_N      = 0.50f;
    float    frac             = 0.20f;
    float    loaded_frac      = 0.50f;
    uint32_t loaded_recent_ms = 200;
    uint32_t debounce_ms      = 30;

    void group(Configuration::HandlerBase& handler) override;
};

class WireFeedConfig : public Configuration::Configurable {
public:
    WireFeedConfig() = default;

    void group(Configuration::HandlerBase& handler) override;

    // Build a TensionConfig from the parsed sections (defaults fill any gaps).
    TensionConfig buildTensionConfig() const;

    // Parser-owned pointer sub-sections (FluidNC house style). Nullable: a missing
    // YAML section leaves the pointer null and the corresponding defaults apply.
    MotorCfg*       _feed_motor     = nullptr;
    MotorCfg*       _tension_motor  = nullptr;
    TensionCtrlCfg* _tension_control = nullptr;
    CollapseCfg*    _collapse       = nullptr;
    Hx711*          _load_cell      = nullptr;  // Hx711 is itself Configurable

    // severity_tension_scale[4]; copied into TensionConfig if exactly 4 entries.
    std::vector<float> _sev_tension_scale;
};

// --- runtime facade ---

class WireFeed {
public:
    WireFeed() = default;

    // Wire up hardware from `cfg` and remember the controller. Non-const because
    // StepGen takes ownership of the configured Pins (Pin is move-only), so the
    // pins are moved out of the config sub-sections into the StepGens.
    void begin(EDM::EdmController* ctl, WireFeedConfig& cfg);

    // One control step (~83 Hz from the wire-feed task).
    void tick(uint32_t now_ms);

    // A detected mechanical wire-tension collapse LATCHES (feed stays DISABLED):
    // this is a safety stop — the wire has physically broken/slackened, so we never
    // auto-recover. Recovery is a deliberate operator action; P4 wires this to an
    // `$EDM/ResetWireFeed` command (and M5+re-arm re-runs begin(), which also resets).
    void resetCollapse() { _state = TensionController::reset(); }

    // --- P4 telemetry / command surface (consumed by EdmReportChannel + $EDM/*) ---
    // Live measured wire tension (N), last sampled by tick().
    float tensionN() const         { return _last_N; }
    // Active tension setpoint (N); settable at runtime via `$EDM/Tension=<N>`.
    float tensionSetpointN() const { return _controller.setpointN(); }
    void  setTensionSetpointN(float n) { _controller.setSetpointN(n); }
    // True while a mechanical tension collapse is latched (feed e-stopped).
    bool  tensionCollapsed() const { return _state.collapse_latched; }

private:
    TensionController  _controller { TensionConfig {} };
    TensionState       _state;
    Hx711*             _hx711 = nullptr;  // owned by WireFeedConfig (parser-allocated)
    StepGen            _feedGen;
    StepGen            _tensionGen;
    EDM::EdmController* _ctl = nullptr;

    uint32_t _last_ms        = 0;
    int32_t  _last_raw       = 0;
    float    _last_N         = 0.0f;
};

}}  // namespace EDM::feed
