// FluidNC/src/EDM/Feed/WireFeed.cpp
//
// ESP32-COUPLED HARDWARE (NOT host-testable; NOT in the native test build_src_filter).
// Verified by the ESP32 firmware build + on-target bench (P4).

#include "WireFeed.h"

#include "EDM/Control/EdmController.h"

#include <utility>  // std::move

namespace EDM { namespace feed {

// --- config sub-sections ---

void MotorCfg::group(Configuration::HandlerBase& handler) {
    handler.item("step_pin", step_pin);
    handler.item("dir_pin", dir_pin);
    handler.item("disable_pin", disable_pin);
    handler.item("steps_per_mm", steps_per_mm);
    handler.item("default_feed_mm_min", default_feed_mm_min);
    handler.item("min_sps", min_sps);
    handler.item("max_sps", max_sps);
    handler.item("hold_sps", hold_sps);
}

void TensionCtrlCfg::group(Configuration::HandlerBase& handler) {
    handler.item("ff_ratio", ff_ratio);
    handler.item("setpoint_N", setpoint_N);
    handler.item("kp", kp);
    handler.item("ki", ki);
    handler.item("alpha", alpha);
    handler.item("deadband_N", deadband_N);
    handler.item("pi_clamp_sps", pi_clamp_sps);
    handler.item("integ_bleed", integ_bleed);
}

void CollapseCfg::group(Configuration::HandlerBase& handler) {
    handler.item("threshold_N", threshold_N);
    handler.item("frac", frac);
    handler.item("loaded_frac", loaded_frac);
    handler.item("loaded_recent_ms", loaded_recent_ms);
    handler.item("debounce_ms", debounce_ms);
}

void WireFeedConfig::group(Configuration::HandlerBase& handler) {
    handler.section("feed_motor", _feed_motor);
    handler.section("tension_motor", _tension_motor);
    handler.section("tension_control", _tension_control);
    handler.section("collapse", _collapse);
    handler.section("load_cell", _load_cell);
    handler.item("severity_tension_scale", _sev_tension_scale);
}

TensionConfig WireFeedConfig::buildTensionConfig() const {
    TensionConfig tc;  // start from the host-tested defaults

    if (_feed_motor) {
        tc.feed_steps_per_mm   = _feed_motor->steps_per_mm;
        tc.default_feed_mm_min = _feed_motor->default_feed_mm_min;
        tc.feed_max_sps        = _feed_motor->max_sps;
    }
    if (_tension_motor) {
        tc.tension_min_sps  = _tension_motor->min_sps;
        tc.tension_max_sps  = _tension_motor->max_sps;
        tc.tension_hold_sps = _tension_motor->hold_sps;
    }
    if (_tension_control) {
        tc.ff_ratio        = _tension_control->ff_ratio;
        tc.setpoint_N      = _tension_control->setpoint_N;
        tc.Kp              = _tension_control->kp;
        tc.Ki              = _tension_control->ki;
        tc.tens_alpha      = _tension_control->alpha;
        tc.tens_deadband_N = _tension_control->deadband_N;
        tc.pi_clamp_sps    = _tension_control->pi_clamp_sps;
        tc.integ_bleed     = _tension_control->integ_bleed;
    }
    if (_collapse) {
        tc.collapse_N        = _collapse->threshold_N;
        tc.collapse_frac     = _collapse->frac;
        tc.loaded_frac       = _collapse->loaded_frac;
        tc.loaded_recent_ms  = _collapse->loaded_recent_ms;
        tc.collapse_ms       = _collapse->debounce_ms;
    }
    if (_sev_tension_scale.size() == 4) {
        for (int i = 0; i < 4; ++i) {
            tc.sev_tension_scale[i] = _sev_tension_scale[size_t(i)];
        }
    }
    return tc;
}

// --- runtime facade ---

void WireFeed::begin(EDM::EdmController* ctl, WireFeedConfig& cfg) {
    _ctl = ctl;

    // Rebuild the controller from the parsed config (implicit copy-assignment;
    // TensionController only holds a TensionConfig value).
    _controller = TensionController(cfg.buildTensionConfig());
    _state      = TensionController::reset();

    // Load cell: parser-allocated and owned by the config; begin() configures pins.
    _hx711 = cfg._load_cell;
    if (_hx711) {
        _hx711->begin();
    }

    // Feed StepGen on Timer Group-1 timer 2; tension on timer 3. StepGen takes
    // ownership of the pins (move), so the config sub-sections are emptied here.
    if (cfg._feed_motor) {
        _feedGen.begin(std::move(cfg._feed_motor->step_pin),
                       std::move(cfg._feed_motor->dir_pin),
                       std::move(cfg._feed_motor->disable_pin),
                       2);
    }
    if (cfg._tension_motor) {
        _tensionGen.begin(std::move(cfg._tension_motor->step_pin),
                          std::move(cfg._tension_motor->dir_pin),
                          std::move(cfg._tension_motor->disable_pin),
                          3);
    }

    _last_ms = 0;
}

void WireFeed::tick(uint32_t now_ms) {
    // 1) Sample the load cell only when a fresh conversion is ready (non-blocking).
    const bool fresh = _hx711 ? _hx711->dataReady() : false;
    if (fresh) {
        _last_raw = _hx711->readRaw();
        _last_N   = _hx711->toNewtons(_last_raw);
    }

    // 2) Pull live state from the gap-servo controller.
    uint8_t        cap = 100;
    uint8_t        sev = 0;
    EDM::EdmState  st  = EDM::EdmState::Idle;
    if (_ctl) {
        cap = _ctl->feedCapPct();
        sev = _ctl->wireBreakSeverity();
        st  = _ctl->reportedState();
    }
    const bool cutting = (st == EDM::EdmState::Cutting || st == EDM::EdmState::TouchOff ||
                          st == EDM::EdmState::Hold || st == EDM::EdmState::BreakRelief);

    // 3) Assemble the control input.
    TensionInput in;
    in.measured_N            = _last_N;
    in.meas_valid            = fresh;
    in.feed_cap_mult         = float(cap) / 100.0f;
    in.severity              = sev;
    in.cutting               = cutting;
    in.commanded_feed_mm_min = -1.0f;  // use config default feed
    in.dt_ms                 = now_ms - _last_ms;
    _last_ms                 = now_ms;

    // 4) Step the controller.
    const TensionOutput o = _controller.step(_state, in);
    _state                = o.next;

    // 5) Drive the steppers.
    _feedGen.setRate(o.feed_steps_per_s);
    _tensionGen.setRate(o.tension_steps_per_s);

    // 6) Feed DISABLE as a true e-stop: cut power to the feed axis on a severe
    //    wire break or a detected tension collapse; otherwise re-enable while cutting.
    if (sev >= 3 || o.tension_collapse) {
        _feedGen.enable(false);
        if (o.tension_collapse) {
            _collapse_sticky = true;
        }
    } else if (cutting) {
        _feedGen.enable(true);
    }
}

}}  // namespace EDM::feed
