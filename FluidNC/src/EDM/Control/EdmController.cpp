// FluidNC/src/EDM/Control/EdmController.cpp
#include "EDM/Control/EdmController.h"
#include <cstring>
#include <cmath>

namespace EDM {

using namespace EDM::servo;
using namespace EDM::psu;

EdmController::EdmController(IPsuLink& link, const ServoConfig& cfg,
                            const ModeTable& modes, const ControllerTimers& timers)
    : _link(link), _servo(cfg), _modes(modes), _t(timers), _ss(GapServo::reset()) {}

void EdmController::requestCut(uint16_t s_word) {
    if (_state == EdmState::Fault || _state == EdmState::StallFault) return;
    _s_word = s_word;
    _mode = _modes.modeForSword(s_word);
    if (_mode == EdmMode::Idle) { requestStop(); return; }
    _state = EdmState::Armed;
    _arm_deadline = _now + _t.ack;
    _arm_fresh = true;
    maybeSendModeBounds();
}

void EdmController::requestStop() {
    _link.stopCut();
    _state = EdmState::Idle;
    _mode = EdmMode::Idle;
    _ss = GapServo::reset();
}

void EdmController::notifyCutComplete() { if (_state != EdmState::Fault) requestStop(); }

void EdmController::clearFault() {
    if (_state == EdmState::Fault || _state == EdmState::StallFault) {
        _link.clearFault();
        _state = EdmState::Idle;
        _fault = FaultReason::None;
    }
}

void EdmController::enterFault(FaultReason r) {
    _link.stopCut();
    _state = EdmState::Fault;
    _fault = r;
}

void EdmController::maybeSendModeBounds() {
    SetModeBounds want = _modes.build(_mode, _s_word, 0);
    SetModeBounds a = want;       a.seq = 0;
    SetModeBounds b = _last_sent; b.seq = 0;
    if (!_have_sent || std::memcmp(&a, &b, sizeof(SetModeBounds)) != 0) {
        want.seq = ++_seq;
        _link.setModeBounds(want);
        _last_sent = want;
        _have_sent = true;
    }
}

GapServoInput EdmController::buildInput(const StatsAgg& s, bool fresh) {
    GapServoInput in;
    uint32_t N = uint32_t(s.n_normal) + s.n_arc + s.n_short + s.n_open;
    in.total_pulses = uint16_t(N > 65535 ? 65535 : N);
    in.valid = fresh && N >= _servo.config().n_min;
    if (in.valid) {
        in.open_ratio   = float(s.n_open)   / float(N);
        in.short_ratio  = float(s.n_short)  / float(N);
        in.arc_ratio    = float(s.n_arc)    / float(N);
        in.normal_ratio = float(s.n_normal) / float(N);
        float nom = float(_modes.params(_mode).ign_delay_nom_ns);
        if (nom > 0.0f) in.ign_delay_norm = (float(s.ignition_delay_mean_ns) - nom) / nom;
    }
    in.feed_cap_mult = _feed_cap_mult;
    in.in_touch_off  = (_state == EdmState::TouchOff);
    in.force_break_relief = (_state == EdmState::BreakRelief);
    return in;
}

void EdmController::drainEvents() {
    Event e; int budget = 8;
    while (budget-- > 0 && _link.popEvent(e)) {
        if (e.kind == Event::FaultEvt) { enterFault(FaultReason::PsuFault); return; }
        if (e.kind == Event::WireBreak) {
            uint8_t sev = e.wire_break.severity;
            if (sev > _wire_break_sev) _wire_break_sev = sev;
            _sev_clear_after = _now + _t.clear;
            if (_wire_break_sev >= 3) {
                _feed_cap_mult = 0.0f;
                _state = EdmState::BreakRelief;
                _break_relief_until = _now + _t.break_relief;
                _mode = EdmMode::Ignite; maybeSendModeBounds();
            } else if (_wire_break_sev == 2) {
                _feed_cap_mult = 0.3f;
                _mode = _modes.lowerEnergy(_mode); maybeSendModeBounds();
            } else {
                _feed_cap_mult = 0.6f;
            }
        }
    }
}

bool EdmController::dielReadyToCut(const EDM::diel::DielStats& s) const {
    return s.flow_clpm >= _dielCfg.flow_min_clpm && s.level_pct >= _dielCfg.level_min_pct;
}

void EdmController::tick(uint32_t now_ms) {
    _now = now_ms;
    drainEvents();

    bool connected = _link.isConnected();
    bool proto     = _link.protocolCompatible();
    StatsAgg s; bool got = _link.latestStats(s);
    bool fresh = got && (!_have_window || s.window_id != _last_window_id);
    if (fresh) { _last_window_id = s.window_id; _have_window = true; _last_window_change_ms = now_ms; }

    bool stale_hold  = (now_ms - _last_window_change_ms) > _t.tele_hold;
    bool stale_fault = (now_ms - _last_window_change_ms) > _t.tele_fault;

    // ---- dielectric telemetry + interlock readiness (sub-project D) ----
    EDM::diel::DielStats ds{};
    const bool dgot    = _diel && _diel->latestStats(ds);
    const bool dielReq = _diel && _dielCfg.required;
    const bool dielOk  = !dielReq || (dgot && _diel->present() && dielReadyToCut(ds));
    _report.diel_present = (_diel && _diel->present());
    if (dgot) {
        _report.diel_pump_on        = ds.pump_on;
        _report.diel_flush_level     = ds.flush_level;
        _report.diel_flush_mbar      = ds.flush_mbar;
        _report.diel_flow_clpm       = ds.flow_clpm;
        _report.diel_temp_dC         = ds.temp_dC;
        _report.diel_temp_set_dC     = ds.temp_set_dC;
        _report.diel_conductivity_uS = ds.conductivity_uS;
        _report.diel_level_pct       = ds.level_pct;
        _report.diel_filter_pct      = ds.filter_pct;
        uint16_t dfl = ds.flags;
        if (ds.conductivity_uS > _dielCfg.conductivity_warn_uS) dfl |= 0x0008;  // high-conductivity warning
        _report.diel_flags = dfl;
    }

    if (_state != EdmState::Fault && _state != EdmState::StallFault && _state != EdmState::Idle) {
        if (got && s.state == 2)            { enterFault(FaultReason::PsuFault); }
        else if (!proto)                    { enterFault(FaultReason::ProtocolMismatch); }
        else if (!connected && stale_fault) { enterFault(FaultReason::HeartbeatLost); }
        else if (_have_window && stale_fault){ enterFault(FaultReason::HeartbeatLost); }
    }

    // dielectric mid-cut loss -> fault (feed-hold + spark-off handled by Fault state)
    if (dielReq && (_state == EdmState::Cutting || _state == EdmState::Hold || _state == EdmState::BreakRelief)) {
        if (!_diel->present() || !(dgot && dielReadyToCut(ds))) enterFault(FaultReason::DielectricLost);
    }

    if (_state == EdmState::Armed) {
        if (!proto) { enterFault(FaultReason::ProtocolMismatch); }
        else if (_arm_fresh) { _arm_fresh = false; }  // settle one tick after arming before evaluating ack
        else if (connected && _link.lastAckStatus() == 0 && dielOk) {
            _link.startCut();
            _ss = GapServo::reset();
            _state = EdmState::TouchOff;
            _touchoff_start = now_ms; _touchoff_contig = 0;
        } else if (now_ms >= _arm_deadline) {
            enterFault((dielReq && !dielOk) ? FaultReason::DielectricNotReady : FaultReason::AckTimeout);
        }
    }

    if (_state == EdmState::BreakRelief && now_ms >= _break_relief_until) {
        _state = EdmState::TouchOff; _touchoff_start = now_ms; _touchoff_contig = 0;
    }

    if (_wire_break_sev > 0 && now_ms >= _sev_clear_after) {
        _wire_break_sev = 0; _feed_cap_mult = 1.0f;
        // Restore the S-word cut mode that a sev2/sev3 break temporarily reduced
        // (sev3 -> Ignite, sev2 -> lowerEnergy). Without this the cut would stay
        // at reduced energy for the rest of the job. Only while actively cutting.
        if (_state == EdmState::TouchOff || _state == EdmState::Cutting || _state == EdmState::Hold) {
            _mode = _modes.modeForSword(_s_word);
            maybeSendModeBounds();
        }
    }

    GapServoOutput out;
    bool ran = false;
    if (_state == EdmState::TouchOff || _state == EdmState::Cutting ||
        _state == EdmState::Hold     || _state == EdmState::BreakRelief) {
        GapServoInput in = buildInput(s, fresh);
        if (stale_hold) in.valid = false;
        out = _servo.step(_ss, in); _ss = out.next; ran = true;

        if (_state == EdmState::TouchOff) {
            if (in.valid && (1.0f - in.open_ratio) > 0.5f) {
                if (++_touchoff_contig >= 2) { _state = EdmState::Cutting; _ss.integ = 0.0f; }
            } else {
                _touchoff_contig = 0;
            }
            if (now_ms - _touchoff_start > _t.touchoff_budget) enterFault(FaultReason::TouchOffNoContact);
        } else if (_state == EdmState::Cutting || _state == EdmState::Hold) {
            if (in.valid && in.open_ratio > 0.95f) {
                if (++_lostgap_contig >= 5) { _state = EdmState::TouchOff; _touchoff_start = now_ms; _touchoff_contig = 0; }
            } else { _lostgap_contig = 0; }

            bool advancing = out.v_cmd_mm_min > 0.0f;
            // A balanced in-band hold at a HEALTHY gap is NOT a stall. Only accrue
            // stall time when the gap is unhealthy (elevated short/arc) and we cannot advance.
            bool healthy = in.valid &&
                           in.short_ratio <= _servo.config().short_ref * 2.0f &&
                           in.arc_ratio   <= _servo.config().arc_brake;
            if (out.state == ServoState::Hold || out.state == ServoState::ArcHold || out.v_cmd_mm_min <= 0.0f) {
                if (_state == EdmState::Cutting) { _state = EdmState::Hold; _stall_since = now_ms; }
                if (healthy) {
                    _stall_since = now_ms;   // regulating fine at a good gap; reset the stall timer
                } else if (!advancing && (now_ms - _stall_since) > _t.stall &&
                           (out.state == ServoState::Retract || out.state == ServoState::Hold)) {
                    _link.stopCut(); _state = EdmState::StallFault; _fault = FaultReason::ServoStall;
                }
            } else if (_state == EdmState::Hold) {
                _state = EdmState::Cutting;
            }
        }
    }

    EdmReport& r = _report;
    r.window_id = s.window_id; r.psu_state = s.state;
    r.connected = connected; r.protocol_ok = proto; r.last_ack_status = _link.lastAckStatus();
    r.controller_state = uint8_t(_state); r.fault_reason = uint8_t(_fault);
    r.active_mode_id = _modes.params(_mode).mode_id; r.s_word = _s_word;
    r.n_normal = s.n_normal; r.n_arc = s.n_arc; r.n_short = s.n_short; r.n_open = s.n_open;
    if (ran) {
        r.servo_state = uint8_t(out.state);
        r.v_cmd_um_s  = out.v_s_um_s;
        r.g_milli        = int16_t(std::lround(out.g_telemetry * 1000.0f));
        r.g_target_milli = int16_t(std::lround(out.g_target   * 1000.0f));
        r.e_filtered_milli = int16_t(std::lround(out.e_filtered * 1000.0f));
        r.integrator_milli = int16_t(std::lround(out.integrator * 1000.0f));
        uint32_t N = uint32_t(s.n_normal) + s.n_arc + s.n_short + s.n_open;
        if (N > 0) {
            r.open_ratio_milli  = int16_t(s.n_open  * 1000 / N);
            r.short_ratio_milli = int16_t(s.n_short * 1000 / N);
            r.arc_ratio_milli   = int16_t(s.n_arc   * 1000 / N);
            r.total_pulses = uint16_t(N > 65535 ? 65535 : N);
        }
    } else {
        r.servo_state = uint8_t(ServoState::Hold); r.v_cmd_um_s = 0;
    }
    r.feed_cap_pct = uint8_t(_feed_cap_mult * 100.0f + 0.5f);
    r.wire_break_sev = _wire_break_sev;
    r.peak_I_mean_dA = s.peak_I_mean_dA; r.peak_I_max_dA = s.peak_I_max_dA;
    r.energy_delivered_uJ = s.energy_delivered_uJ;
    r.temp_GaN_dC = s.temp_GaN_dC; r.temp_L_dC = s.temp_L_dC; r.dc_link_V_dV = s.dc_link_V_dV;
    r.flags = s.flags;
}

}  // namespace EDM
