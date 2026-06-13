// FluidNC/src/EDM/Control/EdmController.h
#pragma once
#include <cstdint>
#include "EDM/Psu/IPsuLink.h"
#include "EDM/Servo/GapServo.h"
#include "EDM/Servo/ModeTable.h"
#include "EDM/Servo/FaultReason.h"
#include "EDM/Servo/EdmReport.h"

namespace EDM {

enum class EdmState : uint8_t { Idle, Armed, TouchOff, Cutting, Hold, BreakRelief, StallFault, Fault };

struct ControllerTimers {
    uint32_t tele_hold = 20, tele_fault = 200, ack = 200, stall = 2000, break_relief = 50, clear = 500;
    uint32_t touchoff_budget = 6000;
};

class EdmController {
public:
    EdmController(EDM::psu::IPsuLink& link, const EDM::servo::ServoConfig& cfg,
                  const EDM::servo::ModeTable& modes, const ControllerTimers& timers = {});
    void requestCut(uint16_t s_word);
    void requestStop();
    void notifyCutComplete();
    void clearFault();
    void tick(uint32_t now_ms);
    EdmState     state() const  { return _state; }
    FaultReason  fault() const  { return _fault; }
    EdmReport    snapshot() const { return _report; }
    int32_t  vCmdUmPerS()   const { return _report.v_cmd_um_s; }
    EdmState reportedState() const { return EdmState(_report.controller_state); }
private:
    EDM::servo::GapServoInput buildInput(const EDM::psu::StatsAgg& s, bool fresh);
    void drainEvents();
    void maybeSendModeBounds();
    void enterFault(FaultReason r);
    EDM::psu::IPsuLink& _link;
    EDM::servo::GapServo _servo;
    EDM::servo::ModeTable _modes;
    ControllerTimers _t;
    EDM::servo::GapServoState _ss;
    EdmState    _state = EdmState::Idle;
    FaultReason _fault = FaultReason::None;
    EDM::servo::EdmMode _mode = EDM::servo::EdmMode::Idle;
    uint16_t    _s_word = 0;
    uint32_t _now = 0;
    uint32_t _last_window_id = 0;
    bool     _have_window = false;
    uint32_t _last_window_change_ms = 0;
    uint32_t _arm_deadline = 0;
    bool     _arm_fresh = false;
    uint32_t _touchoff_start = 0;
    uint8_t  _touchoff_contig = 0;
    uint32_t _stall_since = 0;
    uint32_t _break_relief_until = 0;
    uint8_t  _lostgap_contig = 0;
    uint8_t  _wire_break_sev = 0;
    uint32_t _sev_clear_after = 0;
    float    _feed_cap_mult = 1.0f;
    uint16_t _seq = 0;
    EDM::psu::SetModeBounds _last_sent{};
    bool     _have_sent = false;
    EdmReport _report{};
};

}  // namespace EDM
