// FluidNC/src/EDM/Feed/StepGen.h
//
// ESP32-COUPLED HARDWARE (NOT host-testable; NOT in the native test build_src_filter).
// Uses the FluidNC Pin class + an ESP32 hardware timer ISR; verified by the ESP32
// firmware build + on-target bench (P4).
//
// Software step generator: a hardware-timer ISR toggles a STEP pin at a rate set
// by the wire-feed task. Single producer (task: setRate/enable) / single consumer
// (ISR: pulse generation), synchronised with std::atomic -- no mutex on the ISR
// path. Uses Timer Group 1 so FluidNC's own stepping (Group 0) is untouched.
#pragma once

#include "Pin.h"

#include <atomic>
#include <cstdint>

#ifndef IRAM_ATTR
#    define IRAM_ATTR
#endif

namespace EDM { namespace feed {

class StepGen {
public:
    StepGen() = default;

    // Take ownership of the pins (Pin is move-only), configure them as outputs
    // and arm the hardware timer `timer_num` in Group 1. Parked (no pulses) and
    // disabled until the first setRate()/enable().
    void begin(Pin&& step, Pin&& dir, Pin&& disable, int timer_num);

    // Set signed step rate. Sign selects DIR; magnitude sets the pulse period.
    // 0 parks the generator (DIR held, no pulses emitted).
    void setRate(int32_t steps_per_s);

    // Drive the DISABLE pin directly (true e-stop independent of the pulse train).
    void enable(bool on);

    // Hardware timer tick used for half-period math (80 MHz ESP32 timer base
    // prescaled to 1 MHz -> ticks are microseconds).
    static constexpr uint32_t TIMER_HZ = 1000000;

private:
    static void IRAM_ATTR onTimer(void* self);

    Pin _step;
    Pin _dir;
    Pin _disable;

    std::atomic<uint32_t> _half_period_ticks { 0 };  // 0 == idle (parked)
    std::atomic<bool>     _dir_level { false };
    volatile bool         _step_level = false;

    int _timer_num = -1;
    // TODO(P4): hardware-timer handle member, matching the confirmed Group-1 API
    // (timer_ll uses a fixed TIMERG1 + index rather than an opaque handle).
};

}}  // namespace EDM::feed
