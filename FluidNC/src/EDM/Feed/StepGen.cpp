// FluidNC/src/EDM/Feed/StepGen.cpp
//
// ESP32-COUPLED HARDWARE (NOT host-testable; NOT in the native test build_src_filter).
// Verified by the ESP32 firmware build + on-target bench (P4).
//
// Timer note: FluidNC's StepTimer.cpp drives Timer Group 0 / Timer 0 directly via
// the ESP-IDF low-level `timer_ll` API (esp32/StepTimer.cpp). We mirror that style
// for Timer Group 1 here, but the exact Group-1 register handle / per-instance ISR
// dispatch is left as TODO(P4): it is IDF-version-dependent (v4 timer_ll vs v5
// gptimer) and must be confirmed against the build's ESP-IDF before bench bring-up.
// The pure STEP/DIR/DISABLE logic and the atomic hand-off ARE final.

#include "StepGen.h"

#include <cstdlib>  // abs
#include <utility>  // std::move

namespace EDM { namespace feed {

// Default idle re-arm period (us). When parked we still wake periodically so a
// newly-set rate is picked up promptly without leaving the timer disarmed.
static constexpr uint32_t IDLE_REARM_TICKS = 1000;  // 1 ms

void StepGen::begin(Pin&& step, Pin&& dir, Pin&& disable, int timer_num) {
    _step      = std::move(step);
    _dir       = std::move(dir);
    _disable   = std::move(disable);
    _timer_num = timer_num;

    _step.setAttr(Pin::Attr::Output);
    _dir.setAttr(Pin::Attr::Output);
    _disable.setAttr(Pin::Attr::Output);

    _step.synchronousWrite(false);
    _dir.synchronousWrite(_dir_level.load(std::memory_order_relaxed));
    // Start disabled; the wire-feed task enables on the first cutting tick.
    enable(false);

    // TODO(P4): set up Timer Group 1 / `timer_num` at TIMER_HZ with onTimer(this)
    // as the ISR, then arm it (initial alarm ~IDLE_REARM_TICKS). Follow the
    // timer_ll sequence in esp32/StepTimer.cpp (set_divider/prescale, auto-reload,
    // esp_intr_alloc_intrstatus with ESP_INTR_FLAG_IRAM|LEVEL3), but for TIMERG1
    // and routing the ISR arg so onTimer() receives `this`.
}

void StepGen::setRate(int32_t steps_per_s) {
    const bool dir = (steps_per_s < 0);
    _dir_level.store(dir, std::memory_order_relaxed);
    _dir.synchronousWrite(dir);

    uint32_t half = 0;  // parked
    if (steps_per_s != 0) {
        const uint32_t mag = uint32_t(std::abs(steps_per_s));
        // Half-period in timer ticks; a full step is two toggles.
        half = TIMER_HZ / (2u * mag);
        if (half == 0) {
            half = 1;  // clamp absurdly-high rates to the timer resolution
        }
    }
    _half_period_ticks.store(half, std::memory_order_relaxed);
}

void StepGen::enable(bool on) {
    // Many step-stick drivers are active-high DISABLE (HIGH == coils off), so
    // "enabled" drives the pin LOW.
    // TODO(P4): confirm DISABLE polarity for the actual driver; invert here if the
    // board uses active-low ENABLE instead.
    _disable.synchronousWrite(!on);
}

void IRAM_ATTR StepGen::onTimer(void* selfp) {
    auto* self = static_cast<StepGen*>(selfp);

    const uint32_t half = self->_half_period_ticks.load(std::memory_order_relaxed);
    if (half == 0) {
        // Parked: emit no pulse, just re-arm at the idle cadence.
        // TODO(P4): re-arm the Group-1 alarm at IDLE_REARM_TICKS.
        (void)IDLE_REARM_TICKS;
        return;
    }

    // Toggle STEP and re-arm at the half-period.
    self->_step_level = !self->_step_level;
    self->_step.synchronousWrite(self->_step_level);
    // TODO(P4): re-arm the Group-1 alarm at `half` ticks.
    (void)half;
}

}}  // namespace EDM::feed
