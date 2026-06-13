# EDM PSU Integration — Phase P3 (Wire Feed + Tension) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Closed-loop wire feed + tension: a feed motor that meters wire (scaled by the controller's `feed_cap_mult`), a tension motor held to a setpoint by feedforward + PI on an HX711 load cell, with severity-scaled wire-break reactions and a latched mechanical-collapse e-stop.

**Architecture:** A pure `TensionController` (feedforward base = `ff_ratio × capped feed rate`, plus PI on load-cell error, anti-windup, integrator bleed on emergency, 3-condition latched collapse detector) — struct-in/struct-out, mirrors `GapServo` — 100% host-tested. The hardware (HX711 bit-bang, two dedicated **hardware-timer-ISR step generators**, the `WireFeed` façade, a ~83 Hz `wireFeedTask`) is ESP32-coupled and bench-verified. WireFeed reads the controller's wire-break state via two new lock-free per-field accessors.

**Tech Stack:** C++17, PlatformIO, googletest (native `[env:tests]`), ESP32 hardware timers, HX711, FluidNC Pin/Config.

**Builds on (P0–P2, merged to `main`):** `EDM/Control/EdmController` (computes `_wire_break_sev`, `_feed_cap_mult`; exposes `feed_cap_pct`/`wire_break_sev` in `EdmReport`), `EDM/EdmSpindle` (owns the controller, starts background tasks via the `EdmServoTask` idiom). `IPsuLink`/`EdmController::tick()` unchanged.

**Verified FluidNC facts (from the design workflow):**
- Step generation: **dedicated hardware-timer ISR per motor** is the chosen mechanism. LEDC tone rejected (`ledcSetup` not IRAM-safe to retune from a task, jitters at high rates); RMT rejected (ESP32-S3 has `MAX_N_RMT=0`; on classic ESP32 it shares channels with the Cartesian axes). Use Timer Group 1 (leaving Group 0 to FluidNC's stepping). Mirror `esp32/StepTimer.cpp`.
- HX711: 2-wire, 24-bit two's-complement MSB-first; DOUT low = data-ready; **25 SCK pulses = ch A gain 128**, 26 = ch B gain 32, 27 = ch A gain 64; ≤80 SPS; SCK high must not exceed ~60 µs (else power-down). Use the FluidNC `Pin` class + `delay_us()`.
- Task idiom: `xTaskCreatePinnedToCore(fn, name, stack, arg, 2, nullptr, SUPPORT_TASK_CORE)`, `add_watchdog_to_task()`, `get_ms()` — mirror `EdmServoTask.cpp`.
- `EdmReport.feed_cap_pct`/`wire_break_sev` are uint8, written wholesale once per 1 kHz tick (`EdmController.cpp:214-215`); severity map sev3→cap 0.0, sev2→0.3, sev1→0.6, decay to 1.0 after 500 ms (`:88-98,140-141`). Per-field reads on the same core (`SUPPORT_TASK_CORE`) are atomic; no `snapshot()` struct copy.

**Spec:** `../specs/2026-06-13-edm-psu-integration-design.md` §5 (wire feed + tension).

**Conventions (same as P0–P2):**
- Test cmd: `export PATH="/c/ProgramData/mingw64/mingw64/bin:$HOME/.platformio/penv/Scripts:$PATH" && pio test -e tests -f '<filter>'`.
- `[H]` = host-tested googletest (RED→GREEN→refactor). `[B]` = ESP32-coupled, bench-verified, NOT host-built.
- Floats permitted inside the pure controller + the ~83 Hz task (not an ISR). Config/report boundary integer.
- Namespace for new pure types: `EDM::feed`. New dir `FluidNC/src/EDM/Feed/`.

---

## File structure (locked)

| File | Kind | Host-tested | In `tests_common` |
|---|---|---|---|
| `Feed/Hx711Calibration.h` | pure raw→N converter (header-only inline) | **yes** (via test) | header-only |
| `Feed/TensionController.h` / `.cpp` | pure feedforward+PI+collapse controller | **yes** | `+<EDM/Feed/TensionController.cpp>` |
| `Feed/ITensionSensor.h` | sensor interface (header-only) | yes | header-only |
| `Feed/Hx711.h` / `.cpp` | HX711 bit-bang + Configurable | no (bench) | excluded |
| `Feed/StepGen.h` / `.cpp` | hardware-timer-ISR step generator | no (bench) | excluded |
| `Feed/WireFeed.h` / `.cpp` | façade: controller + Hx711 + 2×StepGen + DISABLE; `tick()` | no (bench) | excluded |
| `EdmWireFeedTask.h` / `.cpp` | ~83 Hz task launcher | no (bench) | excluded |

Edits: `EDM/Control/EdmController.h` (+2 accessors), `EDM/EdmSpindle.h`/`.cpp` (own + start WireFeed, parse `wire_feed:`), `platformio.ini` (`[tests_common]` += the controller `.cpp` + 2 test files).

New host tests: `tests/EdmTensionControllerTest.cpp`, `tests/EdmHx711CalibrationTest.cpp`.

---

## Task 1 [H]: pure HX711 calibration converter

**Files:** Create `FluidNC/src/EDM/Feed/Hx711Calibration.h`, `FluidNC/tests/EdmHx711CalibrationTest.cpp`. Modify `platformio.ini`.

- [ ] **Step 1: `Hx711Calibration.h`**

```cpp
// FluidNC/src/EDM/Feed/Hx711Calibration.h
#pragma once
#include <cstdint>

namespace EDM { namespace feed {

// Sign-extend a 24-bit two's-complement HX711 sample to int32. PURE.
inline int32_t hx711SignExtend(uint32_t v24) {
    return (v24 & 0x800000u) ? int32_t(v24 | 0xFF000000u) : int32_t(v24 & 0x00FFFFFFu);
}

// Affine raw-counts -> Newtons converter. PURE value type.
struct Hx711Calibration {
    int32_t offset       = 0;        // tare counts at zero load
    float   counts_per_N = 1000.0f;  // ADC counts per Newton

    float toNewtons(int32_t raw) const {
        return (float(raw) - float(offset)) / counts_per_N;
    }
    Hx711Calibration withCalibration(int32_t newOffset, float newCountsPerN) const {
        return Hx711Calibration{ newOffset, newCountsPerN };
    }
};

}}  // namespace EDM::feed
```

- [ ] **Step 2: Failing test `EdmHx711CalibrationTest.cpp`**

```cpp
// FluidNC/tests/EdmHx711CalibrationTest.cpp
#include "gtest/gtest.h"
#include "EDM/Feed/Hx711Calibration.h"

using namespace EDM::feed;

TEST(EdmHx711Cal, SignExtendBoundaries) {
    EXPECT_EQ(hx711SignExtend(0x800000u), -8388608);
    EXPECT_EQ(hx711SignExtend(0xFFFFFFu), -1);
    EXPECT_EQ(hx711SignExtend(0x7FFFFFu),  8388607);
    EXPECT_EQ(hx711SignExtend(0x000000u),  0);
}

TEST(EdmHx711Cal, ToNewtonsAffine) {
    Hx711Calibration cal{ 84213, 4271.5f };
    EXPECT_NEAR(cal.toNewtons(84213), 0.0f, 1e-3f);
    EXPECT_NEAR(cal.toNewtons(84213 + int32_t(4271.5f * 5)), 5.0f, 1e-2f);
}
```

- [ ] **Step 3: Register in `platformio.ini`** — in `[tests_common] build_src_filter`, after the last `+<tests/Edm...>` line, add:
```
    +<tests/EdmHx711CalibrationTest.cpp>
```
(Header-only; no `.cpp` line — like `Endian.h`.)

- [ ] **Step 4: Run, verify** — `pio test -e tests -f '*EdmHx711Cal*'` (2 tests pass — the header already implements the logic, so this is GREEN immediately; that's fine for a header-only pure converter).

- [ ] **Step 5: Commit**
```bash
git add FluidNC/src/EDM/Feed/Hx711Calibration.h FluidNC/tests/EdmHx711CalibrationTest.cpp platformio.ini
git commit -m "feat(edm): pure HX711 sign-extend + raw->Newtons calibration (TDD)"
```

---

## Task 2 [H]: TensionController — full feedforward + PI + collapse + all 14 tests

**Files:** Create `FluidNC/src/EDM/Feed/TensionController.h`, `.cpp`, `FluidNC/tests/EdmTensionControllerTest.cpp`. Modify `platformio.ini`.

- [ ] **Step 1: `TensionController.h`**

```cpp
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
    float    Kp                  = 40.0f;   // steps/s per N
    float    Ki                  = 8.0f;    // steps/s per (N*s)
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
    float    commanded_feed_mm_min= -1.0f;   // <0 => use cfg.default_feed_mm_min
    uint32_t dt_ms                = 12;
};

struct TensionState {
    float    meas_f               = 0.0f;
    float    e_f                  = 0.0f;
    float    integ                = 0.0f;
    uint32_t collapse_accum_ms    = 0;
    uint32_t since_loaded_ms      = 0;
    bool     collapse_latched     = false;
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
    static TensionState reset() { return TensionState{}; }
private:
    TensionConfig _c;
};

}}  // namespace EDM::feed
```

- [ ] **Step 2: Failing test `EdmTensionControllerTest.cpp`** (all 14)

```cpp
// FluidNC/tests/EdmTensionControllerTest.cpp
#include "gtest/gtest.h"
#include "EDM/Feed/TensionController.h"

using namespace EDM::feed;

// measured at setpoint, cutting, healthy, fresh sample, 12ms tick
static TensionInput healthy() {
    TensionInput in;
    in.measured_N = 5.0f; in.meas_valid = true; in.feed_cap_mult = 1.0f;
    in.severity = 0; in.cutting = true; in.commanded_feed_mm_min = -1.0f; in.dt_ms = 12;
    return in;
}

TEST(EdmTension, ResetYieldsZeroedState) {
    TensionConfig c; TensionController t(c);
    TensionState s = TensionController::reset();
    TensionInput in = healthy(); in.cutting = false;
    TensionOutput o = t.step(s, in);
    EXPECT_NEAR(o.integrator, 0.0f, 1e-6f);
    EXPECT_FALSE(o.tension_collapse);
    EXPECT_EQ(o.feed_steps_per_s, 0);
}

TEST(EdmTension, FeedScalesWithCapHealthy) {
    TensionConfig c; TensionController t(c);
    TensionOutput o = t.step(TensionController::reset(), healthy());
    EXPECT_EQ(o.feed_steps_per_s, 100);   // 60mm/min /60 *100 steps/mm = 100
}

TEST(EdmTension, FeedScalesWithCapSev1Sev2) {
    TensionConfig c; TensionController t(c);
    TensionInput a = healthy(); a.feed_cap_mult = 0.6f;
    EXPECT_EQ(t.step(TensionController::reset(), a).feed_steps_per_s, 60);
    TensionInput b = healthy(); b.feed_cap_mult = 0.3f;
    EXPECT_EQ(t.step(TensionController::reset(), b).feed_steps_per_s, 30);
}

TEST(EdmTension, FeedZeroWhenNotCutting) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.cutting = false;
    EXPECT_EQ(t.step(TensionController::reset(), in).feed_steps_per_s, 0);
}

TEST(EdmTension, FeedZeroOnSev3Holds) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.severity = 3; in.feed_cap_mult = 0.0f;
    TensionOutput o = t.step(TensionController::reset(), in);
    EXPECT_EQ(o.feed_steps_per_s, 0);
    EXPECT_EQ(o.tension_steps_per_s, 200);   // tension_hold_sps
}

TEST(EdmTension, FeedforwardTracksFeed) {
    TensionConfig c; c.Kp = 0.0f; c.Ki = 0.0f; TensionController t(c);
    TensionOutput o = t.step(TensionController::reset(), healthy());  // measured==setpoint
    EXPECT_NEAR(o.pi_sps, 0.0f, 1e-6f);
    EXPECT_EQ(o.tension_steps_per_s, o.feed_steps_per_s);  // ff_ratio 1.0
}

TEST(EdmTension, SlackIncreasesTension) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.measured_N = 3.0f;   // below 5 -> slack
    TensionState s = TensionController::reset();
    TensionOutput o;
    for (int i = 0; i < 5; ++i) { o = t.step(s, in); s = o.next; }
    EXPECT_GT(o.e_filtered, 0.0f);
    EXPECT_GT(o.pi_sps, 0.0f);
    EXPECT_GT(o.tension_steps_per_s, int32_t(o.ff_sps));
}

TEST(EdmTension, OverTensionReducesTension) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.measured_N = 7.0f;   // above 5
    TensionState s = TensionController::reset();
    TensionOutput o;
    for (int i = 0; i < 5; ++i) { o = t.step(s, in); s = o.next; }
    EXPECT_LT(o.pi_sps, 0.0f);
}

TEST(EdmTension, DeadbandHoldsNoIntegrate) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.measured_N = 5.1f;   // |e|=0.1 < deadband 0.25
    TensionState s = TensionController::reset();
    TensionOutput o;
    for (int i = 0; i < 10; ++i) { o = t.step(s, in); s = o.next; }
    EXPECT_NEAR(o.pi_sps, 0.0f, 1e-6f);
    EXPECT_NEAR(o.integrator, 0.0f, 1e-6f);
}

TEST(EdmTension, AntiWindupClampsIntegrator) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.measured_N = 0.0f;   // huge persistent slack
    TensionState s = TensionController::reset();
    TensionOutput o;
    for (int i = 0; i < 400; ++i) { o = t.step(s, in); s = o.next; }
    EXPECT_LE(o.integrator, c.pi_clamp_sps / c.Ki + 1e-3f);   // imax = 400/8 = 50
    EXPECT_LE(o.pi_sps, c.pi_clamp_sps + 1e-3f);
}

TEST(EdmTension, IntegralFrozenWhenStale) {
    TensionConfig c; TensionController t(c);
    TensionState s = TensionController::reset();
    TensionInput load = healthy(); load.measured_N = 3.0f;
    for (int i = 0; i < 5; ++i) s = t.step(s, load).next;    // wind integ up
    float integ_before = s.integ; float meas_before = s.meas_f;
    TensionInput stale = healthy(); stale.measured_N = 3.0f; stale.meas_valid = false;
    for (int i = 0; i < 10; ++i) s = t.step(s, stale).next;
    EXPECT_NEAR(s.integ, integ_before, 1e-6f);
    EXPECT_NEAR(s.meas_f, meas_before, 1e-6f);
}

TEST(EdmTension, Sev3BleedsIntegratorAndHolds) {
    TensionConfig c; TensionController t(c);
    TensionState s = TensionController::reset();
    TensionInput load = healthy(); load.measured_N = 3.0f;
    for (int i = 0; i < 20; ++i) s = t.step(s, load).next;   // wind integ
    float integ_before = s.integ;
    ASSERT_GT(integ_before, 0.1f);
    TensionInput sev3 = healthy(); sev3.severity = 3; sev3.feed_cap_mult = 0.0f;
    TensionOutput o = t.step(s, sev3);
    EXPECT_EQ(o.feed_steps_per_s, 0);
    EXPECT_EQ(o.tension_steps_per_s, 200);
    EXPECT_NEAR(o.next.integ, integ_before * c.integ_bleed, 1e-3f);
}

TEST(EdmTension, Sev1TrimsTensionSetpoint) {
    TensionConfig c; TensionController t(c);
    TensionInput sev0 = healthy(); sev0.measured_N = 5.0f; sev0.severity = 0;
    TensionInput sev1 = healthy(); sev1.measured_N = 5.0f; sev1.severity = 1; sev1.feed_cap_mult = 0.6f;
    // sev1 effective setpoint = 5*0.85 = 4.25 -> measured 5 is now OVER -> lower tension than sev0
    TensionOutput o0 = t.step(TensionController::reset(), sev0);
    TensionOutput o1 = t.step(TensionController::reset(), sev1);
    EXPECT_LT(o1.tension_steps_per_s, o0.tension_steps_per_s);
}

TEST(EdmTension, CollapseLatchesThreeConditionDebounced) {
    TensionConfig c; TensionController t(c);
    // never-loaded baseline at 0.1 N must NOT latch collapse
    TensionState s0 = TensionController::reset();
    TensionInput idle = healthy(); idle.measured_N = 0.1f; idle.cutting = false;
    for (int i = 0; i < 20; ++i) s0 = t.step(s0, idle).next;
    EXPECT_FALSE(s0.collapse_latched);

    // load to 5 N a few ticks, then drop to 0.1 N -> latches after collapse_ms (~30ms = ~3 ticks)
    TensionState s = TensionController::reset();
    TensionInput load = healthy(); load.measured_N = 5.0f;
    for (int i = 0; i < 5; ++i) s = t.step(s, load).next;
    TensionInput drop = healthy(); drop.measured_N = 0.1f;
    TensionOutput o;
    o = t.step(s, drop); s = o.next;    // accum 12ms
    EXPECT_FALSE(o.tension_collapse);
    o = t.step(s, drop); s = o.next;    // accum 24ms
    EXPECT_FALSE(o.tension_collapse);
    o = t.step(s, drop); s = o.next;    // accum 36ms >= 30 -> latch
    EXPECT_TRUE(o.tension_collapse);
    // stays latched even if tension recovers
    TensionInput rec = healthy(); rec.measured_N = 5.0f;
    o = t.step(s, rec);
    EXPECT_TRUE(o.tension_collapse);
}
```

- [ ] **Step 3: Register in `platformio.ini`** — after the Hx711 test line, add:
```
    +<EDM/Feed/TensionController.cpp>
    +<tests/EdmTensionControllerTest.cpp>
```

- [ ] **Step 4: Run, verify FAIL** (undefined `TensionController::step`).

- [ ] **Step 5: Create `FluidNC/src/EDM/Feed/TensionController.cpp`**

```cpp
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

    // (a) Feed motor: open-loop master, gated by cutting, scaled by feed cap.
    float feed_mm_min = (in.commanded_feed_mm_min >= 0.0f) ? in.commanded_feed_mm_min
                                                           : c.default_feed_mm_min;
    float feed_sps = (feed_mm_min / 60.0f) * c.feed_steps_per_mm;
    if (!in.cutting) feed_sps = 0.0f;
    feed_sps *= in.feed_cap_mult;
    feed_sps  = clampf(feed_sps, 0.0f, c.feed_max_sps);

    // (b) Effective (severity-trimmed) setpoint.
    float sp_eff = c.setpoint_N * c.sev_tension_scale[sev];

    // (c) Measured EWMA; hold last value on a stale tick.
    if (in.meas_valid) ns.meas_f = c.tens_alpha * in.measured_N + (1.0f - c.tens_alpha) * s.meas_f;

    // (d) Feedforward: tension motor tracks the capped feed rate.
    float ff_sps = c.ff_ratio * feed_sps;

    // (e) PI on tension error; deadband + back-calc anti-windup; freeze integral on stale.
    float e   = sp_eff - ns.meas_f;
    ns.e_f    = e;
    bool in_db = std::fabs(e) < c.tens_deadband_N;
    if (c.Ki > 0.0f && !in_db && in.meas_valid) {
        ns.integ += e * dt;
        float imax = c.pi_clamp_sps / c.Ki;
        ns.integ  = clampf(ns.integ, -imax, imax);
    }
    float pi_sps = in_db ? 0.0f : (c.Kp * e + c.Ki * ns.integ);
    pi_sps = clampf(pi_sps, -c.pi_clamp_sps, c.pi_clamp_sps);

    // (f) Combine (severity FF trim).
    float tension_sps = ff_sps * c.sev_tension_scale[sev] + pi_sps;

    // (g) Three-condition latched mechanical-collapse detector.
    if (in.meas_valid) {
        bool loaded = in.measured_N > c.loaded_frac * sp_eff;
        if (loaded) {
            ns.since_loaded_ms = 0;
        } else {
            uint32_t acc = s.since_loaded_ms + in.dt_ms;
            ns.since_loaded_ms = acc < s.since_loaded_ms ? 0xFFFFFFFFu : acc;  // saturate
        }
        bool was_loaded_recently = loaded || ns.since_loaded_ms <= c.loaded_recent_ms;
        bool below = (in.measured_N < c.collapse_N) && (in.measured_N < c.collapse_frac * sp_eff);
        ns.collapse_accum_ms = (below && was_loaded_recently) ? (s.collapse_accum_ms + in.dt_ms) : 0;
        if (ns.collapse_accum_ms >= c.collapse_ms) ns.collapse_latched = true;
    }
    bool collapse = ns.collapse_latched;

    // (h) sev3 / collapse emergency: zero feed, gentle hold, bleed integrator.
    if (sev >= 3 || collapse) {
        feed_sps    = 0.0f;
        tension_sps = c.tension_hold_sps;
        ns.integ   *= c.integ_bleed;
    }

    // (i) Final clamps.
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
```

- [ ] **Step 6: Run, verify PASS** — `pio test -e tests -f '*EdmTension*'` (14 tests). Investigate any failure: prefer fixing the impl when a test encodes clear intent; adjust a test only if a threshold is mis-calibrated to the math (report it). Likely-sensitive: `Sev1TrimsTensionSetpoint` (depends on FF trim AND deadband — with sev1 sp_eff=4.25, measured 5 → e=−0.75 (>deadband) → pi<0, and FF trimmed ×0.85, so both push tension below sev0; should hold). `CollapseLatchesThreeConditionDebounced` (12 ms ticks; 3 ticks = 36 ms ≥ 30).

- [ ] **Step 7: Commit**
```bash
git add FluidNC/src/EDM/Feed/TensionController.h FluidNC/src/EDM/Feed/TensionController.cpp FluidNC/tests/EdmTensionControllerTest.cpp platformio.ini
git commit -m "feat(edm): TensionController feedforward+PI + collapse detector + sev reactions (TDD)"
```

- [ ] **Step 8: Coverage** — `pio test -e tests_coverage -f '*Edm*'` (+ `python coverage.py` if present); confirm TensionController.cpp ≥80%. Add a targeted test if a branch is uncovered.

---

## Task 3 [B]: EdmController wire-break accessors

**Files:** Modify `FluidNC/src/EDM/Control/EdmController.h` (host-built — must compile natively).

- [ ] **Step 1:** After `reportedState()` (the P2 accessor) in the `public:` section, add:
```cpp
    // P3 wire-feed read channel: single aligned-byte reads of report fields tick() writes.
    uint8_t feedCapPct()        const { return _report.feed_cap_pct; }   // 0..100
    uint8_t wireBreakSeverity() const { return _report.wire_break_sev; } // 0..3
```
- [ ] **Step 2:** `pio test -e tests -f '*Edm*'` — still green (no behavior change; EdmControllerTest unaffected). **Step 3:** `git add FluidNC/src/EDM/Control/EdmController.h && git commit -m "feat(edm): controller feedCapPct/wireBreakSeverity accessors for wire feed"`

---

## Task 4 [B]: HX711 hardware driver + ITensionSensor

**Files:** Create `FluidNC/src/EDM/Feed/ITensionSensor.h`, `FluidNC/src/EDM/Feed/Hx711.h`, `.cpp`. ESP32-coupled; NOT in `tests_common`.

- [ ] **Step 1:** READ `FluidNC/src/Pin.h` (the `Pin` class: `read()`, `synchronousWrite()`/`write()`, `setAttr`, `Pins::PinAttributes`), `FluidNC/include/Driver/delay_usecs.h` (`delay_us`), and an existing `Configuration::Configurable` with pin items (e.g. a Spindle) for the `group()`/`validate()` idiom. Confirm exact APIs.

- [ ] **Step 2:** Create `ITensionSensor.h`:
```cpp
// FluidNC/src/EDM/Feed/ITensionSensor.h
#pragma once
#include <cstdint>
namespace EDM { namespace feed {
class ITensionSensor {
public:
    virtual ~ITensionSensor() = default;
    virtual bool    dataReady() const = 0;   // non-blocking: true when a sample is ready
    virtual int32_t readRaw()         = 0;    // sign-extended 24-bit
};
}}  // namespace EDM::feed
```

- [ ] **Step 3:** Create `Hx711.{h,cpp}` implementing `ITensionSensor` + `Configuration::Configurable`:
  - Members: `Pin _dout, _sck;` `int _gain_pulses = 1;` (128→1, 64→3, 32→2); `Hx711Calibration _cal;` `portMUX_TYPE _mux`.
  - `begin()`: `_dout.setAttr(Input [+PullUp])`, `_sck.setAttr(Output)`, `_sck.off()`.
  - `bool dataReady() const { return !_dout.read(); }`
  - `int32_t readRaw()`: `portENTER_CRITICAL(&_mux);` clock 24 bits MSB-first (`_sck.write(true); delay_us(1); v=(v<<1)|_dout.read(); _sck.write(false); delay_us(1);`), then `_gain_pulses` extra pulses; `portEXIT_CRITICAL`; `return hx711SignExtend(v);` (critical section ≈50 µs, well under the 60 µs power-down limit).
  - `void tare(int n=16)`: average n raws → `_cal = _cal.withCalibration(avg, _cal.counts_per_N)`.
  - `float toNewtons(int32_t raw) const { return _cal.toNewtons(raw); }` and a `setCalibration`.
  - `group(handler)`: items `dout_pin`, `sck_pin`, `tare_offset`, `scale_counts_per_N`, `channel_gain`. `validate()`: both pins set; `channel_gain ∈ {128,64,32}`.

- [ ] **Step 4:** `pio test -e tests -f '*Edm*'` still green (not host-built). **Step 5:** Bench (later, §accept): tare + known masses ±2%; scope 25 SCK pulses, SCK-high <60 µs. **Step 6:** `git add FluidNC/src/EDM/Feed/ITensionSensor.h FluidNC/src/EDM/Feed/Hx711.h FluidNC/src/EDM/Feed/Hx711.cpp && git commit -m "feat(edm): HX711 load-cell driver (bit-bang, non-blocking, Configurable) (ESP32)"`

---

## Task 5 [B]: StepGen — hardware-timer-ISR step generator

**Files:** Create `FluidNC/src/EDM/Feed/StepGen.h`, `.cpp`. ESP32-coupled.

- [ ] **Step 1:** READ `FluidNC/esp32/StepTimer.cpp` (the `timerBegin`/`timerAttachInterrupt`/`timerAlarmWrite` or the ESP-IDF `gptimer` pattern this codebase uses) and `Pin.h`. Confirm the timer API (Arduino-ESP32 `hw_timer_t*` vs IDF `gptimer`). Use **Timer Group 1** (leave Group 0 to FluidNC stepping).

- [ ] **Step 2:** Implement `StepGen`:
```cpp
class StepGen {
public:
    void begin(Pin step, Pin dir, Pin disable, int timer_num);
    void setRate(int32_t steps_per_s);   // sign->DIR; 0 -> park (no pulses), motor holds
    void enable(bool on);                // drives DISABLE pin directly (true e-stop)
private:
    static void IRAM_ATTR onTimer(void* self);
    Pin _step, _dir, _disable;
    std::atomic<uint32_t> _half_period_ticks{0};  // 0 == idle
    std::atomic<bool>     _dir_level{false};
    volatile bool         _step_level = false;
};
```
  - `setRate`: set `_dir_level` from sign; `half = (sps==0)?0:timer_hz/(2*abs(sps))`; store atomically. ISR re-reads `_half_period_ticks` each fire; if 0, emit nothing; else toggle STEP and re-arm the alarm at `half`. Single-writer(task)/single-reader(ISR) → no mutex.
  - `enable(bool)`: `_disable.synchronousWrite(!on)` (DISABLE active-low/high per board; confirm polarity).
  - DIR set at interval start before the STEP pulse (short dir delay).
  - Mark any uncertain timer API `// TODO(P4): confirm <symbol>`.

- [ ] **Step 3:** `pio test -e tests -f '*Edm*'` still green. **Step 4:** Bench: 100/1000/4000 sps within ±1%, DIR sign, park at 0, DISABLE assert. **Step 5:** `git add FluidNC/src/EDM/Feed/StepGen.h FluidNC/src/EDM/Feed/StepGen.cpp && git commit -m "feat(edm): StepGen hardware-timer-ISR step generator (ESP32)"`

---

## Task 6 [B]: WireFeed façade

**Files:** Create `FluidNC/src/EDM/Feed/WireFeed.h`, `.cpp`. ESP32-coupled.

- [ ] **Step 1:** Implement `WireFeedConfig` (`Configuration::Configurable` parsing the `wire_feed:` schema — motor pins/steps_per_mm/feed/max_sps, tension_control gains, severity_tension_scale, load_cell pins+cal, collapse params) and `WireFeed`:
  - Owns `TensionController _controller; TensionState _state; Hx711 _hx711; Hx711Calibration _cal; StepGen _feedGen, _tensionGen; EDM::EdmController* _ctl;` `uint32_t _last_ms; int32_t _last_raw; float _last_N; bool _collapse_sticky;`.
  - `begin(EdmController* ctl)`: store ctl, `_hx711.begin()`, `_feedGen.begin(...)`, `_tensionGen.begin(...)` from config; build `TensionConfig` from `WireFeedConfig`.
  - `tick(uint32_t now_ms)` per the design:
    1. `bool fresh = _hx711.dataReady(); if (fresh) { _last_raw = _hx711.readRaw(); _last_N = _cal.toNewtons(_last_raw); }`
    2. read `cap = _ctl->feedCapPct(); sev = _ctl->wireBreakSeverity(); st = _ctl->reportedState();` `cutting = st ∈ {Cutting,TouchOff,Hold,BreakRelief}`.
    3. build `TensionInput` (measured=_last_N, meas_valid=fresh, feed_cap_mult=cap/100, severity=sev, cutting, dt_ms=now_ms−_last_ms); `_last_ms=now_ms`.
    4. `TensionOutput o = _controller.step(_state, in); _state = o.next;`
    5. `_feedGen.setRate(o.feed_steps_per_s); _tensionGen.setRate(o.tension_steps_per_s);`
    6. `if (sev>=3 || o.tension_collapse) { _feedGen.enable(false); if (o.tension_collapse) _collapse_sticky=true; } else if (cutting) { _feedGen.enable(true); }`

- [ ] **Step 2:** `pio test -e tests -f '*Edm*'` still green. **Step 3:** `git add FluidNC/src/EDM/Feed/WireFeed.h FluidNC/src/EDM/Feed/WireFeed.cpp && git commit -m "feat(edm): WireFeed facade (controller+HX711+2 StepGen+DISABLE) (ESP32)"`

---

## Task 7 [B]: wireFeedTask + EdmSpindle wiring + YAML

**Files:** Create `FluidNC/src/EDM/EdmWireFeedTask.h`, `.cpp`; modify `FluidNC/src/EDM/EdmSpindle.h`, `.cpp`.

- [ ] **Step 1:** `EdmWireFeedTask.{h,cpp}` mirroring `EdmServoTask.cpp`:
```cpp
static void wireFeedLoop(void* arg) {
    add_watchdog_to_task();
    auto* wf = static_cast<EDM::feed::WireFeed*>(arg);
    for (;;) { wf->tick(get_ms()); vTaskDelay(12); }   // ~83 Hz, aligns to 80 SPS HX711
}
void startWireFeedTask(EDM::feed::WireFeed* wf) {
    xTaskCreatePinnedToCore(wireFeedLoop, "wire_feed", 4096, wf, 2, nullptr, SUPPORT_TASK_CORE);
}
```
- [ ] **Step 2:** In `EdmSpindle.h` add members `EDM::feed::WireFeedConfig _feed_cfg; std::unique_ptr<EDM::feed::WireFeed> _feed;`. In `EdmSpindle.cpp::init()` after `startEdmServoTask(_ctl.get())`: construct `_feed`, `_feed->begin(_ctl.get())`, `startWireFeedTask(_feed.get())`. In `group()` add `handler.section("wire_feed", _feed_cfg)`.
- [ ] **Step 3:** Add the full `wire_feed:` YAML section (from the spec §5) to the board config / an example config file.
- [ ] **Step 4:** `pio test -e tests -f '*Edm*'` still green. **Step 5:** `git add FluidNC/src/EDM/EdmWireFeedTask.h FluidNC/src/EDM/EdmWireFeedTask.cpp FluidNC/src/EDM/EdmSpindle.h FluidNC/src/EDM/EdmSpindle.cpp && git commit -m "feat(edm): wireFeedTask + EdmSpindle wiring + wire_feed YAML (ESP32)"`

---

## Task 8 [B]: Integrated bench acceptance (on-target, dry-run)

No code. Record results in the PR. Needs hardware (gated by P0 pin map). With `EdmSpindle._use_sim=true`:
- [ ] HX711: tare + known masses (0/100/500 g) track `m·9.80665` ±2%; scope 25 SCK pulses, SCK-high <60 µs under stepping load; `dataReady()` ≈80 Hz.
- [ ] StepGen: 100/1000/4000 sps within ±1%; DIR sign; `setRate(0)` parks; `enable(false)` asserts DISABLE. (Decision point: if I2SO STEP timing is too coarse at high rate, switch STEP to native GPIO.)
- [ ] Closed loop: live wire + real load cell settles within ±10% of 5 N, no sustained oscillation.
- [ ] **Feedforward transient suppression** (the winning architecture's advantage): step feed 60→120→60 mm/min; tension stays within ±10% of setpoint throughout.
- [ ] Wire-break: inject sev1/2/3 via the SimPsuLink wire-break path; feed steps 100→60→30→0, tension trims per the severity table, at sev3 the feed DISABLE asserts + tension drops to ~200 sps hold; normal resume after the 500 ms decay (no windup kick).
- [ ] Mechanical collapse: snip/slack the wire mid-cut; `tension_collapse` latches within ~30–50 ms, feed stops, DISABLE asserts, sticky flag set; no false collapse during idle/startup before load.
- [ ] **Commit** a bench-results note.

---

## Phase P3 exit criteria
- `pio test -e tests -f '*Edm*'` green: TensionController (14) + Hx711Calibration (2) host tests + all P0–P2; ≥80% coverage on TensionController.cpp.
- Pure controller carries all the control logic; hardware (HX711, StepGen, WireFeed, task) is bench-verified.
- WireFeed reads EdmController wire-break state via the new lock-free accessors; sev1/2/3 + collapse handled; feedforward suppresses tension transients.
- `IPsuLink`/`EdmController::tick()` unchanged; existing tests unaffected.

## Roadmap after P3
- **P4** — live integration: wire all three EDM tasks (servo/motion/wirefeed) on real hardware, `Report.cpp`/`Channel::autoReport()` hookup + `$EDM/*` settings + full `edm:` YAML parse, real-PSU bring-up, the IPsuLink cross-core spinlock/ring-buffer (P0/P1 TODOs), and the SamplerConfig-from-YAML (P2 TODO). Then the joint PSU acceptance cut.
- **Sub-project A** (XYUV kinematics) and **C** (customer-facing WebUI) remain; C's `EdmReport` contract is frozen.

## Self-review notes
- **Spec coverage (§5):** feed motor (open-loop, cap-scaled) + tension motor (FF+PI) — T2; HX711 (pure converter T1 + driver T4); two RMT/timer step generators — T5; wire-break severity reactions + collapse — T2 (logic) + T6 (wiring); 100 Hz→83 Hz task — T7; config — T7.
- **Grafts applied:** feedforward winner (T2 step d/f); C's latched 3-condition collapse detector (T2 step g, test 14); A's hard DISABLE-pin e-stop on sev3/collapse (T5 `enable`, T6 tick step 6); A's 83 Hz cadence (T7); integrator bleed on emergency (T2 step h, test 12).
- **No placeholders in host code:** T1–T2 complete pure code + 16 tests. T3 trivial accessors. T4–T7 ESP32-coupled (firmware-build + bench verified, not googletest) with exact API-confirmation steps — consistent with P0/P1/P2 hardware tasks.
- **Type consistency:** `TensionConfig/Input/State/Output`, `Hx711Calibration`, `ITensionSensor`, `StepGen`, `WireFeed` used identically; accessor names (`feedCapPct`/`wireBreakSeverity`) match the synthesis.
- **Concurrency:** the ~83 Hz task is the sole caller of the controller + step generators; reads EdmController report fields per-field (atomic uint8 on one core); StepGen uses atomics single-writer/single-reader.
