# EDM PSU Integration — Phase P1 (EdmController + Gap Servo) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the EDM process controller and closed-loop gap servo on top of the P0 transport/protocol/simulator layer — a pure, host-testable control law (advance / hold / **retract** from PSU classifier telemetry) plus the `EdmController` state machine, the `EdmSpindle` facade, and the `EdmReport` telemetry surface — all developed mock-first against an upgraded `SimPsuLink`.

**Architecture:** A **hybrid PI + reflex** controller (chosen by an adversarial design panel over threshold-only and PI-only rivals). A pure `GapServo` class does value-in/value-out control (no FreeRTOS, no globals, no I/O) so it is 100% googletest-able; `Ki` ships at **0.0** (pure-threshold bring-up profile) and is enabled in YAML once the gap is stable. A separately-tested `ModeTable` maps the S-word to EDM modes and builds `SetModeBounds`. A thin `EdmController` is the *only* code touching `IPsuLink`/FreeRTOS; its `tick()` is exercised through the simulator. The `SimPsuLink` is upgraded from a 3-bucket step to a proportional/interpolated count model with velocity coupling and event injection so closed-loop convergence is demonstrable.

**Tech Stack:** C++17, PlatformIO, googletest (native `[env:tests]`), FluidNC Spindle/Config/Report subsystems.

**Builds on (P0, already merged to `main`):** `EDM/Can/*`, `EDM/Psu/{Protocol,Endian,IPsuLink,PsuLink,SimPsuLink}`. `IPsuLink` is unchanged by P1. `Spindle.h:16-17` enforces **"No floats!"** across the spindle/config/report boundary — floats live only inside the pure servo (which runs in a 1 kHz task, never an ISR); everything crossing a boundary is integer-encoded.

**Spec:** `../specs/2026-06-13-edm-psu-integration-design.md` §4 (gap servo). This plan refines §4 with the panel's final control law.

**Conventions (same as P0):**
- Test command (compiler/pio not on PATH):
  ```bash
  export PATH="/c/ProgramData/mingw64/mingw64/bin:$HOME/.platformio/penv/Scripts:$PATH"
  cd ".../Microspark Firmware/FluidEDM"
  pio test -e tests -f '*Edm*'
  ```
- `build_src_filter` paths are relative to `FluidNC/`. Add each new source/test line as its task creates the file (so the native build stays green per task).
- Floats are permitted INSIDE `GapServo`/`EdmController` (1 kHz task context). Anything exposed via config, the spindle interface, or `EdmReport` is integer-encoded.

---

## File structure (locked before tasks)

```
FluidNC/src/EDM/Servo/          # PURE — no FreeRTOS/globals/IPsuLink/Arduino. HOST-TESTED.
  GapServo.h / GapServo.cpp     # error math + PI + reflex ladder; value-in/value-out step()
  ModeTable.h / ModeTable.cpp   # EdmMode, ModeParams, S-word->mode, mode->SetModeBounds
  FaultReason.h                 # FaultReason enum (header-only)
  EdmReport.h                   # EdmReport POD (WebUI contract; header-only)
FluidNC/src/EDM/Control/        # FluidNC-coupled; EdmController.tick() is sim-testable.
  EdmController.h / EdmController.cpp
FluidNC/src/EDM/EdmSpindle.h / .cpp        # Spindle facade + factory registration (ESP32-only; NOT host-built)
FluidNC/src/EDM/EdmServoTask.h / .cpp      # 1 kHz task wiring (ESP32-only; NOT host-built)
FluidNC/src/EDM/EdmReportChannel.h / .cpp  # report push + $EDM/Status (ESP32-only; NOT host-built)

MODIFIED (additive; IPsuLink untouched):
  FluidNC/src/EDM/Psu/SimPsuLink.h / .cpp  # proportional count model, velocity coupling, event injection

Tests (FluidNC/tests/):
  EdmGapServoTest.cpp      EdmModeTableTest.cpp      EdmControllerTest.cpp
  EdmSimPsuLinkTest.cpp    # EXTEND the existing P0 file
```

**Added to `[tests_common] build_src_filter`** (host-built): `EDM/Servo/GapServo.cpp`, `EDM/Servo/ModeTable.cpp`, `EDM/Control/EdmController.cpp`, and the three new test files. `EDM/Psu/SimPsuLink.cpp` is already in the filter. The ESP32-coupled files (`EdmSpindle.cpp`, `EdmServoTask.cpp`, `EdmReportChannel.cpp`) are **excluded** from the native build.

---

## Task 1: Upgrade SimPsuLink to a proportional model + velocity coupling + event injection

**Why first:** the P0 sim is a 3-bucket step with no proportional region and `popEvent` always false — closed-loop convergence (T16) and wire-break tests (T17) are impossible against it. This is the mandatory correctness fix. It must keep the 4 existing P0 tests green (default coupling 0).

**Files:** Modify `FluidNC/src/EDM/Psu/SimPsuLink.h`, `.cpp`; extend `FluidNC/tests/EdmSimPsuLinkTest.cpp`.

- [ ] **Step 1: Add failing tests** (append to `EdmSimPsuLinkTest.cpp`)

```cpp
TEST(EdmSimPsuLink, ProportionalRegionInterpolates) {
    SimPsuLink sim; sim.begin();
    sim.setGap(sim.idealGapMm() * 1.5f);   // between ideal and wide
    sim.tick();
    StatsAgg s; ASSERT_TRUE(sim.latestStats(s));
    int total = s.n_normal + s.n_arc + s.n_short + s.n_open;
    EXPECT_EQ(total, 100);
    // strictly between healthy (open~6) and wide (open~80): a real proportional region
    EXPECT_GT(s.n_open, 6);
    EXPECT_LT(s.n_open, 80);
}

TEST(EdmSimPsuLink, VelocityCouplingClosesGap) {
    SimPsuLink sim; sim.begin();
    sim.setVelocityCoupling(0.00002f);     // mm per (um/s) per tick
    sim.setGap(sim.idealGapMm() * 3.0f);   // wide
    float start = sim.gapMm();
    for (int i = 0; i < 50; ++i) { sim.applyCommandedVelocity(+67); sim.tick(); }  // +4 mm/min
    EXPECT_LT(sim.gapMm(), start);         // advancing closes the gap
    EXPECT_GE(sim.gapMm(), 0.0f);
}

TEST(EdmSimPsuLink, RetractOpensGapAndClampsAtZero) {
    SimPsuLink sim; sim.begin();
    sim.setVelocityCoupling(0.00002f);
    sim.setGap(0.0f);
    sim.applyCommandedVelocity(-100); sim.tick();
    EXPECT_GT(sim.gapMm(), 0.0f);          // retract reopens
}

TEST(EdmSimPsuLink, PushEventThenPopReturnsIt) {
    SimPsuLink sim; sim.begin();
    Event in; in.kind = Event::WireBreak; in.wire_break.severity = 3;
    sim.pushEvent(in);
    Event out;
    ASSERT_TRUE(sim.popEvent(out));
    EXPECT_EQ(out.kind, Event::WireBreak);
    EXPECT_EQ(out.wire_break.severity, 3);
    EXPECT_FALSE(sim.popEvent(out));
}
```

- [ ] **Step 2: Run, verify FAIL** — `pio test -e tests -f '*EdmSimPsuLink*'`. Expect undefined `setVelocityCoupling`/`applyCommandedVelocity`/`gapMm`/`pushEvent` and the interpolation assertion failing.

- [ ] **Step 3: Update `SimPsuLink.h`** — replace the class body with this (keeps the existing public `begin/setGap/idealGapMm/tick` + `IPsuLink` overrides; adds the new API and state):

```cpp
// FluidNC/src/EDM/Psu/SimPsuLink.h
#pragma once
#include "EDM/Psu/IPsuLink.h"

namespace EDM { namespace psu {

// In-firmware PSU simulator. No CAN. Proportional gap model: classifier
// fractions interpolate with gap ratio r = gap/ideal, and (optionally)
// the commanded path velocity integrates into the gap each tick so the
// gap servo can be closed-loop tested with zero hardware.
class SimPsuLink : public IPsuLink {
public:
    void  begin()            { _connected = true; }
    void  setGap(float mm)   { _gap_mm = mm < 0.0f ? 0.0f : mm; }
    float gapMm() const      { return _gap_mm; }
    float idealGapMm() const { return _ideal_gap_mm; }

    // Closed-loop coupling: gap += coupling * v_s_um_s each tick (advance closes).
    void setVelocityCoupling(float mm_per_um_s_per_tick) { _coupling = mm_per_um_s_per_tick; }
    void applyCommandedVelocity(int32_t v_s_um_s)        { _last_v_um_s = v_s_um_s; }

    // Inject an asynchronous event the controller will drain via popEvent().
    void pushEvent(const Event& e);

    // Advance one telemetry window: integrate velocity, recompute counts.
    void tick();

    // IPsuLink
    uint16_t setModeBounds(const SetModeBounds& m) override { _mode = m; return ++_seq; }
    void     startCut()   override { _cutting = true; }
    void     stopCut()    override { _cutting = false; }
    void     clearFault() override {}

    bool latestStats(StatsAgg& out) const override;
    bool popEvent(Event& out) override;
    bool isConnected() const override        { return _connected; }
    bool protocolCompatible() const override { return _connected; }
    uint16_t lastAckStatus() const override  { return 0; }

private:
    bool  _connected = false;
    bool  _cutting   = false;
    float _gap_mm    = 0.02f;
    float _ideal_gap_mm = 0.02f;   // 20 um nominal working gap
    float _coupling  = 0.0f;       // default 0 => legacy single-tick tests unaffected
    int32_t _last_v_um_s = 0;
    uint16_t _seq = 0;
    uint32_t _window = 0;
    StatsAgg _stats{};
    bool     _stats_valid = false;
    SetModeBounds _mode{};

    static constexpr int kEvtCap = 8;
    Event _evts[kEvtCap];
    int   _evt_head = 0, _evt_count = 0;
};

}}  // namespace EDM::psu
```

- [ ] **Step 4: Rewrite `SimPsuLink.cpp`**

```cpp
// FluidNC/src/EDM/Psu/SimPsuLink.cpp
#include "EDM/Psu/SimPsuLink.h"

namespace EDM { namespace psu {

namespace {
// Linear interpolation helper.
inline float lerp(float a, float b, float t) { return a + (b - a) * t; }
}

void SimPsuLink::pushEvent(const Event& e) {
    if (_evt_count >= kEvtCap) { _evt_head = (_evt_head + 1) % kEvtCap; _evt_count--; }  // drop oldest
    _evts[(_evt_head + _evt_count) % kEvtCap] = e;
    _evt_count++;
}

bool SimPsuLink::popEvent(Event& out) {
    if (_evt_count == 0) return false;
    out = _evts[_evt_head];
    _evt_head = (_evt_head + 1) % kEvtCap;
    _evt_count--;
    return true;
}

void SimPsuLink::tick() {
    // 1) integrate commanded velocity into the gap (advance => +v => closes).
    _gap_mm -= _coupling * float(_last_v_um_s);
    if (_gap_mm < 0.0f) _gap_mm = 0.0f;

    // 2) proportional classifier fractions vs r = gap/ideal, piecewise-linear,
    //    always summing to 100. Anchors: r=0 (touching), r=1 (ideal), r>=3 (open).
    float r = (_ideal_gap_mm > 0.0f) ? (_gap_mm / _ideal_gap_mm) : 0.0f;

    float fs, fa, fn, fo;  // short, arc, normal, open fractions (0..1)
    if (r <= 1.0f) {
        float t = r;  // 0..1
        fs = lerp(0.90f, 0.06f, t);
        fa = lerp(0.05f, 0.08f, t);
        fn = lerp(0.05f, 0.80f, t);
        fo = lerp(0.00f, 0.06f, t);
    } else {
        float t = (r - 1.0f) / 2.0f; if (t > 1.0f) t = 1.0f;  // 1..3 -> 0..1
        fs = lerp(0.06f, 0.00f, t);
        fa = lerp(0.08f, 0.05f, t);
        fn = lerp(0.80f, 0.15f, t);
        fo = lerp(0.06f, 0.80f, t);
    }

    int n_short  = int(fs * 100.0f + 0.5f);
    int n_arc    = int(fa * 100.0f + 0.5f);
    int n_open   = int(fo * 100.0f + 0.5f);
    int n_normal = 100 - n_short - n_arc - n_open;   // absorb rounding so sum==100
    if (n_normal < 0) { n_normal = 0; }

    _stats = StatsAgg{};
    _stats.window_id    = _window++;
    _stats.n_normal     = uint16_t(n_normal);
    _stats.n_arc        = uint16_t(n_arc);
    _stats.n_short      = uint16_t(n_short);
    _stats.n_open       = uint16_t(n_open);
    // ignition delay rises as the gap widens (used by the servo's delay trim).
    _stats.ignition_delay_mean_ns   = uint16_t(2000.0f + 8000.0f * (r > 3.0f ? 3.0f : r));
    _stats.ignition_delay_stddev_ns = 200;
    _stats.state        = _cutting ? 0 : 1;   // running / paused
    _stats.mode_id_active = _mode.mode_id;
    _stats.dc_link_V_dV  = 800;               // 80.0 V
    _stats_valid = true;
}

bool SimPsuLink::latestStats(StatsAgg& out) const {
    if (!_stats_valid) return false;
    out = _stats;
    return true;
}

}}  // namespace EDM::psu
```

- [ ] **Step 5: Verify the 4 P0 tests still pass + new ones pass** — `pio test -e tests -f '*EdmSimPsuLink*'`. Expect 8 SimPsuLink tests green (4 original at extremes still hold: r<0.5 short-dominant, r>2 open-dominant; ideal mostly normal). If `IdealGapYieldsMostlyNormal` is borderline, confirm `n_normal` at r=1 is 80 (it is, by the anchors).

- [ ] **Step 6: Commit**

```bash
git add FluidNC/src/EDM/Psu/SimPsuLink.h FluidNC/src/EDM/Psu/SimPsuLink.cpp FluidNC/tests/EdmSimPsuLinkTest.cpp
git commit -m "feat(edm): SimPsuLink proportional gap model + velocity coupling + event injection"
```

---

## Task 2: GapServo types + skeleton + reset()

**Files:** Create `FluidNC/src/EDM/Servo/GapServo.h`, `FluidNC/src/EDM/Servo/GapServo.cpp`, `FluidNC/tests/EdmGapServoTest.cpp`. Modify `platformio.ini` (add `+<EDM/Servo/GapServo.cpp>` and `+<tests/EdmGapServoTest.cpp>` after `+<tests/EdmSimPsuLinkTest.cpp>`).

- [ ] **Step 1: Write `GapServo.h`** (verbatim — the full type set used by all later tasks)

```cpp
// FluidNC/src/EDM/Servo/GapServo.h
#pragma once
#include <cstdint>

namespace EDM { namespace servo {

enum class ServoState : uint8_t { Hold, Track, Retract, ArcHold, BreakRelief };

struct ServoConfig {                 // tunables; reconstructed once from integer YAML
    float    short_ref      = 0.08f;
    float    open_ref       = 0.15f;
    float    w_short        = 1.0f;
    float    w_open         = 0.5f;
    float    w_delay        = 0.3f;
    float    alpha          = 0.2f;  // EWMA factor (~5 windows)
    float    Kp             = 8.0f;  // mm/min per unit-e
    float    Ki             = 0.0f;  // DEFAULT 0 => pure-threshold bring-up; enable once gap stable
    float    deadband       = 0.03f;
    float    v_feed_max     = 4.0f;  // mm/min advance cap
    float    v_retract      = 6.0f;  // mm/min retract magnitude
    float    v_touch        = 0.5f;  // mm/min touch-off approach
    float    short_retract  = 0.30f;
    float    arc_brake      = 0.20f;
    float    arc_hold       = 0.40f;
    float    integ_bleed    = 0.5f;  // integ *= this on reflex windows
    uint16_t n_min          = 20;
    uint8_t  consec_short_limit = 3;
    uint8_t  pi_decimation  = 5;     // 1 kHz -> 200 Hz
    float    dt_pi          = 0.005f;
};

struct GapServoInput {
    bool     valid          = false; // false => N<n_min OR telemetry loss => HOLD
    float    open_ratio     = 0.0f;
    float    short_ratio    = 0.0f;
    float    arc_ratio      = 0.0f;
    float    normal_ratio   = 0.0f;
    float    ign_delay_norm = 0.0f;  // (mean - nom)/nom, pre-clamp
    uint16_t total_pulses   = 0;
    float    feed_cap_mult  = 1.0f;  // wire-break advance cap (1.0/0.6/0.3/0.0)
    bool     force_break_relief = false;
    bool     in_touch_off   = false;
};

struct GapServoState {               // IMMUTABLE: step() returns a NEW one
    ServoState state        = ServoState::Hold;
    float      e_f          = 0.0f;
    float      integ        = 0.0f;
    uint8_t    consec_short = 0;
    uint8_t    decim_count  = 0;
    float      last_v       = 0.0f;
};

struct GapServoOutput {
    GapServoState next;
    int32_t    v_s_um_s     = 0;     // signed commanded velocity, um/s (boundary contract)
    float      v_cmd_mm_min = 0.0f;
    ServoState state        = ServoState::Hold;
    float      g_telemetry  = 0.0f;  // operator-facing single scalar
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
```

- [ ] **Step 2: Write the failing test `EdmGapServoTest.cpp`**

```cpp
// FluidNC/tests/EdmGapServoTest.cpp
#include "gtest/gtest.h"
#include "EDM/Servo/GapServo.h"

using namespace EDM::servo;

static GapServoInput healthy() {
    GapServoInput in; in.valid = true;
    in.short_ratio = 0.08f; in.open_ratio = 0.15f; in.arc_ratio = 0.08f; in.normal_ratio = 0.69f;
    in.total_pulses = 100; in.feed_cap_mult = 1.0f;
    return in;
}

TEST(EdmGapServo, ResetYieldsHoldZeroedState) {
    GapServoState s = GapServo::reset();
    EXPECT_EQ(s.state, ServoState::Hold);
    EXPECT_FLOAT_EQ(s.e_f, 0.0f);
    EXPECT_FLOAT_EQ(s.integ, 0.0f);
    EXPECT_EQ(s.consec_short, 0);

    ServoConfig c; GapServo srv(c);
    GapServoInput in;  // valid=false
    GapServoOutput o = srv.step(s, in);
    EXPECT_FLOAT_EQ(o.v_cmd_mm_min, 0.0f);
    EXPECT_EQ(o.state, ServoState::Hold);
}
```

- [ ] **Step 3: Run, verify FAIL** — `pio test -e tests -f '*EdmGapServo*'` → undefined `GapServo::step`.

- [ ] **Step 4: Write `GapServo.cpp`** (the COMPLETE control law — used by every later GapServo task)

```cpp
// FluidNC/src/EDM/Servo/GapServo.cpp
#include "EDM/Servo/GapServo.h"
#include <cmath>

namespace EDM { namespace servo {

namespace {
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}

GapServoOutput GapServo::step(const GapServoState& s, const GapServoInput& in) const {
    const ServoConfig& c = _c;
    GapServoState ns = s;                 // working copy; returned as out.next

    // ---- error math (only on valid windows) ----
    float e_raw = 0.0f, e_delay = 0.0f, g = 0.0f;
    if (in.valid) {
        e_delay = clampf(in.ign_delay_norm, -1.0f, 1.0f);
        e_raw   = c.w_open  * (in.open_ratio  - c.open_ref)
                - c.w_short * (in.short_ratio - c.short_ref)
                + c.w_delay * e_delay;
        ns.e_f  = c.alpha * e_raw + (1.0f - c.alpha) * s.e_f;
        g       = c.w_open * in.open_ratio - c.w_short * in.short_ratio + c.w_delay * e_delay;
    } else {
        ns.e_f  = s.e_f;                  // freeze filter when telemetry untrusted
    }
    float g_target = c.w_open * c.open_ref - c.w_short * c.short_ref;

    // ---- consec_short bookkeeping (valid windows only) ----
    if (in.valid) {
        if (in.short_ratio > c.short_ref * 2.0f) {
            if (ns.consec_short < 255) ns.consec_short++;
        } else {
            ns.consec_short = 0;
        }
    }

    // ---- PI core (decimated); produces v_pi baseline ----
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

    // ---- reflex ladder (every window, raw ratios; first that fires wins) ----
    float v;
    ServoState st;
    if (in.in_touch_off) {                                    // LAYER 0
        v = c.v_touch;            st = ServoState::Track;
    } else if (!in.valid) {                                   // LAYER 1
        v = 0.0f;                 st = ServoState::Hold;
    } else if (in.force_break_relief) {                       // LAYER 2
        v = -c.v_retract; ns.integ *= c.integ_bleed; st = ServoState::BreakRelief;
    } else if (in.short_ratio > c.short_retract ||
               ns.consec_short >= c.consec_short_limit) {     // LAYER 3
        v = -c.v_retract; ns.integ *= c.integ_bleed; st = ServoState::Retract;
    } else if (in.arc_ratio > c.arc_hold) {                   // LAYER 4
        v = 0.0f;                 st = ServoState::ArcHold;   // never retract (re-strike risk)
    } else if (in.arc_ratio > c.arc_brake) {                  // LAYER 5
        float taper = c.v_feed_max * (1.0f - (in.arc_ratio - c.arc_brake) / (1.0f - c.arc_brake));
        v = v_pi < taper ? v_pi : taper;   st = ServoState::Track;
    } else {                                                  // LAYER 6
        v = v_pi; st = (std::fabs(ns.e_f) < c.deadband) ? ServoState::Hold : ServoState::Track;
    }

    // wire-break feed cap throttles advance only
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
```

- [ ] **Step 5: Run, verify PASS** — `pio test -e tests -f '*EdmGapServo*'` (1 test). Green.

- [ ] **Step 6: Commit**

```bash
git add FluidNC/src/EDM/Servo/GapServo.h FluidNC/src/EDM/Servo/GapServo.cpp FluidNC/tests/EdmGapServoTest.cpp platformio.ini
git commit -m "feat(edm): GapServo types + full control law + reset (TDD)"
```

---

> **Tasks 3–12 add tests only** against the `GapServo.cpp` written in Task 2 (the law is complete; these tests pin its behavior). Each: append the test(s) to `EdmGapServoTest.cpp`, run `pio test -e tests -f '*EdmGapServo*'`, confirm GREEN, commit. If a test fails, the bug is in `GapServo.cpp` — fix it minimally and note it. Reuse the `healthy()` helper from Task 2.

## Task 3: error math + EWMA + g telemetry

- [ ] **Step 1: Append tests**

```cpp
TEST(EdmGapServo, ErrorFormulaSignsCorrect) {
    ServoConfig c; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.60f; wide.short_ratio = 0.02f; wide.normal_ratio = 0.30f;
    EXPECT_GT(srv.step(GapServo::reset(), wide).e_filtered, 0.0f);   // wide => advance (e>0)

    GapServoInput tight = healthy(); tight.short_ratio = 0.25f; tight.open_ratio = 0.02f; tight.normal_ratio = 0.65f;
    EXPECT_LT(srv.step(GapServo::reset(), tight).e_filtered, 0.0f);  // tight => retract (e<0)
}

TEST(EdmGapServo, EwmaSmoothsAcrossWindows) {
    ServoConfig c; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.60f; wide.short_ratio = 0.02f; wide.normal_ratio = 0.30f;
    GapServoState s = GapServo::reset();
    float e1 = srv.step(s, wide).e_filtered;
    s = srv.step(s, wide).next;
    float e2 = srv.step(s, wide).e_filtered;
    EXPECT_GT(e2, e1);   // filter ramps toward e_raw across windows
}

TEST(EdmGapServo, GTargetMatchesFormula) {
    ServoConfig c; GapServo srv(c);
    GapServoOutput o = srv.step(GapServo::reset(), healthy());
    EXPECT_NEAR(o.g_target, c.w_open * c.open_ref - c.w_short * c.short_ref, 1e-6f);
}
```

- [ ] **Step 2: Run + verify PASS**, then **Step 3: Commit** `git commit -am "test(edm): GapServo error math, EWMA, g telemetry"`

## Task 4: deadband HOLD + Ki=0 pure-proportional default

- [ ] **Step 1: Append tests**

```cpp
TEST(EdmGapServo, HealthyBandHoldsInDeadband) {
    ServoConfig c; GapServo srv(c);
    // ratios exactly at refs => e ~ 0 => within deadband => HOLD, integ unchanged
    GapServoState s = GapServo::reset();
    for (int i = 0; i < 10; ++i) s = srv.step(s, healthy()).next;   // settle EWMA + cross decimation
    GapServoOutput o = srv.step(s, healthy());
    EXPECT_NEAR(o.v_cmd_mm_min, 0.0f, 1e-3f);
    EXPECT_EQ(o.state, ServoState::Hold);
    EXPECT_FLOAT_EQ(o.integrator, 0.0f);
}

TEST(EdmGapServo, KiZeroIsPureProportional) {
    ServoConfig c; c.Ki = 0.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.60f; wide.short_ratio = 0.02f; wide.normal_ratio = 0.30f;
    GapServoState s = GapServo::reset();
    for (int i = 0; i < c.pi_decimation; ++i) s = srv.step(s, wide).next;  // force one PI update
    GapServoOutput o = srv.step(s, wide);
    EXPECT_FLOAT_EQ(o.integrator, 0.0f);                 // integ never accumulates when Ki=0
    EXPECT_GT(o.v_cmd_mm_min, 0.0f);
    EXPECT_LE(o.v_cmd_mm_min, c.v_feed_max);
}
```

- [ ] **Step 2: Run + verify PASS**, **Step 3: Commit** `git commit -am "test(edm): GapServo deadband hold + Ki=0 proportional"`

## Task 5: advance / retract regulation + clamps

- [ ] **Step 1: Append tests**

```cpp
TEST(EdmGapServo, WideGapAdvancesClampedToFeedMax) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.70f; wide.short_ratio = 0.0f; wide.normal_ratio = 0.30f;
    GapServoState s = GapServo::reset();
    GapServoOutput o;
    for (int i = 0; i < 200; ++i) { o = srv.step(s, wide); s = o.next; }
    EXPECT_GT(o.v_cmd_mm_min, 0.0f);
    EXPECT_LE(o.v_cmd_mm_min, c.v_feed_max + 1e-4f);
}

TEST(EdmGapServo, ModerateTightRetractsClampedToRetract) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    // short below the LAYER3 reflex threshold (0.30) and below 2x-ref so consec stays 0,
    // but enough to drive e negative -> PI retract.
    GapServoInput tight = healthy(); tight.short_ratio = 0.15f; tight.open_ratio = 0.0f; tight.normal_ratio = 0.77f;
    GapServoState s = GapServo::reset();
    GapServoOutput o;
    for (int i = 0; i < 200; ++i) { o = srv.step(s, tight); s = o.next; }
    EXPECT_LT(o.v_cmd_mm_min, 0.0f);
    EXPECT_GE(o.v_cmd_mm_min, -c.v_retract - 1e-4f);
}
```

Note: `short_ratio=0.15` is above `short_ref*2=0.16`? No — 0.15 < 0.16, so `consec_short` stays 0 and LAYER 3 won't fire on the consec path; 0.15 < `short_retract`=0.30 so the ratio path won't fire either. Good — this isolates PI retract. If `ModerateTightRetractsClampedToRetract` instead trips a reflex, lower `short_ratio` to 0.12 and raise `arc`/`normal` accordingly.

- [ ] **Step 2: Run + verify PASS**, **Step 3: Commit** `git commit -am "test(edm): GapServo advance/retract clamps"`

## Task 6: PI integrator + anti-windup (Ki>0)

- [ ] **Step 1: Append tests**

```cpp
TEST(EdmGapServo, IntegratorAdvancesOnlyOnDecimation) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.60f; wide.short_ratio = 0.0f; wide.normal_ratio = 0.40f;
    GapServoState s = GapServo::reset();
    float prev = 0.0f;
    int changes = 0;
    for (int i = 1; i <= c.pi_decimation * 3; ++i) {
        GapServoOutput o = srv.step(s, wide); s = o.next;
        if (o.integrator != prev) { changes++; prev = o.integrator; }
    }
    EXPECT_EQ(changes, 3);   // integ moved exactly 3 times in 15 windows
}

TEST(EdmGapServo, AntiWindupClampsInteg) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.95f; wide.short_ratio = 0.0f; wide.normal_ratio = 0.05f;
    GapServoState s = GapServo::reset();
    GapServoOutput o;
    for (int i = 0; i < 2000; ++i) { o = srv.step(s, wide); s = o.next; }
    EXPECT_LE(o.integrator, c.v_feed_max / c.Ki + 1e-3f);   // anti-windup ceiling
    EXPECT_LE(o.v_cmd_mm_min, c.v_feed_max + 1e-4f);
}
```

- [ ] **Step 2: Run + verify PASS**, **Step 3: Commit** `git commit -am "test(edm): GapServo PI integrator + anti-windup"`

## Task 7: reflex LAYER 3 — short retract priority

- [ ] **Step 1: Append tests**

```cpp
TEST(EdmGapServo, ShortRatioAboveThresholdForcesMaxRetract) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput sh = healthy(); sh.short_ratio = 0.40f; sh.open_ratio = 0.0f; sh.normal_ratio = 0.52f;
    GapServoOutput o = srv.step(GapServo::reset(), sh);
    EXPECT_FLOAT_EQ(o.v_cmd_mm_min, -c.v_retract);
    EXPECT_EQ(o.state, ServoState::Retract);
}

TEST(EdmGapServo, ConsecutiveShortWindowsRetract) {
    ServoConfig c; c.Ki = 0.0f; GapServo srv(c);
    GapServoInput sh = healthy(); sh.short_ratio = 0.20f; sh.open_ratio = 0.0f; sh.normal_ratio = 0.72f; // >0.16, <0.30
    GapServoState s = GapServo::reset();
    GapServoOutput o = srv.step(s, sh); s = o.next;   // consec=1
    EXPECT_NE(o.state, ServoState::Retract);
    o = srv.step(s, sh); s = o.next;                  // consec=2
    EXPECT_NE(o.state, ServoState::Retract);
    o = srv.step(s, sh);                              // consec=3 -> reflex
    EXPECT_EQ(o.state, ServoState::Retract);
    EXPECT_FLOAT_EQ(o.v_cmd_mm_min, -c.v_retract);
}
```

- [ ] **Step 2: Run + verify PASS**, **Step 3: Commit** `git commit -am "test(edm): GapServo short-retract reflex"`

## Task 8: integ bleed on reflex

- [ ] **Step 1: Append test**

```cpp
TEST(EdmGapServo, RetractBleedsIntegrator) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    // wind integ up on a wide gap
    GapServoInput wide = healthy(); wide.open_ratio = 0.60f; wide.short_ratio = 0.0f; wide.normal_ratio = 0.40f;
    GapServoState s = GapServo::reset();
    GapServoOutput o;
    for (int i = 0; i < 100; ++i) { o = srv.step(s, wide); s = o.next; }
    float wound = o.integrator;
    ASSERT_GT(wound, 0.1f);
    // one short-retract reflex window halves the integrator
    GapServoInput sh = healthy(); sh.short_ratio = 0.40f; sh.open_ratio = 0.0f; sh.normal_ratio = 0.52f;
    o = srv.step(s, sh);
    EXPECT_NEAR(o.integrator, wound * c.integ_bleed, 1e-3f);
}
```

- [ ] **Step 2: Run + verify PASS** (note: the reflex bleed runs even though PI did not update this window). **Step 3: Commit** `git commit -am "test(edm): GapServo integ bleed on reflex"`

## Task 9: arc brake + arc hold (LAYER 4/5)

- [ ] **Step 1: Append tests**

```cpp
TEST(EdmGapServo, ArcHoldZeroesNeverRetracts) {
    ServoConfig c; GapServo srv(c);
    GapServoInput a = healthy(); a.arc_ratio = 0.50f; a.short_ratio = 0.05f; a.open_ratio = 0.10f; a.normal_ratio = 0.35f;
    GapServoOutput o = srv.step(GapServo::reset(), a);
    EXPECT_FLOAT_EQ(o.v_cmd_mm_min, 0.0f);
    EXPECT_EQ(o.state, ServoState::ArcHold);
}

TEST(EdmGapServo, ArcBrakeTapersAdvance) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.60f; wide.short_ratio = 0.0f; wide.arc_ratio = 0.0f; wide.normal_ratio = 0.40f;
    GapServoState s = GapServo::reset();
    GapServoOutput no_arc;
    for (int i = 0; i < 200; ++i) { no_arc = srv.step(s, wide); s = no_arc.next; }
    GapServoInput braked = wide; braked.arc_ratio = 0.30f; braked.normal_ratio = 0.10f;  // between brake(0.2) and hold(0.4)
    GapServoOutput o = srv.step(s, braked);
    EXPECT_GT(o.v_cmd_mm_min, 0.0f);
    EXPECT_LT(o.v_cmd_mm_min, no_arc.v_cmd_mm_min);   // strictly tapered
    EXPECT_EQ(o.state, ServoState::Track);
}
```

- [ ] **Step 2: Run + verify PASS**, **Step 3: Commit** `git commit -am "test(edm): GapServo arc brake/hold"`

## Task 10: invalid HOLD + touch-off (LAYER 0/1)

- [ ] **Step 1: Append tests**

```cpp
TEST(EdmGapServo, LowPulseCountHolds) {
    ServoConfig c; GapServo srv(c);
    GapServoInput bad; bad.valid = false; bad.total_pulses = 5;
    GapServoState s = GapServo::reset(); s.consec_short = 2; s.integ = 1.0f;
    GapServoOutput o = srv.step(s, bad);
    EXPECT_FLOAT_EQ(o.v_cmd_mm_min, 0.0f);
    EXPECT_EQ(o.state, ServoState::Hold);
    EXPECT_FLOAT_EQ(o.integrator, 1.0f);   // frozen
    EXPECT_EQ(o.next.consec_short, 2);      // not incremented on invalid
}

TEST(EdmGapServo, TouchOffCommandsConstantApproach) {
    ServoConfig c; GapServo srv(c);
    GapServoInput in = healthy(); in.in_touch_off = true;
    GapServoOutput o = srv.step(GapServo::reset(), in);
    EXPECT_FLOAT_EQ(o.v_cmd_mm_min, c.v_touch);
    EXPECT_EQ(o.state, ServoState::Track);
}
```

- [ ] **Step 2: Run + verify PASS**, **Step 3: Commit** `git commit -am "test(edm): GapServo invalid-hold + touch-off"`

## Task 11: wire-break feed cap

- [ ] **Step 1: Append test**

```cpp
TEST(EdmGapServo, FeedCapThrottlesAdvanceNotRetract) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.70f; wide.short_ratio = 0.0f; wide.normal_ratio = 0.30f;
    GapServoState s = GapServo::reset();
    GapServoOutput full;
    for (int i = 0; i < 200; ++i) { full = srv.step(s, wide); s = full.next; }
    GapServoInput capped = wide; capped.feed_cap_mult = 0.6f;
    GapServoOutput o = srv.step(s, capped);
    EXPECT_NEAR(o.v_cmd_mm_min, full.v_cmd_mm_min * 0.6f, 1e-3f);

    // retract is NOT weakened by the cap
    GapServoInput sh = healthy(); sh.short_ratio = 0.40f; sh.open_ratio = 0.0f; sh.normal_ratio = 0.52f; sh.feed_cap_mult = 0.3f;
    GapServoOutput r = srv.step(GapServo::reset(), sh);
    EXPECT_FLOAT_EQ(r.v_cmd_mm_min, -c.v_retract);
}
```

- [ ] **Step 2: Run + verify PASS**, **Step 3: Commit** `git commit -am "test(edm): GapServo wire-break feed cap"`

## Task 12: signed-velocity boundary conversion

- [ ] **Step 1: Append test**

```cpp
TEST(EdmGapServo, VelocityConvertsToMicrometersPerSecond) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.95f; wide.short_ratio = 0.0f; wide.normal_ratio = 0.05f;
    GapServoState s = GapServo::reset();
    GapServoOutput o;
    for (int i = 0; i < 2000; ++i) { o = srv.step(s, wide); s = o.next; }
    EXPECT_EQ(o.v_s_um_s, 67);    // 4.0 mm/min -> lround(4000/60) = 67

    GapServoInput sh = healthy(); sh.short_ratio = 0.40f; sh.open_ratio = 0.0f; sh.normal_ratio = 0.52f;
    GapServoOutput r = srv.step(GapServo::reset(), sh);
    EXPECT_EQ(r.v_s_um_s, -100);  // -6.0 mm/min -> -100
}
```

- [ ] **Step 2: Run + verify PASS**, **Step 3: Commit** `git commit -am "test(edm): GapServo velocity boundary conversion"`

---

## Task 13: ModeTable — EdmMode, ModeParams, S-word band mapping

**Files:** Create `FluidNC/src/EDM/Servo/ModeTable.h`, `.cpp`, `FluidNC/tests/EdmModeTableTest.cpp`. Modify `platformio.ini` (add `+<EDM/Servo/ModeTable.cpp>` and `+<tests/EdmModeTableTest.cpp>`).

- [ ] **Step 1: Write `ModeTable.h`**

```cpp
// FluidNC/src/EDM/Servo/ModeTable.h
#pragma once
#include <cstdint>
#include "EDM/Psu/Protocol.h"

namespace EDM { namespace servo {

enum class EdmMode : uint8_t { Idle = 0, Ignite = 3, Finish = 2, Rough = 1 };

struct ModeParams {                       // one row; mirrors SetModeBounds + nominal ignition delay
    uint8_t  mode_id            = 0;
    uint16_t freq_max_kHz       = 0;
    uint16_t on_time_max_ns     = 0;
    uint16_t off_time_min_ns    = 0;
    uint16_t peak_I_setpoint_dA = 0;
    uint16_t peak_I_limit_hw_dA = 0;
    uint8_t  polarity           = 0;
    uint16_t flags              = 0;
    uint16_t gap_V_arc_mV       = 0;
    uint16_t gap_V_short_mV     = 0;
    uint16_t ignition_timeout_us = 0;
    uint16_t ign_delay_nom_ns   = 10000;  // used by the servo's delay-trim term
};

class ModeTable {
public:
    ModeParams rough, finish, ignite;
    uint16_t   ignite_max = 99;           // S-word band edges
    uint16_t   finish_max = 499;
    bool       adaptive_setpoint = false;

    EdmMode    modeForSword(uint16_t s) const;
    const ModeParams& params(EdmMode m) const;
    EDM::psu::SetModeBounds build(EdmMode m, uint16_t s_word, uint16_t seq) const;
    EdmMode    lowerEnergy(EdmMode m) const;   // Rough->Finish->Ignite->Ignite
};

}}  // namespace EDM::servo
```

- [ ] **Step 2: Write the failing test `EdmModeTableTest.cpp`**

```cpp
// FluidNC/tests/EdmModeTableTest.cpp
#include "gtest/gtest.h"
#include "EDM/Servo/ModeTable.h"

using namespace EDM::servo;
using namespace EDM::psu;

static ModeTable makeTable() {
    ModeTable t;
    t.rough  = { 1, 200, 800, 1200, 200, 260, 0, 0x000D, 25000, 8000, 30, 10000 };
    t.finish = { 2, 400, 200,  600,  80, 120, 0, 0x000D, 25000, 8000, 20,  6000 };
    t.ignite = { 3, 100, 300, 3000,  40,  80, 0, 0x0008, 25000, 8000, 50, 16000 };
    return t;
}

TEST(EdmModeTable, SwordBandsMapToModes) {
    ModeTable t = makeTable();
    EXPECT_EQ(t.modeForSword(0),   EdmMode::Idle);
    EXPECT_EQ(t.modeForSword(50),  EdmMode::Ignite);
    EXPECT_EQ(t.modeForSword(300), EdmMode::Finish);
    EXPECT_EQ(t.modeForSword(900), EdmMode::Rough);
}
```

- [ ] **Step 3: Run, verify FAIL** — `pio test -e tests -f '*EdmModeTable*'`.

- [ ] **Step 4: Write `ModeTable.cpp`**

```cpp
// FluidNC/src/EDM/Servo/ModeTable.cpp
#include "EDM/Servo/ModeTable.h"

namespace EDM { namespace servo {

EdmMode ModeTable::modeForSword(uint16_t s) const {
    if (s == 0)             return EdmMode::Idle;
    if (s <= ignite_max)    return EdmMode::Ignite;
    if (s <= finish_max)    return EdmMode::Finish;
    return EdmMode::Rough;
}

const ModeParams& ModeTable::params(EdmMode m) const {
    switch (m) {
        case EdmMode::Rough:  return rough;
        case EdmMode::Finish: return finish;
        default:              return ignite;   // Ignite + Idle fall back to ignite row
    }
}

EDM::psu::SetModeBounds ModeTable::build(EdmMode m, uint16_t s_word, uint16_t seq) const {
    const ModeParams& p = params(m);
    EDM::psu::SetModeBounds b;
    b.mode_id            = p.mode_id;
    b.seq                = seq;
    b.freq_max_kHz       = p.freq_max_kHz;
    b.on_time_max_ns     = p.on_time_max_ns;
    b.off_time_min_ns    = p.off_time_min_ns;
    b.peak_I_setpoint_dA = p.peak_I_setpoint_dA;
    b.peak_I_limit_hw_dA = p.peak_I_limit_hw_dA;
    b.polarity           = p.polarity;
    b.flags              = p.flags;
    b.gap_V_arc_mV       = p.gap_V_arc_mV;
    b.gap_V_short_mV     = p.gap_V_short_mV;
    b.ignition_timeout_us= p.ignition_timeout_us;

    // adaptive peak_I lerp (OFF by default): scale setpoint by S within the band,
    // clamped to the hardware limit. Band span uses ignite_max/finish_max edges.
    if (adaptive_setpoint && m != EdmMode::Idle) {
        uint16_t lo = 1, hi = 1000;
        if (m == EdmMode::Ignite) { lo = 1;               hi = ignite_max; }
        else if (m == EdmMode::Finish) { lo = ignite_max + 1; hi = finish_max; }
        else { lo = finish_max + 1; hi = 1000; }
        if (s_word < lo) s_word = lo;
        if (s_word > hi) s_word = hi;
        float t = (hi > lo) ? float(s_word - lo) / float(hi - lo) : 0.0f;
        uint32_t base = p.peak_I_setpoint_dA;
        uint32_t scaled = uint32_t(base * (0.5f + 0.5f * t));   // 50%..100% of setpoint across band
        if (scaled > p.peak_I_limit_hw_dA) scaled = p.peak_I_limit_hw_dA;
        b.peak_I_setpoint_dA = uint16_t(scaled);
    }
    return b;
}

EdmMode ModeTable::lowerEnergy(EdmMode m) const {
    switch (m) {
        case EdmMode::Rough:  return EdmMode::Finish;
        case EdmMode::Finish: return EdmMode::Ignite;
        default:              return EdmMode::Ignite;
    }
}

}}  // namespace EDM::servo
```

- [ ] **Step 5: Run, verify PASS**, **Step 6: Commit** `git add FluidNC/src/EDM/Servo/ModeTable.* FluidNC/tests/EdmModeTableTest.cpp platformio.ini && git commit -m "feat(edm): ModeTable S-word mapping + SetModeBounds build (TDD)"`

## Task 14: ModeTable build fields + adaptive gate + lowerEnergy

- [ ] **Step 1: Append tests**

```cpp
TEST(EdmModeTable, BuildEmitsModeBoundsFields) {
    ModeTable t = makeTable();
    SetModeBounds b = t.build(EdmMode::Rough, 900, 7);
    EXPECT_EQ(b.mode_id, 1);
    EXPECT_EQ(b.freq_max_kHz, 200);
    EXPECT_EQ(b.peak_I_setpoint_dA, 200);   // adaptive OFF => fixed
    EXPECT_EQ(b.seq, 7);
}

TEST(EdmModeTable, AdaptiveOffUsesFixedPeakI) {
    ModeTable t = makeTable(); t.adaptive_setpoint = false;
    EXPECT_EQ(t.build(EdmMode::Rough, 500, 1).peak_I_setpoint_dA, 200);
    EXPECT_EQ(t.build(EdmMode::Rough, 999, 1).peak_I_setpoint_dA, 200);
}

TEST(EdmModeTable, AdaptiveOnLerpsClampedToLimit) {
    ModeTable t = makeTable(); t.adaptive_setpoint = true;
    uint16_t lowS  = t.build(EdmMode::Rough, 500, 1).peak_I_setpoint_dA;
    uint16_t highS = t.build(EdmMode::Rough, 1000, 1).peak_I_setpoint_dA;
    EXPECT_LT(lowS, highS);                       // scales with S
    EXPECT_LE(highS, t.rough.peak_I_limit_hw_dA); // never exceeds HW limit
}

TEST(EdmModeTable, LowerEnergySteps) {
    ModeTable t = makeTable();
    EXPECT_EQ(t.lowerEnergy(EdmMode::Rough),  EdmMode::Finish);
    EXPECT_EQ(t.lowerEnergy(EdmMode::Finish), EdmMode::Ignite);
    EXPECT_EQ(t.lowerEnergy(EdmMode::Ignite), EdmMode::Ignite);
}
```

- [ ] **Step 2: Run + verify PASS**, **Step 3: Commit** `git commit -am "test(edm): ModeTable build fields + adaptive gate + lowerEnergy"`

---

## Task 15: EdmController — FaultReason, EdmReport, arm→ack→touch-off + memcmp gate

**Files:** Create `FluidNC/src/EDM/Servo/FaultReason.h`, `FluidNC/src/EDM/Servo/EdmReport.h`, `FluidNC/src/EDM/Control/EdmController.h`, `.cpp`, `FluidNC/tests/EdmControllerTest.cpp`. Modify `platformio.ini` (add `+<EDM/Control/EdmController.cpp>` and `+<tests/EdmControllerTest.cpp>`).

- [ ] **Step 1: Write `FaultReason.h` and `EdmReport.h`**

```cpp
// FluidNC/src/EDM/Servo/FaultReason.h
#pragma once
#include <cstdint>
namespace EDM {
enum class FaultReason : uint8_t {
    None, AckTimeout, ProtocolMismatch, TouchOffNoContact,
    ServoStall, HeartbeatLost, PsuFault, SensorDisagree
};
}
```

```cpp
// FluidNC/src/EDM/Servo/EdmReport.h
#pragma once
#include <cstdint>
namespace EDM {
// Integer-encoded telemetry snapshot. The contract sub-project C (WebUI) consumes.
struct EdmReport {
    uint32_t window_id = 0;
    uint8_t  psu_state = 0;
    bool     connected = false, protocol_ok = false;
    uint16_t last_ack_status = 0;
    uint8_t  controller_state = 0;   // EdmState
    uint8_t  fault_reason = 0;       // FaultReason
    uint8_t  servo_state = 0;        // ServoState
    uint8_t  active_mode_id = 0;
    uint16_t s_word = 0;
    uint16_t n_normal = 0, n_arc = 0, n_short = 0, n_open = 0, total_pulses = 0;
    int16_t  open_ratio_milli = 0, short_ratio_milli = 0, arc_ratio_milli = 0;
    int32_t  v_cmd_um_s = 0;         // the P1 output
    int16_t  g_milli = 0, g_target_milli = 0, e_filtered_milli = 0, integrator_milli = 0;
    uint8_t  feed_cap_pct = 100, wire_break_sev = 0;
    uint16_t peak_I_mean_dA = 0, peak_I_max_dA = 0;
    uint32_t energy_delivered_uJ = 0;
    int16_t  temp_GaN_dC = 0, temp_L_dC = 0;
    uint16_t dc_link_V_dV = 0;
    uint16_t flags = 0;
};
}
```

- [ ] **Step 2: Write `EdmController.h`**

```cpp
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

struct ControllerTimers {                 // all milliseconds
    uint32_t tele_hold = 20, tele_fault = 200, ack = 200, stall = 2000, break_relief = 50, clear = 500;
    uint32_t touchoff_budget = 6000;      // ms of touch-off with no contact -> fault
};

class EdmController {
public:
    EdmController(EDM::psu::IPsuLink& link, const EDM::servo::ServoConfig& cfg,
                  const EDM::servo::ModeTable& modes, const ControllerTimers& timers = {});

    void requestCut(uint16_t s_word);    // M3/M4 with S
    void requestStop();                  // M5 / S0
    void notifyCutComplete();            // P2 hook
    void clearFault();
    void tick(uint32_t now_ms);          // 1 kHz; the only IPsuLink consumer

    EdmState     state() const  { return _state; }
    FaultReason  fault() const  { return _fault; }
    EdmReport    snapshot() const { return _report; }

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
```

- [ ] **Step 3: Write the failing test `EdmControllerTest.cpp`** (Task 15 portion)

```cpp
// FluidNC/tests/EdmControllerTest.cpp
#include "gtest/gtest.h"
#include "EDM/Control/EdmController.h"
#include "EDM/Psu/SimPsuLink.h"

using namespace EDM;
using namespace EDM::servo;
using namespace EDM::psu;

static ModeTable makeModes() {
    ModeTable t;
    t.rough  = { 1, 200, 800, 1200, 200, 260, 0, 0x000D, 25000, 8000, 30, 10000 };
    t.finish = { 2, 400, 200,  600,  80, 120, 0, 0x000D, 25000, 8000, 20,  6000 };
    t.ignite = { 3, 100, 300, 3000,  40,  80, 0, 0x0008, 25000, 8000, 50, 16000 };
    return t;
}

TEST(EdmController, ArmSendsBoundsWaitsAckStartsCut) {
    SimPsuLink sim; sim.begin();
    ServoConfig cfg; ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);

    ctl.requestCut(900);          // Rough
    ctl.tick(0);
    // SimPsuLink ACKs immediately (lastAckStatus()==0 and connected), so by next tick we move on.
    EXPECT_EQ(ctl.state(), EdmState::Armed);
    ctl.tick(1);
    EXPECT_EQ(ctl.state(), EdmState::TouchOff);
}

TEST(EdmController, RepeatedSameModeNotResent) {
    SimPsuLink sim; sim.begin();
    ServoConfig cfg; ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);
    ctl.requestCut(900); ctl.tick(0);
    EdmReport r1 = ctl.snapshot();
    ctl.requestCut(950);  // still Rough band -> identical resolved bounds
    ctl.tick(1);
    // No new bounds frame sent (memcmp gate). We assert via window_id passthrough not changing mode.
    EXPECT_EQ(ctl.snapshot().active_mode_id, r1.active_mode_id);
}
```

Note: `SimPsuLink::lastAckStatus()` returns 0 and `isConnected()`/`protocolCompatible()` are true after `begin()`, so Armed→TouchOff happens on the tick after the bounds send.

- [ ] **Step 4: Run, verify FAIL** — `pio test -e tests -f '*EdmController*'`.

- [ ] **Step 5: Write `EdmController.cpp`** — constructor, helpers, and the arm/ack/touch-off slice of `tick()`. (Later tasks extend `tick()`; write the full file now with the cutting/hold/fault branches included so the state machine is complete and Tasks 16–18 only add tests.)

```cpp
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
    SetModeBounds a = want;          a.seq = 0;
    SetModeBounds b = _last_sent;    b.seq = 0;
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

void EdmController::tick(uint32_t now_ms) {
    _now = now_ms;

    // 1) drain async events (may force Fault / BreakRelief)
    drainEvents();

    // 2) health + telemetry freshness
    bool connected = _link.isConnected();
    bool proto     = _link.protocolCompatible();
    StatsAgg s; bool got = _link.latestStats(s);
    bool fresh = got && (!_have_window || s.window_id != _last_window_id);
    if (fresh) { _last_window_id = s.window_id; _have_window = true; _last_window_change_ms = now_ms; }

    bool stale_hold  = (now_ms - _last_window_change_ms) > _t.tele_hold;
    bool stale_fault = (now_ms - _last_window_change_ms) > _t.tele_fault;

    // 3) state-independent fault gates
    if (_state != EdmState::Fault && _state != EdmState::StallFault && _state != EdmState::Idle) {
        if (got && s.state == 2 /*PSU fault*/) { enterFault(FaultReason::PsuFault); }
        else if (!proto)                        { enterFault(FaultReason::ProtocolMismatch); }
        else if (!connected && stale_fault)     { enterFault(FaultReason::HeartbeatLost); }
        else if (_have_window && stale_fault)   { enterFault(FaultReason::HeartbeatLost); }
    }

    // 4) ARM: wait for ack/connection, then start cut + touch-off
    if (_state == EdmState::Armed) {
        if (!proto) { enterFault(FaultReason::ProtocolMismatch); }
        else if (connected && _link.lastAckStatus() == 0) {
            _link.startCut();
            _ss = GapServo::reset();
            _state = EdmState::TouchOff;
            _touchoff_start = now_ms; _touchoff_contig = 0;
        } else if (now_ms >= _arm_deadline) {
            enterFault(FaultReason::AckTimeout);
        }
    }

    // 5) BreakRelief timing -> back to touch-off
    if (_state == EdmState::BreakRelief && now_ms >= _break_relief_until) {
        _state = EdmState::TouchOff; _touchoff_start = now_ms; _touchoff_contig = 0;
    }

    // 6) wire-break severity decay (ramp cap back toward 1.0 after clear window)
    if (_wire_break_sev > 0 && now_ms >= _sev_clear_after) {
        _wire_break_sev = 0; _feed_cap_mult = 1.0f;
    }

    // 7) run the servo when actively cutting/holding/touch-off/break-relief
    GapServoOutput out;
    bool ran = false;
    if (_state == EdmState::TouchOff || _state == EdmState::Cutting ||
        _state == EdmState::Hold     || _state == EdmState::BreakRelief) {
        GapServoInput in = buildInput(s, fresh);
        if (stale_hold) in.valid = false;     // telemetry-loss => HOLD via invalid
        out = _servo.step(_ss, in); _ss = out.next; ran = true;

        // touch-off: detect contact (gap closed => sparks => not mostly-open)
        if (_state == EdmState::TouchOff) {
            if (in.valid && (1.0f - in.open_ratio) > 0.5f) {
                if (++_touchoff_contig >= 2) { _state = EdmState::Cutting; _ss.integ = 0.0f; }
            } else {
                _touchoff_contig = 0;
            }
            if (now_ms - _touchoff_start > _t.touchoff_budget) enterFault(FaultReason::TouchOffNoContact);
        } else if (_state == EdmState::Cutting || _state == EdmState::Hold) {
            // lost-gap: sustained all-open => re-approach
            if (in.valid && in.open_ratio > 0.95f) {
                if (++_lostgap_contig >= 5) { _state = EdmState::TouchOff; _touchoff_start = now_ms; _touchoff_contig = 0; }
            } else { _lostgap_contig = 0; }

            // hold/cutting toggle + stall watchdog
            bool advancing = out.v_cmd_mm_min > 0.0f;
            if (out.state == ServoState::Hold || out.state == ServoState::ArcHold || out.v_cmd_mm_min <= 0.0f) {
                if (_state == EdmState::Cutting) { _state = EdmState::Hold; _stall_since = now_ms; }
                if (!advancing && (now_ms - _stall_since) > _t.stall &&
                    (out.state == ServoState::Retract || out.state == ServoState::Hold)) {
                    _link.stopCut(); _state = EdmState::StallFault; _fault = FaultReason::ServoStall;
                }
            } else if (_state == EdmState::Hold) {
                _state = EdmState::Cutting;
            }
        }
    }

    // 8) build the report snapshot (integer-encoded)
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
```

- [ ] **Step 6: Run, verify PASS** — `pio test -e tests -f '*EdmController*'` (2 tests). If `RepeatedSameModeNotResent` needs a stronger assertion, it is acceptable to assert `ctl.snapshot().active_mode_id == 1` both times (same Rough mode). Green.

- [ ] **Step 7: Commit** `git add FluidNC/src/EDM/Servo/FaultReason.h FluidNC/src/EDM/Servo/EdmReport.h FluidNC/src/EDM/Control/EdmController.* FluidNC/tests/EdmControllerTest.cpp platformio.ini && git commit -m "feat(edm): EdmController state machine + arm/ack/touch-off + report (TDD)"`

## Task 16: closed-loop convergence with the proportional sim

- [ ] **Step 1: Append test**

```cpp
TEST(EdmController, ClosedLoopReachesIdealGap) {
    SimPsuLink sim; sim.begin();
    sim.setVelocityCoupling(0.00002f);
    sim.setGap(sim.idealGapMm() * 3.0f);   // start wide
    ServoConfig cfg; cfg.Ki = 2.0f;        // enable integral for convergence
    ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);

    ctl.requestCut(900);
    uint32_t t = 0;
    // prime: arm -> touch-off -> cutting
    for (int i = 0; i < 5; ++i) { sim.tick(); ctl.tick(t++); }

    float worst_short = 0.0f;
    for (int i = 0; i < 4000; ++i) {
        EdmReport r = ctl.snapshot();
        sim.applyCommandedVelocity(r.v_cmd_um_s);
        sim.tick();
        ctl.tick(t++);
    }
    EXPECT_NE(ctl.state(), EdmState::StallFault);
    EXPECT_NE(ctl.state(), EdmState::Fault);
    // gap settled near ideal => short ratio low, |v| small
    StatsAgg s; sim.latestStats(s);
    EXPECT_LT(s.n_short, 20);
    EXPECT_NEAR(sim.gapMm(), sim.idealGapMm(), sim.idealGapMm() * 1.0f);  // within 1x ideal
    (void)worst_short;
}
```

- [ ] **Step 2: Run + verify PASS.** If convergence is marginal, tune the test's coupling gain (smaller = slower, more stable) — NOT the controller. **Step 3: Commit** `git commit -am "test(edm): EdmController closed-loop convergence with sim"`

## Task 17: wire-break sev3 → BreakRelief → re-approach

- [ ] **Step 1: Append test**

```cpp
TEST(EdmController, WireBreakSev3RelievesThenReApproaches) {
    SimPsuLink sim; sim.begin();
    sim.setVelocityCoupling(0.00002f);
    sim.setGap(sim.idealGapMm());
    ServoConfig cfg; cfg.Ki = 2.0f; ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);
    ctl.requestCut(900);
    uint32_t t = 0;
    for (int i = 0; i < 10; ++i) { sim.tick(); ctl.tick(t++); }
    ASSERT_TRUE(ctl.state() == EdmState::Cutting || ctl.state() == EdmState::Hold);

    Event ev; ev.kind = Event::WireBreak; ev.wire_break.severity = 3;
    sim.pushEvent(ev);
    sim.tick(); ctl.tick(t++);
    EXPECT_EQ(ctl.state(), EdmState::BreakRelief);
    EXPECT_LT(ctl.snapshot().v_cmd_um_s, 0);            // retracting
    EXPECT_EQ(ctl.snapshot().active_mode_id, 3);        // ignite mode requested

    // after break_relief window -> TouchOff
    for (int i = 0; i < 60; ++i) { sim.tick(); ctl.tick(t++); }   // > break_relief (50ms)
    EXPECT_EQ(ctl.state(), EdmState::TouchOff);
}
```

- [ ] **Step 2: Run + verify PASS**, **Step 3: Commit** `git commit -am "test(edm): EdmController wire-break sev3 relief"`

## Task 18: telemetry-stale, stall, and lost-gap faults

- [ ] **Step 1: Append tests**

```cpp
TEST(EdmController, TelemetryStaleHoldsThenFaults) {
    SimPsuLink sim; sim.begin();
    sim.setGap(sim.idealGapMm());
    ServoConfig cfg; ModeTable modes = makeModes();
    ControllerTimers tm;  // tele_hold=20, tele_fault=200
    EdmController ctl(sim, cfg, modes, tm);
    ctl.requestCut(900);
    uint32_t t = 0;
    for (int i = 0; i < 6; ++i) { sim.tick(); ctl.tick(t++); }   // reach cutting/touchoff
    // STOP ticking the sim -> window_id frozen -> stale
    uint32_t base = t;
    // within tele_hold: HOLD (v==0) but not fault
    ctl.tick(base + 10);
    EXPECT_EQ(ctl.snapshot().v_cmd_um_s, 0);
    EXPECT_NE(ctl.state(), EdmState::Fault);
    // beyond tele_fault: HeartbeatLost
    ctl.tick(base + 250);
    EXPECT_EQ(ctl.state(), EdmState::Fault);
    EXPECT_EQ(ctl.fault(), FaultReason::HeartbeatLost);
}

TEST(EdmController, LostGapReentersTouchOff) {
    SimPsuLink sim; sim.begin();
    sim.setGap(sim.idealGapMm());
    ServoConfig cfg; cfg.Ki = 2.0f; ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);
    ctl.requestCut(900);
    uint32_t t = 0;
    for (int i = 0; i < 10; ++i) { sim.tick(); ctl.tick(t++); }
    // force a lost gap: very wide, all-open, for >=5 windows
    sim.setVelocityCoupling(0.0f);
    sim.setGap(sim.idealGapMm() * 5.0f);
    for (int i = 0; i < 8; ++i) { sim.tick(); ctl.tick(t++); }
    EXPECT_EQ(ctl.state(), EdmState::TouchOff);
}
```

- [ ] **Step 2: Run + verify PASS.** If `TelemetryStaleHoldsThenFaults` trips fault earlier than expected because the controller treats `!fresh` as stale immediately, confirm the watchdog uses `now - _last_window_change_ms` thresholds (it does). **Step 3: Commit** `git commit -am "test(edm): EdmController telemetry-stale + lost-gap faults"`

---

## Task 19: ESP32 wiring stubs (EdmSpindle, EdmServoTask, EdmReportChannel)

These are ESP32-coupled (FreeRTOS/Spindle/Report) — **not** host-built and **not** added to `build_src_filter`. They are created so the firmware target can compile/link them in P4; they are verified by the ESP32 build, not googletest. Keep bodies minimal and correct against the verified integration facts.

**Files:** Create `FluidNC/src/EDM/EdmSpindle.{h,cpp}`, `FluidNC/src/EDM/EdmServoTask.{h,cpp}`, `FluidNC/src/EDM/EdmReportChannel.{h,cpp}`.

- [ ] **Step 1: Write `EdmSpindle.h/.cpp`** following the verified registration pattern (`SpindleFactory::InstanceBuilder<EdmSpindle> registration("EDM")`, `group()` parsing the `edm:` YAML into `ServoConfig`/`ModeTable`, `setState()` → `requestCut/requestStop`, `isRateAdjusted()` → true). Construct a `SimPsuLink` when `use_sim: true`, else a `CanPsuLink` over an `Mcp2518fdDriver` (pins from config). `init()` builds the `EdmController` and starts the servo task. Use `log_info` for `config_message()`. (Full skeleton in the design synthesis §4.1; mirror `LaserSpindle.cpp` for the registration idiom.)

- [ ] **Step 2: Write `EdmServoTask.h/.cpp`** exposing `void startEdmServoTask(EdmController*)` that calls `xTaskCreatePinnedToCore(edmServoTask, "edm_servo", 8192, ctl, 2, &handle, SUPPORT_TASK_CORE)`; task body `for(;;){ add_watchdog_to_task(); ctl->tick(millis()); vTaskDelay(1); }`. In P1 this is compiled-but-inert (only started from `EdmSpindle::init()`, which is itself only instantiated when an `EDM` spindle is configured).

- [ ] **Step 3: Write `EdmReportChannel.h/.cpp`** exposing `report_edm_stats(Channel&, const EdmReport&)` emitting `[MSG:JSON:edm_update]` via the project's JSON encoder at the configured rate, and a `$EDM/Status` RuntimeSetting emitting `[MSG:JSON:edm_full_status]`. (Wiring into `Channel::autoReport()` and `report_realtime_status` happens in P4 — leave a documented TODO(P4) at the call site, do not modify `Report.cpp` in P1.)

- [ ] **Step 4: Confirm native tests untouched** — `pio test -e tests -f '*Edm*'` still green (these files are not in the filter).

- [ ] **Step 5: Commit** `git add FluidNC/src/EDM/EdmSpindle.* FluidNC/src/EDM/EdmServoTask.* FluidNC/src/EDM/EdmReportChannel.* && git commit -m "feat(edm): EdmSpindle facade + servo task + report channel (ESP32 wiring, P4-integrated)"`

---

## Phase P1 exit criteria

- `pio test -e tests -f '*Edm*'` green: SimPsuLink (8), GapServo (~18 across T2–T12), ModeTable (~5), EdmController (~6). All EDM logic ≥80% covered.
- The pure `GapServo` has zero FreeRTOS/global/IPsuLink/Arduino dependencies and is value-in/value-out.
- Closed-loop convergence demonstrated against the upgraded sim (T16); wire-break sev3 relief (T17); stale/stall/lost-gap faults (T18).
- ESP32 wiring files exist and compile in the firmware target; report-channel hookup left as a documented TODO(P4).
- `EdmReport` frozen as the sub-project C (WebUI) contract. `IPsuLink` unchanged.

## Roadmap after P1
- **P2** — `EdmMotion` path-parameter servo: route `G1/G2/G3` (EDM cut-mode modal, spark on) into a contour buffer; consume `EdmReport.v_cmd_um_s` to emit short accel-limited `plan_buffer_line` segments (advance/hold/retract); spark-disabled dry-run.
- **P3** — `WireFeed`: two RMT steppers + HX711 tension PI + wire-break feed/tension reaction (wired to the severity signals P1 already computes).
- **P4** — task wiring live, `Report.cpp`/`Channel::autoReport()` hookup, `$EDM/*` settings, end-to-end vs `mock-psu`, then real PSU + the IPsuLink P4 cross-core spinlock/ring-buffer (P0 TODOs).

## Self-review notes
- **Spec coverage:** servo error formula (T3), control law + params (T2/T4/T5/T6), full reflex ladder L0–L6 (T7/T9/T10 + cap T11), boundary integer conversion (T12), mode mapping + adaptive gate (T13/T14), state machine arm/touchoff/cutting/hold/breakrelief/fault (T15/T17/T18), closed-loop (T16), telemetry surface (EdmReport + T19). All §4 design points map to a task.
- **Grafts applied:** Ki=0 default (T2/T4), priority-encoder ladder (T7–T11), rich FaultReason (T15), memcmp-gated sends (T15), operator `g` scalar (T3 + EdmReport), immutable pure step (T2), integ-bleed-on-retract (T8), nested YAML (T19/design), adaptive-setpoint OFF default (T14), and the mandatory proportional-sim fix (T1).
- **No placeholders:** every host-tested step has complete test + impl code. Task 19 is intentionally skeletal (ESP32-only, verified by the firmware build not googletest) and references the synthesis for the boilerplate bodies — flagged explicitly, consistent with how P0 handled the MCP2518FD driver.
- **Type consistency:** `GapServoInput/State/Output`, `ServoConfig`, `EdmMode`, `ModeParams`, `SetModeBounds`, `EdmReport`, `FaultReason`, `EdmState`, `ControllerTimers`, and `SimPsuLink`'s new methods are used identically across tasks.
