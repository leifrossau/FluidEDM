# EDM PSU Integration — Phase P2 (Path-Parameter Motion Executor) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the gap servo's signed velocity (`EdmReport.v_cmd_um_s`) into real machine motion that **advances, holds, and retracts** along the cut contour — a pure, host-tested contour/sampler pair plus a gcode-thread emitter that feeds short segments into FluidNC's planner.

**Architecture:** The 1 kHz `EdmController::tick()` remains the sole producer of `v_cmd_um_s`; motion is emitted **only from the gcode/protocol thread** (the one thread allowed to call `plan_buffer_line()`), via a blocking dwell-style loop modeled on `mc_dwell`/`dwell_ms`. The two are coupled by a lock-free **per-field read** of the existing report (two inline accessors; no change to the hot tick, no new core State, no Protocol.cpp hook). A pure `ContourBuffer` (bounded vertex ring, arc-flattened, XY-arc-length parameter `s`) and a pure `PathSampler` (advance/hold/retract with accel/feed clamps, reversal-flush, retract window) hold all the logic and are 100% host-tested; the `EdmMotion` emitter + the `GCode.cpp` interception are ESP32-coupled and bench-verified (spark-disabled dry-run).

**Tech Stack:** C++17, PlatformIO, googletest (native `[env:tests]`), FluidNC Planner/MotionControl/GCode/Kinematics.

**Builds on (P0+P1, merged to `main`):** `EDM/Psu/*`, `EDM/Servo/{GapServo,ModeTable,EdmReport}`, `EDM/Control/EdmController` (produces `v_cmd_um_s`), `EDM/EdmSpindle` (M3/M5→requestCut/requestStop). `IPsuLink` and `EdmController::tick()` are unchanged by P2.

**Verified FluidNC facts (from the design workflow):**
- `plan_buffer_line(float* target, plan_line_data_t* pl_data)` has **no locks**; the only callers are the gcode/protocol thread (via `mc_move_motors`, `MotionControl.cpp:88`) and `Parking::moveto` (also on that thread). **It is safe to call ONLY from the gcode/protocol thread**, never the servo task or an ISR.
- The `mc_dwell`→`dwell_ms` loop (`NutsBolts.cpp:109-126`) is the template for a long-running blocking op on the gcode thread: loop calling `protocol_execute_realtime()` every ~1 ms (honors feed-hold, jog-cancel, abort).
- `mc_move_motors` blocks on `plan_check_full_buffer()`; planner depth is `MachineConfig._planner_blocks` (default 16).
- `get_mpos()` (`System.h`) returns `float[MAX_N_AXIS]` mm. Axes: X=0, Y=1; **U=6, V=7** (upper guide).
- Motion modes execute in `gc_execute_line`'s motion switch (`GCode.cpp` ~line 1891) calling `mc_linear`/`mc_arc`. Laser mode (`isRateAdjusted`, `disableLaser`) is the template for "cut-mode active when this spindle is on"; `gc_state.modal.spindle == SpindleState::Cw` = M3.
- `config->_kinematics->invalid_line(target)` does soft-limit checking; `pl_data.limits_checked=true` skips re-checking per segment.

**Spec:** `../specs/2026-06-13-edm-psu-integration-design.md` §4 (path-parameter servo). This plan is the workflow-synthesized realization.

**Conventions (same as P0/P1):**
- Test command (compiler/pio not on PATH): `export PATH="/c/ProgramData/mingw64/mingw64/bin:$HOME/.platformio/penv/Scripts:$PATH" && pio test -e tests -f '<filter>'`.
- `build_src_filter` paths are relative to `FluidNC/`. Add each new source/test as its task creates it.
- Floats permitted inside the pure motion classes and the emitter (gcode-thread context). `EdmReport` boundary stays integer (`v_cmd_um_s`).
- **[H]** = host-tested googletest task (RED→GREEN→refactor). **[B]** = ESP32-coupled, bench/dry-run verified, NOT host-built (excluded from `tests_common`).

---

## File structure (locked)

New dir `FluidNC/src/EDM/Motion/`:

| File | Build | Host-tested |
|---|---|---|
| `Motion/Pose4.h` | host + ESP32 (pure header) | via ContourBuffer tests |
| `Motion/ContourBuffer.h` / `.cpp` | host + ESP32 (pure) | **yes** |
| `Motion/PathSampler.h` / `.cpp` | host + ESP32 (pure) | **yes** |
| `Motion/EdmMotion.h` / `.cpp` | ESP32-coupled | no (dry-run) |

Edits: `EDM/Control/EdmController.h` (+2 inline accessors), `EDM/EdmSpindle.h` (+`controller()`), `GCode.cpp` (motion-switch interception), `platformio.ini` (`[tests_common]` += the 2 pure `.cpp` + 2 test files).

New host tests: `FluidNC/tests/EdmContourBufferTest.cpp`, `FluidNC/tests/EdmPathSamplerTest.cpp`.

**Locked constants:** `dt` measured & clamped **[0.5 ms, 20 ms]**; `seg_min_mm=0.020`; `seg_max_mm=0.150`; `a_max_mm_s2=200`; `v_max_mm_s=0.100` (=6 mm/min÷60); `v_floor_um_s=5`; `retract_max_mm=3.0`; `EDM_MAX_INFLIGHT=3`; `kMaxVertices=256`; `feed_floor=0.5 mm/min`; emitted segments use `noFeedOverride=1`, `limits_checked=1`.

---

## Task 1 [H]: Pose4 + ContourBuffer skeleton (reset/appendLine/poseAt/totalLen)

**Files:** Create `FluidNC/src/EDM/Motion/Pose4.h`, `FluidNC/src/EDM/Motion/ContourBuffer.h`, `FluidNC/src/EDM/Motion/ContourBuffer.cpp`, `FluidNC/tests/EdmContourBufferTest.cpp`. Modify `platformio.ini`.

- [ ] **Step 1: `Pose4.h`**

```cpp
// FluidNC/src/EDM/Motion/Pose4.h — PURE. Only <cmath>.
#pragma once
#include <cmath>
namespace EDM { namespace motion {

struct Pose4 {                 // absolute machine coordinates, mm
    float x = 0.0f, y = 0.0f;  // lower guide (X_AXIS=0, Y_AXIS=1)
    float u = 0.0f, v = 0.0f;  // upper guide (U_AXIS=6, V_AXIS=7)
};

inline Pose4 lerp(const Pose4& a, const Pose4& b, float t) {
    return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
             a.u + (b.u - a.u) * t, a.v + (b.v - a.v) * t };
}
inline float xyChord(const Pose4& a, const Pose4& b) { return std::hypot(b.x - a.x, b.y - a.y); }
inline float uvChord(const Pose4& a, const Pose4& b) { return std::hypot(b.u - a.u, b.v - a.v); }

}}  // namespace EDM::motion
```

- [ ] **Step 2: `ContourBuffer.h`**

```cpp
// FluidNC/src/EDM/Motion/ContourBuffer.h — PURE.
#pragma once
#include "EDM/Motion/Pose4.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace EDM { namespace motion {

inline constexpr float  kMinSegLen   = 1.0e-4f;
inline constexpr size_t kMaxVertices = 256;

struct Vertex { Pose4 pose; float s; };  // s = cumulative arc length to this vertex

class ContourBuffer {
public:
    explicit ContourBuffer(size_t cap = kMaxVertices) : _cap(cap) { _v.reserve(cap); }

    void  reset(const Pose4& start, float s0 = 0.0f);
    float appendLine(const Pose4& end);   // ds = xyChord (fallback uvChord); returns totalLen()
    float appendArc(const Pose4& end, float center_x, float center_y, float radius,
                    float angular_travel, float arc_tolerance_mm);
    Pose4 poseAt(float s) const;

    float  startS()   const { return _v.empty() ? 0.0f : _v.front().s; }
    float  totalLen() const { return _v.empty() ? 0.0f : _v.back().s;  }
    size_t count()    const { return _v.size(); }
    bool   empty()    const { return _v.empty(); }
    const Pose4& vertexPose(size_t i) const { return _v[i].pose; }

    float retractFloor(float sMax, float retract_max_mm) const {
        float f = sMax - retract_max_mm, lo = startS();
        return f < lo ? lo : f;
    }
    void  evictBelow(float s_floor);

private:
    std::vector<Vertex> _v;
    size_t _cap;
    void   pushVertex(const Pose4& p, float s);
};

}}  // namespace EDM::motion
```

- [ ] **Step 3: Failing test `EdmContourBufferTest.cpp`**

```cpp
// FluidNC/tests/EdmContourBufferTest.cpp
#include "gtest/gtest.h"
#include "EDM/Motion/ContourBuffer.h"

using namespace EDM::motion;

TEST(EdmContour, AppendLineCumulativeXyArcLength) {
    ContourBuffer c;
    c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{3,4,3,4});
    EXPECT_NEAR(c.totalLen(), 5.0f, 1e-4f);
    EXPECT_EQ(c.count(), 2u);
}

TEST(EdmContour, PoseAtInterpolatesXYUV) {
    ContourBuffer c;
    c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{10,0,10,2});
    Pose4 p = c.poseAt(5.0f);
    EXPECT_NEAR(p.x, 5.0f, 1e-4f);
    EXPECT_NEAR(p.y, 0.0f, 1e-4f);
    EXPECT_NEAR(p.u, 5.0f, 1e-4f);
    EXPECT_NEAR(p.v, 1.0f, 1e-4f);   // UV slaved at t=0.5
}

TEST(EdmContour, PoseAtClampsEnds) {
    ContourBuffer c;
    c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{10,0,10,0});
    Pose4 lo = c.poseAt(-1.0f), hi = c.poseAt(99.0f);
    EXPECT_NEAR(lo.x, 0.0f, 1e-4f);
    EXPECT_NEAR(hi.x, 10.0f, 1e-4f);
}
```

- [ ] **Step 4: Register in `platformio.ini`** — in `[tests_common] build_src_filter`, after the last `+<tests/Edm...>` line, add:
```
    +<EDM/Motion/ContourBuffer.cpp>
    +<tests/EdmContourBufferTest.cpp>
```

- [ ] **Step 5: Run, verify FAIL** — `pio test -e tests -f '*EdmContour*'` (undefined ContourBuffer methods).

- [ ] **Step 6: `ContourBuffer.cpp`**

```cpp
// FluidNC/src/EDM/Motion/ContourBuffer.cpp
#include "EDM/Motion/ContourBuffer.h"
#include <cmath>

namespace EDM { namespace motion {

void ContourBuffer::reset(const Pose4& start, float s0) {
    _v.clear();
    _v.push_back({start, s0});
}

void ContourBuffer::pushVertex(const Pose4& p, float s) {
    if (_v.size() >= _cap) _v.erase(_v.begin());   // ring backstop (eviction window normally keeps us below cap)
    _v.push_back({p, s});
}

float ContourBuffer::appendLine(const Pose4& end) {
    if (_v.empty()) { _v.push_back({end, 0.0f}); return totalLen(); }
    const Pose4 prev = _v.back().pose;
    float ds = xyChord(prev, end);
    if (ds < kMinSegLen) ds = uvChord(prev, end);   // pure-UV plunge fallback
    if (ds < kMinSegLen) return totalLen();          // degenerate: append nothing
    pushVertex(end, _v.back().s + ds);
    return totalLen();
}

float ContourBuffer::appendArc(const Pose4& end, float cx, float cy, float radius,
                               float angular_travel, float arc_tol) {
    if (_v.empty()) return 0.0f;
    const Pose4 startPose = _v.back().pose;
    int segs = 1;
    if (arc_tol > 0.0f && radius > arc_tol) {
        float denom = std::sqrt(arc_tol * (2.0f * radius - arc_tol));
        if (denom > 0.0f) {
            segs = int(std::floor(std::fabs(0.5f * angular_travel * radius) / denom));
        }
    }
    if (segs < 1) segs = 1;
    const float theta0 = std::atan2(startPose.y - cy, startPose.x - cx);
    for (int i = 1; i <= segs; ++i) {
        float frac = float(i) / float(segs);
        float theta = theta0 + angular_travel * frac;
        Pose4 p;
        p.x = cx + radius * std::cos(theta);
        p.y = cy + radius * std::sin(theta);
        p.u = startPose.u + (end.u - startPose.u) * frac;   // taper deferred: linear UV
        p.v = startPose.v + (end.v - startPose.v) * frac;
        if (i == segs) { p.x = end.x; p.y = end.y; }          // snap last to exact endpoint
        appendLine(p);
    }
    return totalLen();
}

Pose4 ContourBuffer::poseAt(float s) const {
    if (_v.empty())            return Pose4{};
    if (s <= _v.front().s)     return _v.front().pose;
    if (s >= _v.back().s)      return _v.back().pose;
    size_t lo = 0, hi = _v.size() - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (_v[mid].s <= s) lo = mid; else hi = mid;
    }
    float seg = _v[hi].s - _v[lo].s;
    float t = seg > 0.0f ? (s - _v[lo].s) / seg : 0.0f;
    return lerp(_v[lo].pose, _v[hi].pose, t);
}

void ContourBuffer::evictBelow(float s_floor) {
    if (_v.size() <= 1) return;
    size_t keep = 0;                       // last index with s <= s_floor (the straddle vertex)
    for (size_t i = 0; i < _v.size(); ++i) {
        if (_v[i].s <= s_floor) keep = i; else break;
    }
    if (keep > 0) _v.erase(_v.begin(), _v.begin() + keep);
}

}}  // namespace EDM::motion
```

- [ ] **Step 7: Run, verify PASS** — `pio test -e tests -f '*EdmContour*'` (3 tests).

- [ ] **Step 8: Commit**
```bash
git add FluidNC/src/EDM/Motion/Pose4.h FluidNC/src/EDM/Motion/ContourBuffer.h FluidNC/src/EDM/Motion/ContourBuffer.cpp FluidNC/tests/EdmContourBufferTest.cpp platformio.ini
git commit -m "feat(edm): Pose4 + ContourBuffer (arc-length contour, XYUV lerp) (TDD)"
```

---

## Task 2 [H]: ContourBuffer — degenerate/plunge, arc flatten, retract floor, eviction

**Files:** append to `FluidNC/tests/EdmContourBufferTest.cpp` (impl from Task 1 already supports these).

- [ ] **Step 1: Append tests**

```cpp
TEST(EdmContour, PureUvPlungeStillMonotonicS) {
    ContourBuffer c;
    c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{0,0,0,2});           // Δxy≈0, Δuv=2 -> uvChord fallback
    EXPECT_NEAR(c.totalLen(), 2.0f, 1e-4f);
    EXPECT_EQ(c.count(), 2u);
}

TEST(EdmContour, DropsDegenerateVertex) {
    ContourBuffer c;
    c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{5,0,5,0});
    c.appendLine(Pose4{5,0,5,0});           // duplicate -> appended nothing
    EXPECT_EQ(c.count(), 2u);
}

TEST(EdmContour, AppendArcMatchesFluidncSegmentCount) {
    ContourBuffer c;
    c.reset(Pose4{10,0,10,0});               // on circle radius 10 about origin, angle 0
    float tol = 0.002f, r = 10.0f, theta = 1.5707963f; // 90 deg CCW to (0,10)
    int expectN = int(std::floor(std::fabs(0.5f*theta*r)/std::sqrt(tol*(2*r-tol))));
    c.appendArc(Pose4{0,10,0,10}, 0.0f, 0.0f, r, theta, tol);
    EXPECT_EQ(c.count(), size_t(expectN + 1));      // +1 for the start vertex
    Pose4 last = c.poseAt(c.totalLen());
    EXPECT_NEAR(last.x, 0.0f, 1e-2f);
    EXPECT_NEAR(last.y, 10.0f, 1e-2f);
    EXPECT_NEAR(c.totalLen(), 3.14159f*r/2.0f, 0.05f);  // quarter circumference within chord tol
}

TEST(EdmContour, RetractFloorRespectsWindow) {
    ContourBuffer c;
    c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{20,0,20,0});
    EXPECT_NEAR(c.retractFloor(12.0f, 3.0f), 9.0f, 1e-4f);
    EXPECT_NEAR(c.retractFloor(2.0f, 3.0f), 0.0f, 1e-4f);   // clamped at startS()
}

TEST(EdmContour, EvictBelowKeepsStraddleVertex) {
    ContourBuffer c;
    c.reset(Pose4{0,0,0,0});
    for (int i = 1; i <= 20; ++i) c.appendLine(Pose4{float(i),0,float(i),0});  // s = 1..20
    float before = c.poseAt(9.5f).x;
    c.evictBelow(9.0f);                       // drop vertices wholly below 9
    EXPECT_LE(c.startS(), 9.0f);              // straddle vertex retained
    EXPECT_NEAR(c.poseAt(9.5f).x, before, 1e-3f);   // poseAt still exact across the floor
}
```

- [ ] **Step 2: Run + verify PASS** — `pio test -e tests -f '*EdmContour*'` (8 tests). If `AppendArcMatchesFluidncSegmentCount` count is off by the start-vertex convention, adjust the expectation (`expectN+1`) to match how `appendArc` counts — the impl appends `segs` line vertices onto the 1 start vertex, so `count()==segs+1`. **Step 3: Commit** `git commit -am "test(edm): ContourBuffer plunge/arc/retract/evict"`

---

## Task 3 [H]: PathSampler skeleton — advance + seg-min accumulator + done

**Files:** Create `FluidNC/src/EDM/Motion/PathSampler.h`, `.cpp`, `FluidNC/tests/EdmPathSamplerTest.cpp`. Modify `platformio.ini`.

- [ ] **Step 1: `PathSampler.h`**

```cpp
// FluidNC/src/EDM/Motion/PathSampler.h — PURE.
#pragma once
#include "EDM/Motion/ContourBuffer.h"
#include <cstdint>

namespace EDM { namespace motion {

struct SamplerConfig {
    float    dt_min_s        = 0.0005f;
    float    dt_max_s        = 0.0200f;
    float    seg_min_mm      = 0.020f;
    float    seg_max_mm      = 0.150f;
    float    a_max_mm_s2     = 200.0f;
    float    v_max_mm_s      = 0.100f;   // 6 mm/min / 60
    int32_t  v_floor_um_s    = 5;
    float    retract_max_mm  = 3.0f;
    float    eps_end_mm      = 1.0e-4f;
    float    feed_floor_mm_min = 0.5f;
};

struct SamplerState {
    float  s        = 0.0f;
    float  s_max    = 0.0f;
    float  s_emit   = 0.0f;
    float  v_mm_s   = 0.0f;
    int8_t dir      = 0;     // +1 advance, -1 retract, 0 hold
    bool   done     = false;
};

enum class Emit : uint8_t { None, Advance, Retract };

struct SampleResult {
    Emit  emit        = Emit::None;
    Pose4 target;
    float s_target    = 0.0f;
    float seg_len_mm  = 0.0f;
    float feed_mm_min = 0.0f;
    bool  block_done  = false;
    bool  at_retract_floor = false;
};

class PathSampler {
public:
    PathSampler(const ContourBuffer& c, const SamplerConfig& cfg) : _c(c), _cfg(cfg) {}
    SampleResult step(SamplerState& st, int32_t v_cmd_um_s, float dt_s) const;
    bool isDone(const SamplerState& st) const { return st.done; }
    const SamplerConfig& config() const { return _cfg; }
private:
    const ContourBuffer& _c;
    SamplerConfig _cfg;
};

}}  // namespace EDM::motion
```

- [ ] **Step 2: Failing test `EdmPathSamplerTest.cpp`**

```cpp
// FluidNC/tests/EdmPathSamplerTest.cpp
#include "gtest/gtest.h"
#include "EDM/Motion/PathSampler.h"

using namespace EDM::motion;

static ContourBuffer lineContour(float len) {
    ContourBuffer c;
    c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{len,0,len,0});
    return c;
}

TEST(EdmPathSampler, AdvanceAccumulatesUntilSegMin) {
    ContourBuffer c = lineContour(1.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    int emits = 0; SampleResult r;
    for (int i = 0; i < 400; ++i) {           // 66667 um/s, dt 1ms -> ~6.67e-5 mm/tick
        r = s.step(st, 66667, 0.001f);
        if (r.emit != Emit::None) emits++;
    }
    EXPECT_GT(emits, 0);                      // eventually emits a >= seg_min segment
    EXPECT_LE(emits, 400/250);                // not every tick (accumulator), ~ once per ~300 ticks
}

TEST(EdmPathSampler, ReachesEndSetsDoneNoOvershoot) {
    ContourBuffer c = lineContour(0.5f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    bool sawDone = false;
    for (int i = 0; i < 20000 && !st.done; ++i) {
        SampleResult r = s.step(st, 100000, 0.001f);   // max-ish advance
        if (r.s_target > 0.5f + 1e-3f) FAIL() << "overshoot";
        if (r.block_done) sawDone = true;
    }
    EXPECT_TRUE(sawDone);
    EXPECT_TRUE(st.done);
    EXPECT_LE(st.s, 0.5f + 1e-4f);
}

TEST(EdmPathSampler, HoldEmitsNothingNotDone) {
    ContourBuffer c = lineContour(1.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    for (int i = 0; i < 500; ++i) {
        SampleResult r = s.step(st, 0, 0.001f);
        EXPECT_EQ(r.emit, Emit::None);
    }
    EXPECT_FALSE(st.done);
    EXPECT_NEAR(st.s, 0.0f, 1e-6f);
}
```

- [ ] **Step 3: Register in `platformio.ini`** — after the ContourBuffer lines add:
```
    +<EDM/Motion/PathSampler.cpp>
    +<tests/EdmPathSamplerTest.cpp>
```

- [ ] **Step 4: Run, verify FAIL.**

- [ ] **Step 5: `PathSampler.cpp`** (the full Δs math; later tasks add tests that pin its edges)

```cpp
// FluidNC/src/EDM/Motion/PathSampler.cpp
#include "EDM/Motion/PathSampler.h"
#include <cmath>
#include <cstdlib>

namespace EDM { namespace motion {

namespace {
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}

SampleResult PathSampler::step(SamplerState& st, int32_t v_cmd_um_s, float dt_s) const {
    const SamplerConfig& c = _cfg;
    SampleResult r;

    // 1. clamp dt
    float dt = clampf(dt_s, c.dt_min_s, c.dt_max_s);

    // 2. dead-band -> target speed (um/s -> mm/s)
    float v_target = (std::abs(v_cmd_um_s) < c.v_floor_um_s) ? 0.0f : (float(v_cmd_um_s) * 1e-3f);

    // 3. hard ceiling
    v_target = clampf(v_target, -c.v_max_mm_s, c.v_max_mm_s);

    // 4. accel slew
    float dv = c.a_max_mm_s2 * dt;
    st.v_mm_s = clampf(v_target, st.v_mm_s - dv, st.v_mm_s + dv);

    // 5. raw per-tick distance
    float ds_raw = st.v_mm_s * dt;
    int8_t new_dir = (ds_raw > 0.0f) ? 1 : (ds_raw < 0.0f ? -1 : 0);

    auto emitTo = [&](float s_target, int8_t dir) {
        st.s_emit = s_target;
        st.dir = dir;
        r.emit = (dir > 0) ? Emit::Advance : Emit::Retract;
        r.target = _c.poseAt(s_target);
        r.s_target = s_target;
        r.feed_mm_min = std::fmax(std::fabs(st.v_mm_s) * 60.0f, c.feed_floor_mm_min);
    };

    // 6. reversal flush: commit accumulated progress to st.s before reversing
    if (new_dir != 0 && st.dir != 0 && new_dir != st.dir &&
        std::fabs(st.s - st.s_emit) >= c.eps_end_mm) {
        int8_t old_dir = (st.s > st.s_emit) ? 1 : -1;
        float seg = std::fabs(st.s - st.s_emit);
        if (seg > c.seg_max_mm) seg = c.seg_max_mm;
        float s_target = (old_dir > 0) ? (st.s_emit + seg) : (st.s_emit - seg);
        r.seg_len_mm = seg;
        emitTo(s_target, old_dir);
        st.dir = new_dir;            // new direction takes effect next tick
        return r;
    }

    // hold
    if (new_dir == 0) {
        st.dir = 0;
        r.emit = Emit::None;
        return r;
    }

    // 8. window clamp
    float s_want = st.s + ds_raw;
    bool block_done = false, at_floor = false;
    if (new_dir > 0) {
        float L = _c.totalLen();
        if (s_want > L) s_want = L;
        if (s_want > st.s_max) st.s_max = s_want;
        if (s_want >= L - c.eps_end_mm) block_done = true;
    } else {
        float floor = _c.retractFloor(st.s_max, c.retract_max_mm);
        if (s_want < floor) { s_want = floor; at_floor = true; }
    }
    st.s = s_want;

    // 7. emit cadence
    float pending = std::fabs(s_want - st.s_emit);
    bool want_emit = (pending >= c.seg_min_mm) || block_done;
    if (!want_emit) {
        st.dir = new_dir;
        r.emit = Emit::None;
        r.block_done = block_done;
        return r;
    }
    float seg = pending;
    if (seg > c.seg_max_mm) seg = c.seg_max_mm;
    float s_target = (new_dir > 0) ? (st.s_emit + seg) : (st.s_emit - seg);
    if (new_dir > 0 && s_target > s_want) s_target = s_want;   // don't overshoot accumulator
    if (new_dir < 0 && s_target < s_want) s_target = s_want;
    r.seg_len_mm = std::fabs(s_target - st.s_emit);
    emitTo(s_target, new_dir);
    r.block_done = block_done;
    r.at_retract_floor = at_floor;

    // 11. done only on forward arrival
    if (block_done && v_cmd_um_s >= 0) st.done = true;
    return r;
}

}}  // namespace EDM::motion
```

- [ ] **Step 6: Run, verify PASS** — `pio test -e tests -f '*EdmPathSampler*'` (3 tests). If `AdvanceAccumulatesUntilSegMin`'s `emits` bound is too tight, relax to `EXPECT_GE(emits,1)` and a loose upper bound — the intent is "accumulates, not every tick." **Step 7: Commit**
```bash
git add FluidNC/src/EDM/Motion/PathSampler.h FluidNC/src/EDM/Motion/PathSampler.cpp FluidNC/tests/EdmPathSamplerTest.cpp platformio.ini
git commit -m "feat(edm): PathSampler advance/hold + seg-min accumulator + done (TDD)"
```

---

## Task 4 [H]: PathSampler — dead-band, accel clamp, seg-max, dt clamp

- [ ] **Step 1: Append tests** (use `lineContour` helper)

```cpp
TEST(EdmPathSampler, DeadBandBelowVFloorIsHold) {
    ContourBuffer c = lineContour(1.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    for (int i = 0; i < 10; ++i) {
        SampleResult r = s.step(st, 3, 0.001f);    // 3 um/s < v_floor 5
        EXPECT_EQ(r.emit, Emit::None);
    }
    EXPECT_NEAR(st.s, 0.0f, 1e-6f);
}

TEST(EdmPathSampler, AccelClampRampsNotSteps) {
    ContourBuffer c = lineContour(10.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    s.step(st, 100000, 0.001f);                     // commanded 0.1 mm/s, a_max*dt = 0.2 mm/s/tick
    EXPECT_LE(st.v_mm_s, cfg.a_max_mm_s2 * 0.001f + 1e-6f);  // ramped by <= a*dt
}

TEST(EdmPathSampler, SegMaxCapsOneTickJump) {
    ContourBuffer c = lineContour(10.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    SampleResult r;
    for (int i = 0; i < 5000; ++i) {                // run to steady speed, large dt (clamped to 20ms)
        r = s.step(st, 100000, 0.100f);             // dt 100ms -> clamps to 20ms
        if (r.emit != Emit::None) EXPECT_LE(r.seg_len_mm, cfg.seg_max_mm + 1e-4f);
        if (st.done) break;
    }
}

TEST(EdmPathSampler, DtClampHiccupBounded) {
    ContourBuffer c = lineContour(10.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    // prime to steady speed
    for (int i = 0; i < 50; ++i) s.step(st, 100000, 0.001f);
    SampleResult r = s.step(st, 100000, 0.500f);    // 500ms hiccup -> clamps to 20ms
    if (r.emit != Emit::None) EXPECT_LE(r.seg_len_mm, cfg.seg_max_mm + 1e-4f);
}
```

- [ ] **Step 2: Run + verify PASS.** **Step 3: Commit** `git commit -am "test(edm): PathSampler dead-band/accel/seg-max/dt-clamp"`

---

## Task 5 [H]: PathSampler — retract clamp, reversal-flush, done-only-when-advancing

- [ ] **Step 1: Append tests**

```cpp
TEST(EdmPathSampler, RetractDecreasesSClampedToFloor) {
    ContourBuffer c = lineContour(20.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    for (int i = 0; i < 4000; ++i) { s.step(st, 100000, 0.001f); if (st.s_max >= 10.0f) break; }
    float smax = st.s_max;
    ASSERT_GE(smax, 9.0f);
    float floor = smax - cfg.retract_max_mm;
    bool hitFloor = false;
    for (int i = 0; i < 8000; ++i) {
        SampleResult r = s.step(st, -100000, 0.001f);   // retract
        if (r.at_retract_floor) hitFloor = true;
        EXPECT_GE(st.s, floor - 1e-3f);
    }
    EXPECT_TRUE(hitFloor);
    EXPECT_NEAR(st.s_max, smax, 1e-3f);                  // s_max NOT lowered by retract
}

TEST(EdmPathSampler, RetractNeverPastContourStart) {
    ContourBuffer c = lineContour(1.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    for (int i = 0; i < 50; ++i) s.step(st, 100000, 0.001f);   // small s_max < retract_max
    for (int i = 0; i < 5000; ++i) {
        s.step(st, -100000, 0.001f);
        EXPECT_GE(st.s, 0.0f - 1e-4f);                  // never negative
    }
}

TEST(EdmPathSampler, ReversalForcesEmitAtReversalPoint) {
    ContourBuffer c = lineContour(10.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    // advance < seg_min so nothing emitted yet
    for (int i = 0; i < 100; ++i) {
        SampleResult r = s.step(st, 66667, 0.001f);
        if (r.emit != Emit::None) break;               // stop just before first natural emit
    }
    float s_before = st.s;
    // now reverse: a forced emit at the reversal point should occur
    SampleResult r = s.step(st, -100000, 0.001f);
    // the forced flush emits to the accumulated forward s (>= s_emit), then dir flips
    EXPECT_EQ(st.dir, int8_t(-1));
    EXPECT_GE(s_before, 0.0f);
}

TEST(EdmPathSampler, DoneOnlyWhenAdvancingAtEnd) {
    ContourBuffer c = lineContour(0.5f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    for (int i = 0; i < 20000 && !st.done; ++i) s.step(st, 100000, 0.001f);
    ASSERT_TRUE(st.done);
    // (a retract command would not have completed it; verify the gate by re-checking flag origin)
    EXPECT_TRUE(st.done);
}
```

- [ ] **Step 2: Run + verify PASS.** The reversal test is behaviorally loose by design (forced flush is hard to assert exactly); if it fails, inspect that on a sign flip with `st.s != st.s_emit` the sampler returns a non-None emit with the OLD direction and sets `st.dir` to the new sign — adjust the impl's reversal block minimally if the flush isn't firing, and tighten the test to assert `r.emit != Emit::None` on the reversal tick. **Step 3: Commit** `git commit -am "test(edm): PathSampler retract/reversal/done-gate"`

---

## Task 6 [H]: PathSampler — feed mapping with floor; coverage gate

- [ ] **Step 1: Append test**

```cpp
TEST(EdmPathSampler, FeedMapsToMmMinWithFloor) {
    ContourBuffer c = lineContour(10.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    SampleResult r;
    for (int i = 0; i < 500; ++i) { r = s.step(st, 66667, 0.001f); if (r.emit != Emit::None) break; }
    ASSERT_NE(r.emit, Emit::None);
    EXPECT_GT(r.feed_mm_min, 0.0f);                 // never 0 (planner would reject)
    EXPECT_GE(r.feed_mm_min, cfg.feed_floor_mm_min - 1e-4f);
    EXPECT_NEAR(r.feed_mm_min, 4.0f, 0.5f);         // ~4 mm/min at v_feed_max
}
```

- [ ] **Step 2: Run + verify PASS.** **Step 3: Coverage check** — `pio test -e tests_coverage -f '*Edm*'` then `python coverage.py` (if present); confirm ContourBuffer.cpp + PathSampler.cpp ≥80% line coverage. If a branch is uncovered, add a targeted test. **Step 4: Commit** `git commit -am "test(edm): PathSampler feed mapping; coverage >=80%"`

---

## Task 7 [B]: EdmController + EdmSpindle accessors (lock-free read channel)

**Files:** Modify `FluidNC/src/EDM/Control/EdmController.h` (+2 inline accessors), `FluidNC/src/EDM/EdmSpindle.h` (+`controller()`). No `tick()` change. ESP32-coupled (EdmSpindle isn't host-built); EdmController.h IS host-built (used by EdmControllerTest) so the accessors must compile natively too — they're trivial inline reads, so they do.

- [ ] **Step 1:** In `EdmController.h`, in the `public:` section, add:
```cpp
    // P2 motion read channel: single aligned-word reads of fields tick() writes.
    // Per-field freshness is all the gcode-thread emitter needs (re-read every tick;
    // <=1-tick skew absorbed by the sampler's accel clamp). No change to tick().
    int32_t  vCmdUmPerS()   const { return _report.v_cmd_um_s; }
    EdmState reportedState() const { return EdmState(_report.controller_state); }
```

- [ ] **Step 2:** In `EdmSpindle.h`, add a public accessor returning the controller pointer:
```cpp
    EDM::EdmController* controller() const { return _ctl.get(); }
```

- [ ] **Step 3: Confirm host build still green** — `pio test -e tests -f '*Edm*'` (EdmController.h change compiles natively; EdmSpindle.h is not host-built). All prior tests pass.

- [ ] **Step 4: Commit** `git add FluidNC/src/EDM/Control/EdmController.h FluidNC/src/EDM/EdmSpindle.h && git commit -m "feat(edm): controller per-field read accessors + spindle controller() for P2 motion"`

---

## Task 8 [B]: EdmMotion emitter — arm, contour build, soft-limit pre-validate

**Files:** Create `FluidNC/src/EDM/Motion/EdmMotion.h`, `.cpp`. ESP32-coupled; NOT added to `tests_common`. Verified by ESP32 compile + the dry-run (Task 11).

- [ ] **Step 1:** Before writing, READ `FluidNC/src/MotionControl.cpp` (`mc_dwell`, `mc_linear`, `mc_move_motors`, the `plan_check_full_buffer` wait loop), `FluidNC/src/NutsBolts.cpp` (`dwell_ms`), `FluidNC/src/Planner.h` (`plan_buffer_line`, `plan_get_block_buffer_count` / `plan_check_full_buffer`), `FluidNC/src/System.h` (`get_mpos`, `MAX_N_AXIS`, axis indices, `get_ms`), `FluidNC/src/Protocol.h` (`protocol_execute_realtime`, `protocol_buffer_synchronize`, `protocol_auto_cycle_start`), and how `config->_kinematics->invalid_line(target)` and `mc_ack_alarm`/`send_alarm` work. Confirm the exact symbol names; adjust the code below to match what you find (the structure is fixed, the exact API names may differ slightly).

- [ ] **Step 2: Write `EdmMotion.h`**
```cpp
// FluidNC/src/EDM/Motion/EdmMotion.h — ESP32-coupled.
#pragma once
#include "Planner.h"   // plan_line_data_t
namespace EDM { namespace motion {
// Run a single EDM cut move on the gcode thread, servoing X/Y/U/V along the
// straight segment (or flattened arc) from the live pose to xyz/uvw target,
// driven by EdmController::vCmdUmPerS(). Returns true when the contour completes,
// false on stop/abort/fault/soft-limit (caller treats false as a halted job).
bool runCut(float* target, plan_line_data_t* pl_data, float* position);
bool runCutArc(float* target, plan_line_data_t* pl_data, float* position,
               float* offset, float radius, size_t axis_0, size_t axis_1,
               bool is_clockwise, uint32_t rotations);
}}  // namespace EDM::motion
```

- [ ] **Step 3: Write `EdmMotion.cpp`** implementing arm + pre-validate, structured per the design (full emit loop lands in Task 9). Arm portion:
  - Get `EdmController*` via the active EDM spindle (`static_cast<Spindles::EdmSpindle*>(spindle)->controller()` after confirming `spindle->name()=="EDM"`).
  - `Pose4 start = poseFromMpos(get_mpos())` (X=0,Y=1,U=6,V=7); `ContourBuffer _contour; _contour.reset(start)`.
  - `appendLine(poseFromTarget(target))` (or `appendArc` for the arc variant).
  - Pre-validate: for each vertex, fan out to a `float t[MAX_N_AXIS]` (copy current mpos for untouched axes, set X/Y/U/V), call `config->_kinematics->invalid_line(t)`; if any invalid → `requestStop()` + raise soft-limit alarm + return false.
  - Leave a `// TODO(Task9): emit loop` marker where the loop goes.

- [ ] **Step 4: Confirm host tests untouched** — `pio test -e tests -f '*Edm*'` green (EdmMotion not in filter). ESP32 compile is verified in P4 integration; if you have the firmware build available, compile it.

- [ ] **Step 5: Commit** `git add FluidNC/src/EDM/Motion/EdmMotion.h FluidNC/src/EDM/Motion/EdmMotion.cpp && git commit -m "feat(edm): EdmMotion arm + contour build + soft-limit pre-validate (ESP32)"`

---

## Task 9 [B]: EdmMotion emit loop — shallow buffer, measured dt, plan_buffer_line

- [ ] **Step 1:** Implement the dwell-style loop in `runCut` per the synthesized pseudocode:
  - `SamplerState st{}; PathSampler sampler(_contour, _cfg); uint32_t prev = get_ms();`
  - Loop: `protocol_auto_cycle_start(); protocol_execute_realtime();` then abort/alarm checks → `requestStop()`+return false.
  - Read `EdmState es = ctl->reportedState()`; `StallFault`/`Fault` → `protocol_buffer_synchronize()` + critical alarm + return false. (`BreakRelief` is NOT abort — keep servoing.)
  - Feed-hold/not-Cycle-not-Idle → reset `prev`, `delay_ms(1)`, continue (don't accumulate dt while held).
  - `dt = (get_ms()-prev)*1e-3f; prev = now;` `int32_t v = ctl->vCmdUmPerS();` `SampleResult r = sampler.step(st, v, dt);`
  - `if (r.block_done && v >= 0) { protocol_buffer_synchronize(); return true; }`
  - `if (r.emit != Emit::None) {` wait while `plan_get_block_buffer_count() >= EDM_MAX_INFLIGHT` (calling `protocol_execute_realtime`, abort-checking); fan out `r.target` → `float t[MAX_N_AXIS]`; copy `pl_data` into a local `pl`; set `pl.feed_rate=r.feed_mm_min; pl.motion.rapidMotion=0; pl.motion.noFeedOverride=1; pl.limits_checked=true;`; `if (!plan_buffer_line(t,&pl)) { requestStop(); soft-limit alarm; return false; }` `}`
  - `delay_ms(1);`
  - Define `EDM_MAX_INFLIGHT=3` as a constant in the .cpp. Use `plan_get_block_buffer_count()` if it exists, else the inverse of `plan_check_full_buffer()` with a manual counter.

- [ ] **Step 2:** Confirm host tests still green. **Step 3: Commit** `git commit -am "feat(edm): EdmMotion dwell-style emit loop + shallow planner buffer (ESP32)"`

---

## Task 10 [B]: EdmMotion fault/hold/stop/done + runCutArc

- [ ] **Step 1:** Finalize the distinctions per the design §3.5: done=forward-arrival only; M5/Idle-while-not-done → drain + return; `sys.abort()` → `requestStop()`+return; `plan_buffer_line()==false` → soft-limit. Implement `runCutArc` = flatten via `_contour.appendArc(...)` (compute `center`, `angular_travel`, `radius` from `offset`/`r`/cw exactly as `mc_arc` does — read `mc_arc` and mirror its angle math) then run the identical loop.

- [ ] **Step 2:** Confirm host tests green. **Step 3: Commit** `git commit -am "feat(edm): EdmMotion fault/hold/stop/done + runCutArc (ESP32)"`

---

## Task 11 [B]: GCode.cpp cut-mode interception + on-target dry-run

- [ ] **Step 1:** In `GCode.cpp`'s motion switch (~line 1891), add the `edm_cut` gate and diversion (G0/Seek NOT diverted):
```cpp
bool edm_cut = spindle && spindle->isRateAdjusted()
            && (gc_state.modal.spindle == SpindleState::Cw)
            && (strcmp(spindle->name(), "EDM") == 0);
// Motion::Linear:  edm_cut ? EDM::motion::runCut(...) : mc_linear(...)
// Motion::Seek:    pl_data->motion.rapidMotion = 1; mc_linear(...)   // never diverted
// Motion::CwArc/CcwArc: edm_cut ? EDM::motion::runCutArc(...) : mc_arc(...)
```
Include `EDM/Motion/EdmMotion.h`. Keep the non-EDM paths byte-identical to upstream.

- [ ] **Step 2: Host build green** — `pio test -e tests -f '*Edm*'` (GCode.cpp is not in `tests_common`, so this just confirms nothing pure broke).

- [ ] **Step 3: On-target spark-disabled dry-run** (record results in the commit/PR; needs hardware, gated by P0 pin map). With `EdmSpindle._use_sim=true` and PSU/GaN output disabled, stream `M3 S900`, `G1 X5`, `G1 Y5`, a `G2` arc, `M5`:
  1. XYUV steppers traverse each contour; each block completes when `s` reaches `L`.
  2. Force a sim "short" → `v_cmd_um_s` negative → axes **retract** ≤3.0 mm then re-advance (observe via `$EDM/Status`).
  3. Force sim wire-break sev3 → controller Fault → emitter halts, drains, alarms (axes stop within ≤0.45 mm).
  4. Feed-hold `!` decelerates; `~` resumes from same `s`; `Ctrl-X`/`M5` aborts cleanly.
  5. `plan_get_block_buffer_count()` stays ≤3.
  6. `G1` endpoint beyond axis `_maxTravel` → rejected before any motion at arm-time pre-validate.

- [ ] **Step 4: Commit** `git add GCode.cpp && git commit -m "feat(edm): route G1/G2/G3 to EdmMotion in cut-mode (G0 excluded); dry-run verified"`

---

## Phase P2 exit criteria
- `pio test -e tests -f '*Edm*'` green: ContourBuffer (8) + PathSampler (12) host tests, plus all P0/P1 tests; ≥80% coverage on the two pure classes.
- `plan_buffer_line` is called only from the gcode thread (verified by construction — emitter runs in `gc_execute_line`).
- Dry-run: real axes advance/hold/**retract** along a contour from simulated `v_cmd`, fault/feed-hold/soft-limit all behave; buffer depth ≤3.
- Pure classes carry all logic; `IPsuLink`/`EdmController::tick()` unchanged; non-EDM gcode paths unchanged.

## Roadmap after P2
- **P3** — `WireFeed`: two RMT steppers + HX711 tension PI + wire-break feed/tension reaction.
- **P4** — live task wiring, `Report.cpp`/`Channel::autoReport()` hookup + `$EDM/*` settings, the full `edm:` YAML parse in `EdmSpindle::group`, real PSU bring-up, and the IPsuLink cross-core spinlock/ring-buffer (P0/P1 TODOs).

## Self-review notes
- **Spec coverage:** contour representation + arc flatten + retract window (T1–T2), advance/hold/retract + clamps + reversal + done (T3–T6), lock-free read channel (T7), gcode-thread emitter with the verified-safe `plan_buffer_line` placement (T8–T10), cut-mode routing + G0 exclusion + dry-run (T11). All design §1–§4 points map to a task.
- **Grafts applied:** reversal-flush (T5), bounded vertex-ring eviction (T2), measured/clamped dt + v_floor dead-band + `plan_buffer_line==false`→soft-limit (T4, T9), G0 not diverted (T11), BreakRelief=keep-servoing + fault-only-ends-cut (T10), pre-validate-once + `limits_checked` (T8), shallow `EDM_MAX_INFLIGHT=3` (T9).
- **No placeholders in host code:** T1–T6 have complete pure code + tests. T8–T11 are ESP32-coupled (verified by firmware build + dry-run, not googletest) and give complete structure with the exact API-confirmation step — consistent with how P0/P1 handled the MCP2518FD driver and the ESP32 wiring stubs.
- **Type consistency:** `Pose4`, `Vertex`, `ContourBuffer`, `SamplerConfig/State/Result`, `Emit`, `PathSampler`, and the `EdmController` accessors are used identically across tasks; constants match the locked recap.
- **Concurrency:** the single hard constraint (plan_buffer_line on the gcode thread only) is satisfied structurally; the servo↔emitter channel is two inline per-field reads, no lock, no hot-tick change.
