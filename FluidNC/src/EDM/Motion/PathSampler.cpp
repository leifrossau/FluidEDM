// FluidNC/src/EDM/Motion/PathSampler.cpp
#include "EDM/Motion/PathSampler.h"
#include <cmath>
#include <cstdlib>

namespace EDM { namespace motion {

namespace { inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); } }

SampleResult PathSampler::step(SamplerState& st, int32_t v_cmd_um_s, float dt_s) const {
    const SamplerConfig& c = _cfg;
    SampleResult r;

    float dt = clampf(dt_s, c.dt_min_s, c.dt_max_s);
    float v_target = (std::abs(v_cmd_um_s) < c.v_floor_um_s) ? 0.0f : (float(v_cmd_um_s) * 1e-3f);
    v_target = clampf(v_target, -c.v_max_mm_s, c.v_max_mm_s);

    float dv = c.a_max_mm_s2 * dt;
    st.v_mm_s = clampf(v_target, st.v_mm_s - dv, st.v_mm_s + dv);

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

    // reversal flush: commit accumulated progress to st.s before reversing
    if (new_dir != 0 && st.dir != 0 && new_dir != st.dir &&
        std::fabs(st.s - st.s_emit) >= c.eps_end_mm) {
        int8_t old_dir = (st.s > st.s_emit) ? 1 : -1;
        float seg = std::fabs(st.s - st.s_emit);
        if (seg > c.seg_max_mm) seg = c.seg_max_mm;
        float s_target = (old_dir > 0) ? (st.s_emit + seg) : (st.s_emit - seg);
        r.seg_len_mm = seg;
        emitTo(s_target, old_dir);
        st.dir = new_dir;
        return r;
    }

    if (new_dir == 0) { st.dir = 0; r.emit = Emit::None; return r; }

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

    float pending = std::fabs(s_want - st.s_emit);
    bool want_emit = (pending >= c.seg_min_mm) || block_done;
    if (!want_emit) {
        st.dir = new_dir;
        r.emit = Emit::None;
        r.block_done = block_done;
        r.at_retract_floor = at_floor;  // surface floor even when accumulating (no emit yet)
        return r;
    }

    float seg = pending;
    if (seg > c.seg_max_mm) seg = c.seg_max_mm;
    float s_target = (new_dir > 0) ? (st.s_emit + seg) : (st.s_emit - seg);
    if (new_dir > 0 && s_target > s_want) s_target = s_want;
    if (new_dir < 0 && s_target < s_want) s_target = s_want;
    r.seg_len_mm = std::fabs(s_target - st.s_emit);
    emitTo(s_target, new_dir);
    r.block_done = block_done;
    r.at_retract_floor = at_floor;

    if (block_done && v_cmd_um_s >= 0) st.done = true;
    return r;
}

}}  // namespace EDM::motion
