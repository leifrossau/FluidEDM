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
    int8_t dir      = 0;
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
