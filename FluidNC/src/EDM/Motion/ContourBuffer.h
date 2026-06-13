// FluidNC/src/EDM/Motion/ContourBuffer.h — PURE.
#pragma once
#include "EDM/Motion/Pose4.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace EDM { namespace motion {

inline constexpr float  kMinSegLen   = 1.0e-4f;
inline constexpr size_t kMaxVertices = 256;

struct Vertex { Pose4 pose; float s; };

class ContourBuffer {
public:
    explicit ContourBuffer(size_t cap = kMaxVertices) : _cap(cap) { _v.reserve(cap); }

    void  reset(const Pose4& start, float s0 = 0.0f);
    float appendLine(const Pose4& end);
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
