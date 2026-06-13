// FluidNC/src/EDM/Motion/Pose4.h — PURE. Only <cmath>.
#pragma once
#include <cmath>
namespace EDM { namespace motion {

struct Pose4 {
    float x = 0.0f, y = 0.0f;  // lower guide
    float u = 0.0f, v = 0.0f;  // upper guide
};

inline Pose4 lerp(const Pose4& a, const Pose4& b, float t) {
    return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
             a.u + (b.u - a.u) * t, a.v + (b.v - a.v) * t };
}
inline float xyChord(const Pose4& a, const Pose4& b) { return std::hypot(b.x - a.x, b.y - a.y); }
inline float uvChord(const Pose4& a, const Pose4& b) { return std::hypot(b.u - a.u, b.v - a.v); }

}}  // namespace EDM::motion
