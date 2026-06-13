// FluidNC/src/EDM/Motion/ContourBuffer.cpp
#include "EDM/Motion/ContourBuffer.h"
#include <cmath>

namespace EDM { namespace motion {

void ContourBuffer::reset(const Pose4& start, float s0) {
    _v.clear();
    _v.push_back({start, s0});
}

void ContourBuffer::pushVertex(const Pose4& p, float s) {
    if (_v.size() >= _cap) _v.erase(_v.begin());
    _v.push_back({p, s});
}

float ContourBuffer::appendLine(const Pose4& end) {
    if (_v.empty()) { _v.push_back({end, 0.0f}); return totalLen(); }
    const Pose4 prev = _v.back().pose;
    float ds = xyChord(prev, end);
    if (ds < kMinSegLen) ds = uvChord(prev, end);
    if (ds < kMinSegLen) return totalLen();
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
        if (denom > 0.0f) segs = int(std::floor(std::fabs(0.5f * angular_travel * radius) / denom));
    }
    if (segs < 1) segs = 1;
    const float theta0 = std::atan2(startPose.y - cy, startPose.x - cx);
    for (int i = 1; i <= segs; ++i) {
        float frac = float(i) / float(segs);
        float theta = theta0 + angular_travel * frac;
        Pose4 p;
        p.x = cx + radius * std::cos(theta);
        p.y = cy + radius * std::sin(theta);
        p.u = startPose.u + (end.u - startPose.u) * frac;
        p.v = startPose.v + (end.v - startPose.v) * frac;
        if (i == segs) { p.x = end.x; p.y = end.y; }
        appendLine(p);
    }
    return totalLen();
}

Pose4 ContourBuffer::poseAt(float s) const {
    if (_v.empty())        return Pose4{};
    if (s <= _v.front().s) return _v.front().pose;
    if (s >= _v.back().s)  return _v.back().pose;
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
    size_t keep = 0;
    for (size_t i = 0; i < _v.size(); ++i) {
        if (_v[i].s <= s_floor) keep = i; else break;
    }
    if (keep > 0) _v.erase(_v.begin(), _v.begin() + keep);
}

}}  // namespace EDM::motion
