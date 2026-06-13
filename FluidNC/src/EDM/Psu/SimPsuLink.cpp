// FluidNC/src/EDM/Psu/SimPsuLink.cpp
#include "EDM/Psu/SimPsuLink.h"

namespace EDM { namespace psu {

namespace {
inline float lerp(float a, float b, float t) { return a + (b - a) * t; }
}

void SimPsuLink::pushEvent(const Event& e) {
    if (_evt_count >= kEvtCap) { _evt_head = (_evt_head + 1) % kEvtCap; _evt_count--; }
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
    _gap_mm -= _coupling * float(_last_v_um_s);
    if (_gap_mm < 0.0f) _gap_mm = 0.0f;

    float r = (_ideal_gap_mm > 0.0f) ? (_gap_mm / _ideal_gap_mm) : 0.0f;

    float fs, fa, fn, fo;
    if (r <= 1.0f) {
        float t = r;
        fs = lerp(0.90f, 0.06f, t);
        fa = lerp(0.05f, 0.08f, t);
        fn = lerp(0.05f, 0.80f, t);
        fo = lerp(0.00f, 0.06f, t);
    } else {
        float t = (r - 1.0f) / 2.0f; if (t > 1.0f) t = 1.0f;
        fs = lerp(0.06f, 0.00f, t);
        fa = lerp(0.08f, 0.05f, t);
        fn = lerp(0.80f, 0.15f, t);
        fo = lerp(0.06f, 0.80f, t);
    }

    int n_short  = int(fs * 100.0f + 0.5f);
    int n_arc    = int(fa * 100.0f + 0.5f);
    int n_open   = int(fo * 100.0f + 0.5f);
    int n_normal = 100 - n_short - n_arc - n_open;
    if (n_normal < 0) n_normal = 0;

    _stats = StatsAgg{};
    _stats.window_id    = _window++;
    _stats.n_normal     = uint16_t(n_normal);
    _stats.n_arc        = uint16_t(n_arc);
    _stats.n_short      = uint16_t(n_short);
    _stats.n_open       = uint16_t(n_open);
    _stats.ignition_delay_mean_ns   = uint16_t(2000.0f + 8000.0f * (r > 3.0f ? 3.0f : r));
    _stats.ignition_delay_stddev_ns = 200;
    _stats.state        = _cutting ? 0 : 1;
    _stats.mode_id_active = _mode.mode_id;
    _stats.dc_link_V_dV  = 800;
    _stats_valid = true;
}

bool SimPsuLink::latestStats(StatsAgg& out) const {
    if (!_stats_valid) return false;
    out = _stats;
    return true;
}

}}  // namespace EDM::psu
