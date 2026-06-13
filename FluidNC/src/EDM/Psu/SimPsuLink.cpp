// FluidNC/src/EDM/Psu/SimPsuLink.cpp
#include "EDM/Psu/SimPsuLink.h"

namespace EDM { namespace psu {

void SimPsuLink::tick() {
    // Map gap width to classifier counts over a 100-pulse window.
    // ratio r = gap / ideal: r<<1 -> shorts; r~1 -> normal; r>>1 -> opens.
    float r = (_ideal_gap_mm > 0.0f) ? (_gap_mm / _ideal_gap_mm) : 0.0f;

    int shorts = 0, opens = 0, normals = 0, arcs = 0;
    if (r < 0.5f) {
        shorts  = 70; arcs = 15; normals = 10; opens = 5;     // too tight
    } else if (r > 2.0f) {
        opens   = 70; normals = 20; arcs = 5; shorts = 5;     // too wide
    } else {
        normals = 80; arcs = 8; shorts = 6; opens = 6;        // healthy band
    }

    _stats = StatsAgg{};
    _stats.window_id   = _window++;
    _stats.n_normal    = uint16_t(normals);
    _stats.n_arc       = uint16_t(arcs);
    _stats.n_short     = uint16_t(shorts);
    _stats.n_open      = uint16_t(opens);
    _stats.state       = _cutting ? 0 : 1;        // running / paused
    _stats.mode_id_active = _mode.mode_id;
    _stats.dc_link_V_dV = 800;                    // 80.0 V
    _stats_valid = true;
}

bool SimPsuLink::latestStats(StatsAgg& out) const {
    if (!_stats_valid) return false;
    out = _stats;
    return true;
}

}}  // namespace EDM::psu
