// FluidNC/src/EDM/Diel/SimDielLink.cpp
#include "EDM/Diel/SimDielLink.h"
namespace EDM { namespace diel {
namespace {
inline float lerpf(float a, float b, float t){ return a + (b - a) * t; }
inline float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }
}
uint16_t SimDielLink::setDiel(const SetDiel& s) {
  _pump_on = s.pump_on; _flush_level = s.flush_level > 3 ? 3 : s.flush_level; _temp_set_dC = s.temp_setpoint_dC;
  return ++_seq;
}
void SimDielLink::tick(float /*dt*/) {
  _window++;
  static const float barT[4]  = {0, 1400, 2600, 4200};   // mbar
  static const float flowT[4] = {40, 260, 460, 680};      // cL/min
  if (_cutting) {
    _pump_on = 1;
    _flush_mbar = lerpf(_flush_mbar, barT[_flush_level], 0.2f);
    _flow_clpm  = lerpf(_flow_clpm,  flowT[_flush_level], 0.2f);
    _temp_dC    = clampf(lerpf(_temp_dC, _temp_set_dC + 6.0f, 0.03f), 160, 340);
    _cond_uS    = clampf(_cond_uS + 0.04f, 1, 400);
    if (_window % 2000 == 0 && _level_pct > 0) _level_pct--;
  } else {
    _pump_on = 0;
    _flush_mbar = lerpf(_flush_mbar, 0, 0.3f);
    _flow_clpm  = lerpf(_flow_clpm,  0, 0.3f);
    _temp_dC    = lerpf(_temp_dC, _temp_set_dC, 0.02f);
  }
}
bool SimDielLink::latestStats(DielStats& o) const {
  if (!_connected) return false;
  o = DielStats{};
  o.window_id       = _window;
  o.pump_on         = _pump_on; o.flush_level = _flush_level;
  o.flush_mbar      = uint16_t(_flush_mbar + 0.5f);
  o.flow_clpm       = uint16_t(_flow_clpm + 0.5f);
  o.temp_dC         = int16_t(_temp_dC);
  o.temp_set_dC     = int16_t(_temp_set_dC);
  o.conductivity_uS = uint16_t(_cond_uS + 0.5f);
  o.level_pct       = _level_pct; o.filter_pct = _filter_pct;
  o.flags           = (o.flow_clpm < 100 ? 0x0002 : 0) | (o.level_pct < 15 ? 0x0004 : 0);
  return true;
}
}}  // namespace EDM::diel
