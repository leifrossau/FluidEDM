// FluidNC/src/EDM/Diel/SimDielLink.h
#pragma once
#include "EDM/Diel/IDielLink.h"
namespace EDM { namespace diel {
// In-firmware coolant-loop simulator (mock-first dev / host tests). No CAN.
class SimDielLink : public IDielLink {
public:
  void begin()          { _connected = true; }
  void setCutting(bool c){ _cutting = c; }   // EdmController/test sets this; pump follows it
  void tick(float dt);

  uint16_t setDiel(const SetDiel& s) override;
  bool latestStats(DielStats& out) const override;
  bool popEvent(DielEvent&) override { return false; }  // sim emits no events
  bool isConnected() const override        { return _connected; }
  bool protocolCompatible() const override { return _connected; }
  bool present() const override            { return _connected; }
private:
  bool _connected = false, _cutting = false;
  uint16_t _seq = 0;
  uint8_t  _pump_on = 0, _flush_level = 2, _level_pct = 88, _filter_pct = 96;
  float    _flush_mbar = 0, _flow_clpm = 0, _temp_dC = 215, _temp_set_dC = 220, _cond_uS = 6;
  uint32_t _window = 0;
};
}}  // namespace EDM::diel
