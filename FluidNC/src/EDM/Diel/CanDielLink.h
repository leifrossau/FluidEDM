// FluidNC/src/EDM/Diel/CanDielLink.h
#pragma once
#include <deque>
#include "EDM/Can/CanBus.h"
#include "EDM/Diel/IDielLink.h"
#include "EDM/Psu/EdmLock.h"
namespace EDM { namespace diel {
// Session layer over a CanBus for the dielectric module (IDs 0x7xx). Mirrors CanPsuLink.
class CanDielLink : public IDielLink {
public:
  explicit CanDielLink(CanBus& bus) : _bus(bus) {}
  void begin();                                        // registers RX handler
  uint16_t setDiel(const SetDiel& s) override;
  bool latestStats(DielStats& out) const override;
  bool popEvent(DielEvent& out) override;
  // TODO(P2): like CanPsuLink, _heartbeat_seen latches and never expires. Track the
  // last-status timestamp so isConnected()/present() go false when the module goes
  // silent; until then the EdmController mid-cut interlock relies on flow/level loss
  // (dielReadyToCut) to catch a dead module rather than on present().
  bool isConnected() const override        { return _heartbeat_seen; }
  bool protocolCompatible() const override { return _protocol_ok; }
  bool present() const override            { return _heartbeat_seen; }
  void onFrame(const CanFrame& f);                     // ignores non-0x7xx IDs
private:
  CanBus&  _bus;
  uint16_t _seq = 1;
  DielStats _stats{}; bool _stats_valid = false;
  std::deque<DielEvent> _events; static constexpr size_t kMaxEvents = 16;
  bool _heartbeat_seen = false, _protocol_ok = false;
  uint8_t _last_ack_status = 0;
  mutable EDM::psu::EdmLock _lock;
};
}}  // namespace EDM::diel
