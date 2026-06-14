// FluidNC/src/EDM/Diel/CanDielLink.cpp
#include "EDM/Diel/CanDielLink.h"
namespace EDM { namespace diel {
using EDM::psu::EdmLockGuard;

void CanDielLink::begin() { _bus.onReceive([this](const CanFrame& f){ onFrame(f); }); }

uint16_t CanDielLink::setDiel(const SetDiel& in) {
  SetDiel s = in; s.seq = _seq++; if (_seq == 0) _seq = 1;
  _bus.send(encodeSetDiel(s));
  return s.seq;
}

bool CanDielLink::latestStats(DielStats& out) const {
  EdmLockGuard g(_lock);
  if (!_stats_valid) return false;
  out = _stats; return true;
}

bool CanDielLink::popEvent(DielEvent& out) {
  EdmLockGuard g(_lock);
  if (_events.empty()) return false;
  out = _events.front(); _events.pop_front(); return true;
}

void CanDielLink::onFrame(const CanFrame& f) {
  switch (f.id) {
    case ID_DIEL_STATS: { DielStats s; if (decodeDielStats(f, s)) { EdmLockGuard g(_lock); _stats = s; _stats_valid = true; } break; }
    case ID_ACK_DIEL:   { AckDiel a;   if (decodeAckDiel(f, a))   { EdmLockGuard g(_lock); _last_ack_status = a.status; } break; }
    case ID_DIEL_STATUS:{ DielStatus st; if (decodeDielStatus(f, st)) { EdmLockGuard g(_lock); _heartbeat_seen = true; _protocol_ok = (st.protocol_version == kProtocolVersion); } break; }
    case ID_DIEL_FAULT: { DielEvent e; e.kind = DielEvent::FaultEvt; if (decodeDielFault(f, e.fault)) { EdmLockGuard g(_lock); if (_events.size() >= kMaxEvents) _events.pop_front(); _events.push_back(e); } break; }
    default: break;
  }
}
}}  // namespace EDM::diel
