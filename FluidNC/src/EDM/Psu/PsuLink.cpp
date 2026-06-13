// FluidNC/src/EDM/Psu/PsuLink.cpp
#include "EDM/Psu/PsuLink.h"
#include <cstring>

namespace EDM { namespace psu {

void CanPsuLink::begin() {
    _bus.onReceive([this](const CanFrame& f) { onFrame(f); });
}

uint16_t CanPsuLink::setModeBounds(const SetModeBounds& m_in) {
    SetModeBounds m = m_in;
    m.seq = _seq++;
    if (_seq == 0) _seq = 1;
    _bus.send(encodeSetModeBounds(m));
    return m.seq;
}

void CanPsuLink::startCut()   { _bus.send(encodeControl(CTRL_START_CUT)); }
void CanPsuLink::stopCut()    { _bus.send(encodeControl(CTRL_STOP_CUT)); }
void CanPsuLink::clearFault() { _bus.send(encodeControl(CTRL_CLEAR_FAULT)); }

bool CanPsuLink::latestStats(StatsAgg& out) const {
    // Copy the snapshot out under the lock; the caller reads it after we unlock.
    EdmLockGuard g(_lock);
    if (!_stats_valid) return false;
    out = _stats;
    return true;
}

bool CanPsuLink::popEvent(Event& out) {
    EdmLockGuard g(_lock);
    if (_events.empty()) return false;
    out = _events.front();
    _events.pop_front();
    return true;
}

static void pushEvent(std::deque<Event>& q, size_t cap, const Event& e) {
    if (q.size() >= cap) q.pop_front();  // drop oldest on overflow
    q.push_back(e);
}

void CanPsuLink::onFrame(const CanFrame& f) {
    // Runs on the CAN-RX task. Guard the whole decode/store body: it writes
    // _stats/_stats_valid, _heartbeat_seen/_protocol_ok/_last_ack_status and
    // pushes _events, all of which the servo task reads concurrently. The
    // decode helpers are pure (no I/O), so holding the spinlock for them is
    // cheap and keeps the section short.
    EdmLockGuard g(_lock);
    switch (f.id) {
        case ID_STATS_AGG:
            if (decodeStatsAgg(f, _stats)) _stats_valid = true;
            break;
        case ID_ACK_MODE_BOUNDS: {
            AckModeBounds a;
            if (decodeAckModeBounds(f, a)) _last_ack_status = a.status;
            break;
        }
        case ID_PSU_STATUS: {
            PsuStatus st;
            if (decodePsuStatus(f, st)) {
                _heartbeat_seen = true;
                _protocol_ok = (st.protocol_version == kProtocolVersion);
            }
            break;
        }
        case ID_WIRE_BREAK: {
            Event e; e.kind = Event::WireBreak;
            if (decodeWireBreak(f, e.wire_break)) pushEvent(_events, kMaxEvents, e);
            break;
        }
        case ID_FAULT: {
            Event e; e.kind = Event::FaultEvt;
            if (decodeFault(f, e.fault)) pushEvent(_events, kMaxEvents, e);
            break;
        }
        case ID_ARC_BURST: {
            Event e; e.kind = Event::ArcBurstEvt;
            if (decodeArcBurst(f, e.arc_burst)) pushEvent(_events, kMaxEvents, e);
            break;
        }
        case ID_INFO: {
            Event e; e.kind = Event::Info;
            if (decodeInfo(f, e.info, sizeof(e.info))) pushEvent(_events, kMaxEvents, e);
            break;
        }
        default: break;
    }
}

}}  // namespace EDM::psu
