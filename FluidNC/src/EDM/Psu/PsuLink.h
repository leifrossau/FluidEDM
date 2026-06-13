// FluidNC/src/EDM/Psu/PsuLink.h
#pragma once
#include <deque>
#include "EDM/Can/CanBus.h"
#include "EDM/Psu/IPsuLink.h"
#include "EDM/Psu/EdmLock.h"

namespace EDM { namespace psu {

// Session layer over a CanBus: stamps sequence numbers, decodes inbound
// frames into a telemetry snapshot + event queue, tracks heartbeat health.
// THREAD-SAFETY (Phase P4 — RESOLVED): On target, onFrame() runs on the CAN-RX
// task (one core) while latestStats()/popEvent()/isConnected()/
// protocolCompatible()/lastAckStatus() are read from the 1 kHz edmServoTask
// (other core). The shared members (_stats/_stats_valid, _events,
// _heartbeat_seen/_protocol_ok/_last_ack_status) are now guarded by the _lock
// EdmLock: a FreeRTOS portMUX spinlock on ESP32, a no-op on the single-threaded
// native host test build (so the host googletests are unchanged). Critical
// sections are kept minimal — copy out under the lock, return outside it.
class CanPsuLink : public IPsuLink {
public:
    explicit CanPsuLink(CanBus& bus) : _bus(bus) {}

    void begin();  // registers RX handler

    uint16_t setModeBounds(const SetModeBounds& m) override;
    void     startCut()   override;
    void     stopCut()    override;
    void     clearFault() override;

    bool latestStats(StatsAgg& out) const override;
    bool popEvent(Event& out) override;
    bool isConnected() const override         { EdmLockGuard g(_lock); return _heartbeat_seen; }
    bool protocolCompatible() const override  { EdmLockGuard g(_lock); return _protocol_ok; }
    uint16_t lastAckStatus() const override   { EdmLockGuard g(_lock); return _last_ack_status; }

    // Called by the CAN-RX task each cycle (target). Tests inject directly.
    void onFrame(const CanFrame& f);

private:
    CanBus&  _bus;
    uint16_t _seq = 1;

    // Guards all cross-core shared state below. mutable so the const readers
    // (latestStats/isConnected/protocolCompatible/lastAckStatus) can lock.
    mutable EdmLock _lock;

    StatsAgg _stats{};
    bool     _stats_valid = false;

    std::deque<Event> _events;  // TODO(P4): replace with fixed std::array ring buffer (no heap in RX path); access is now spinlock-guarded
    static constexpr size_t kMaxEvents = 32;

    bool     _heartbeat_seen = false;  // TODO(P2): track last-heartbeat timestamp so isConnected() expires when PSU goes silent
    bool     _protocol_ok = false;
    uint16_t _last_ack_status = 0;
};

}}  // namespace EDM::psu
