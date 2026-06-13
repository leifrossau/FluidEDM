// FluidNC/src/EDM/Psu/PsuLink.h
#pragma once
#include <deque>
#include "EDM/Can/CanBus.h"
#include "EDM/Psu/IPsuLink.h"

namespace EDM { namespace psu {

// Session layer over a CanBus: stamps sequence numbers, decodes inbound
// frames into a telemetry snapshot + event queue, tracks heartbeat health.
// THREAD-SAFETY (Phase P4): On target, onFrame() runs on the CAN-RX task
// (one core) while latestStats()/popEvent()/isConnected() are read from the
// 1 kHz edmServoTask (other core). The members below (_stats/_stats_valid,
// _events, _heartbeat_seen/_protocol_ok/_last_ack_status) are shared cross-core
// with NO synchronization yet — safe for P0's single-threaded host tests only.
// P4 MUST add a portMUX_TYPE spinlock (or lock-free single-producer/consumer
// structures) before these are accessed from two tasks. See plan P4.
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
    bool isConnected() const override         { return _heartbeat_seen; }
    bool protocolCompatible() const override  { return _protocol_ok; }
    uint16_t lastAckStatus() const override   { return _last_ack_status; }

    // Called by the CAN-RX task each cycle (target). Tests inject directly.
    void onFrame(const CanFrame& f);

private:
    CanBus&  _bus;
    uint16_t _seq = 1;

    StatsAgg _stats{};
    bool     _stats_valid = false;

    std::deque<Event> _events;  // TODO(P4): replace with fixed std::array ring buffer (no heap in RX path) + spinlock
    static constexpr size_t kMaxEvents = 32;

    bool     _heartbeat_seen = false;  // TODO(P2): track last-heartbeat timestamp so isConnected() expires when PSU goes silent
    bool     _protocol_ok = false;
    uint16_t _last_ack_status = 0;
};

}}  // namespace EDM::psu
