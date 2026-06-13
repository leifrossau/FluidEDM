// FluidNC/src/EDM/Psu/PsuLink.h
#pragma once
#include <deque>
#include "EDM/Can/CanBus.h"
#include "EDM/Psu/IPsuLink.h"

namespace EDM { namespace psu {

// Session layer over a CanBus: stamps sequence numbers, decodes inbound
// frames into a telemetry snapshot + event queue, tracks heartbeat health.
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

    std::deque<Event> _events;
    static constexpr size_t kMaxEvents = 32;

    bool     _heartbeat_seen = false;
    bool     _protocol_ok = false;
    uint16_t _last_ack_status = 0;
};

}}  // namespace EDM::psu
