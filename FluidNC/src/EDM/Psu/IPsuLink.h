// FluidNC/src/EDM/Psu/IPsuLink.h
#pragma once
#include <cstdint>
#include "EDM/Psu/Protocol.h"

namespace EDM { namespace psu {

// One queued asynchronous event from the PSU.
struct Event {
    enum Kind : uint8_t { WireBreak, FaultEvt, ArcBurstEvt, Info } kind;
    WireBreakImminent wire_break;
    Fault             fault;
    ArcBurst          arc_burst;
    char              info[61];
};

// Interface the EdmController depends on. Implemented by CanPsuLink (real)
// and SimPsuLink (in-firmware simulator).
class IPsuLink {
public:
    virtual ~IPsuLink() = default;

    // Commands.
    virtual uint16_t setModeBounds(const SetModeBounds& m) = 0;  // returns seq
    virtual void     startCut()  = 0;
    virtual void     stopCut()   = 0;
    virtual void     clearFault() = 0;

    // Latest 1 kHz telemetry snapshot. Returns false if none received yet.
    virtual bool latestStats(StatsAgg& out) const = 0;

    // Pops one queued event; returns false if the queue is empty.
    virtual bool popEvent(Event& out) = 0;

    // Connection / protocol health (driven by PSU_STATUS heartbeat).
    virtual bool     isConnected() const = 0;        // heartbeat fresh
    virtual bool     protocolCompatible() const = 0; // version matches
    virtual uint16_t lastAckStatus() const = 0;      // 0 = ok / none
};

}}  // namespace EDM::psu
