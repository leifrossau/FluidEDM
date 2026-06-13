// FluidNC/src/EDM/Psu/SimPsuLink.h
#pragma once
#include "EDM/Psu/IPsuLink.h"

namespace EDM { namespace psu {

// In-firmware PSU simulator. No CAN. Models the spark gap so the gap servo
// and EdmController can be developed and tuned with zero hardware.
class SimPsuLink : public IPsuLink {
public:
    void  begin()            { _connected = true; }
    void  setGap(float mm)   { _gap_mm = mm; }
    float idealGapMm() const { return _ideal_gap_mm; }

    // Advances the simulation one telemetry window and refreshes latestStats().
    void tick();

    // IPsuLink
    uint16_t setModeBounds(const SetModeBounds& m) override { _mode = m; return ++_seq; }
    void     startCut()   override { _cutting = true; }
    void     stopCut()    override { _cutting = false; }
    void     clearFault() override {}

    bool latestStats(StatsAgg& out) const override;
    bool popEvent(Event&) override { return false; }   // sim emits no events in P0
    bool isConnected() const override        { return _connected; }
    bool protocolCompatible() const override { return _connected; }
    uint16_t lastAckStatus() const override  { return 0; }

private:
    bool  _connected = false;
    bool  _cutting   = false;
    float _gap_mm    = 0.02f;
    float _ideal_gap_mm = 0.02f;   // 20 µm nominal working gap
    uint16_t _seq = 0;
    uint32_t _window = 0;
    StatsAgg _stats{};
    bool     _stats_valid = false;
    SetModeBounds _mode{};
};

}}  // namespace EDM::psu
