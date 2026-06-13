// FluidNC/src/EDM/Psu/SimPsuLink.h
#pragma once
#include "EDM/Psu/IPsuLink.h"

namespace EDM { namespace psu {

class SimPsuLink : public IPsuLink {
public:
    void  begin()            { _connected = true; }
    void  setGap(float mm)   { _gap_mm = mm < 0.0f ? 0.0f : mm; }
    float gapMm() const      { return _gap_mm; }
    float idealGapMm() const { return _ideal_gap_mm; }

    void setVelocityCoupling(float mm_per_um_s_per_tick) { _coupling = mm_per_um_s_per_tick; }
    void applyCommandedVelocity(int32_t v_s_um_s)        { _last_v_um_s = v_s_um_s; }
    void pushEvent(const Event& e);

    void tick();

    uint16_t setModeBounds(const SetModeBounds& m) override { _mode = m; return ++_seq; }
    void     startCut()   override { _cutting = true; }
    void     stopCut()    override { _cutting = false; }
    void     clearFault() override {}

    bool latestStats(StatsAgg& out) const override;
    bool popEvent(Event& out) override;
    bool isConnected() const override        { return _connected; }
    bool protocolCompatible() const override { return _connected; }
    uint16_t lastAckStatus() const override  { return 0; }

private:
    bool  _connected = false;
    bool  _cutting   = false;
    float _gap_mm    = 0.02f;
    float _ideal_gap_mm = 0.02f;
    float _coupling  = 0.0f;
    int32_t _last_v_um_s = 0;
    uint16_t _seq = 0;
    uint32_t _window = 0;
    StatsAgg _stats{};
    bool     _stats_valid = false;
    SetModeBounds _mode{};

    static constexpr int kEvtCap = 8;
    Event _evts[kEvtCap];
    int   _evt_head = 0, _evt_count = 0;
};

}}  // namespace EDM::psu
