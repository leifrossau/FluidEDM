// FluidNC/tests/EdmControllerTest.cpp
#include "gtest/gtest.h"
#include "EDM/Control/EdmController.h"
#include "EDM/Psu/SimPsuLink.h"

using namespace EDM;
using namespace EDM::servo;
using namespace EDM::psu;

static ModeTable makeModes() {
    ModeTable t;
    t.rough  = { 1, 200, 800, 1200, 200, 260, 0, 0x000D, 25000, 8000, 30, 10000 };
    t.finish = { 2, 400, 200,  600,  80, 120, 0, 0x000D, 25000, 8000, 20,  6000 };
    t.ignite = { 3, 100, 300, 3000,  40,  80, 0, 0x0008, 25000, 8000, 50, 16000 };
    return t;
}

TEST(EdmController, ArmSendsBoundsWaitsAckStartsCut) {
    SimPsuLink sim; sim.begin();
    ServoConfig cfg; ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);
    ctl.requestCut(900);
    ctl.tick(0);
    EXPECT_EQ(ctl.state(), EdmState::Armed);
    ctl.tick(1);
    EXPECT_EQ(ctl.state(), EdmState::TouchOff);
}

TEST(EdmController, RepeatedSameModeKeepsMode) {
    SimPsuLink sim; sim.begin();
    ServoConfig cfg; ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);
    ctl.requestCut(900); ctl.tick(0);
    uint8_t m1 = ctl.snapshot().active_mode_id;
    ctl.requestCut(950);
    ctl.tick(1);
    EXPECT_EQ(ctl.snapshot().active_mode_id, m1);  // still Rough
}

TEST(EdmController, ClosedLoopReachesIdealGap) {
    SimPsuLink sim; sim.begin();
    sim.setVelocityCoupling(0.00002f);
    sim.setGap(sim.idealGapMm() * 3.0f);
    ServoConfig cfg; cfg.Ki = 2.0f;
    ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);

    ctl.requestCut(900);
    uint32_t t = 0;
    for (int i = 0; i < 5; ++i) { sim.tick(); ctl.tick(t++); }
    // 4000 closed-loop iterations (now_ms ~= t). The loop converges to ideal and
    // holds in-band at a healthy gap. With the healthy-hold fix the ServoStall
    // watchdog must NOT fire even though now_ms passes the 2000 ms stall window,
    // because a balanced hold at a healthy gap resets the stall timer.
    for (int i = 0; i < 4000; ++i) {
        EdmReport r = ctl.snapshot();
        sim.applyCommandedVelocity(r.v_cmd_um_s);
        sim.tick();
        ctl.tick(t++);
    }
    EXPECT_NE(ctl.state(), EdmState::StallFault);
    EXPECT_NE(ctl.state(), EdmState::Fault);
    StatsAgg s; sim.latestStats(s);
    EXPECT_LT(s.n_short, 20);
    EXPECT_NEAR(sim.gapMm(), sim.idealGapMm(), sim.idealGapMm() * 1.0f);
}

TEST(EdmController, WireBreakSev3RelievesThenReApproaches) {
    SimPsuLink sim; sim.begin();
    sim.setVelocityCoupling(0.00002f);
    sim.setGap(sim.idealGapMm());
    ServoConfig cfg; cfg.Ki = 2.0f; ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);
    ctl.requestCut(900);
    uint32_t t = 0;
    for (int i = 0; i < 10; ++i) { sim.tick(); ctl.tick(t++); }
    ASSERT_TRUE(ctl.state() == EdmState::Cutting || ctl.state() == EdmState::Hold);

    Event ev; ev.kind = Event::WireBreak; ev.wire_break.severity = 3;
    sim.pushEvent(ev);
    sim.tick(); ctl.tick(t++);
    EXPECT_EQ(ctl.state(), EdmState::BreakRelief);
    EXPECT_LT(ctl.snapshot().v_cmd_um_s, 0);
    EXPECT_EQ(ctl.snapshot().active_mode_id, 3);

    // After break_relief (50 ms) the controller returns to TouchOff to re-find
    // the gap. Because the sim gap is parked at ideal, re-contact (open<0.5 for
    // 2 windows) happens immediately, so TouchOff is transient and the loop ends
    // back in Cutting/Hold. Assert the re-approach (TouchOff) was actually
    // entered during the loop, then that the controller re-establishes the cut.
    bool reentered_touchoff = false;
    for (int i = 0; i < 60; ++i) {
        sim.tick(); ctl.tick(t++);
        if (ctl.state() == EdmState::TouchOff) reentered_touchoff = true;
    }
    EXPECT_TRUE(reentered_touchoff);
    EXPECT_TRUE(ctl.state() == EdmState::TouchOff ||
                ctl.state() == EdmState::Cutting ||
                ctl.state() == EdmState::Hold);
}

TEST(EdmController, TelemetryStaleHoldsThenFaults) {
    SimPsuLink sim; sim.begin();
    sim.setGap(sim.idealGapMm());
    ServoConfig cfg; ModeTable modes = makeModes();
    ControllerTimers tm;
    EdmController ctl(sim, cfg, modes, tm);
    ctl.requestCut(900);
    uint32_t t = 0;
    for (int i = 0; i < 6; ++i) { sim.tick(); ctl.tick(t++); }
    uint32_t base = t;
    ctl.tick(base + 10);     // within tele_hold: HOLD, not fault
    EXPECT_EQ(ctl.snapshot().v_cmd_um_s, 0);
    EXPECT_NE(ctl.state(), EdmState::Fault);
    ctl.tick(base + 250);    // beyond tele_fault: HeartbeatLost
    EXPECT_EQ(ctl.state(), EdmState::Fault);
    EXPECT_EQ(ctl.fault(), FaultReason::HeartbeatLost);
}

TEST(EdmController, LostGapReentersTouchOff) {
    SimPsuLink sim; sim.begin();
    sim.setGap(sim.idealGapMm());
    ServoConfig cfg; cfg.Ki = 2.0f; ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);
    ctl.requestCut(900);
    uint32_t t = 0;
    for (int i = 0; i < 10; ++i) { sim.tick(); ctl.tick(t++); }
    sim.setVelocityCoupling(0.0f);
    sim.setGap(sim.idealGapMm() * 5.0f);     // r=5 -> open~0.97 > 0.95
    for (int i = 0; i < 8; ++i) { sim.tick(); ctl.tick(t++); }
    EXPECT_EQ(ctl.state(), EdmState::TouchOff);
}

TEST(EdmController, PersistentShortStallFaults) {
    SimPsuLink sim; sim.begin();
    sim.setVelocityCoupling(0.0f);   // retract cannot reopen the gap
    sim.setGap(0.0f);                // wire touching workpiece -> persistent short
    ServoConfig cfg; cfg.Ki = 2.0f; ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);
    ctl.requestCut(900);
    uint32_t t = 0;
    for (int i = 0; i < 5; ++i) { sim.tick(); ctl.tick(t++); }       // arm + touch-off + cutting
    for (int i = 0; i < 2200; ++i) { sim.tick(); ctl.tick(t++); }    // persistent short > stall(2000ms)
    EXPECT_EQ(ctl.state(), EdmState::StallFault);
    EXPECT_EQ(ctl.fault(), FaultReason::ServoStall);
}

TEST(EdmController, ModeRestoresAfterWireBreakClears) {
    SimPsuLink sim; sim.begin();
    sim.setGap(sim.idealGapMm());
    ServoConfig cfg; cfg.Ki = 2.0f; ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);
    ctl.requestCut(900);                                   // Rough (mode_id 1)
    uint32_t t = 0;
    for (int i = 0; i < 10; ++i) { sim.tick(); ctl.tick(t++); }
    ASSERT_EQ(ctl.snapshot().active_mode_id, 1);

    Event ev; ev.kind = Event::WireBreak; ev.wire_break.severity = 2;
    sim.pushEvent(ev);
    sim.tick(); ctl.tick(t++);
    EXPECT_EQ(ctl.snapshot().active_mode_id, 2);           // sev2 lowered Rough -> Finish

    // run past the wire-break clear window (default clear=500ms) at a healthy gap
    for (int i = 0; i < 520; ++i) { sim.tick(); ctl.tick(t++); }
    EXPECT_EQ(ctl.snapshot().active_mode_id, 1);           // restored to Rough
    EXPECT_NE(ctl.state(), EdmState::Fault);
}
