// FluidNC/tests/EdmSimPsuLinkTest.cpp
#include "gtest/gtest.h"
#include "EDM/Psu/SimPsuLink.h"

using namespace EDM;
using namespace EDM::psu;

TEST(EdmSimPsuLink, StartsDisconnectedUntilBegun) {
    SimPsuLink sim;
    EXPECT_FALSE(sim.isConnected());
    sim.begin();
    EXPECT_TRUE(sim.isConnected());
    EXPECT_TRUE(sim.protocolCompatible());
}

TEST(EdmSimPsuLink, IdealGapYieldsMostlyNormal) {
    SimPsuLink sim;
    sim.begin();
    sim.setGap(sim.idealGapMm());
    sim.tick();
    StatsAgg s;
    ASSERT_TRUE(sim.latestStats(s));
    int total = s.n_normal + s.n_arc + s.n_short + s.n_open;
    ASSERT_GT(total, 0);
    EXPECT_GT(s.n_normal, total / 2);  // majority normal at ideal gap
}

TEST(EdmSimPsuLink, TooSmallGapYieldsShorts) {
    SimPsuLink sim;
    sim.begin();
    sim.setGap(0.0f);   // wire touching workpiece
    sim.tick();
    StatsAgg s;
    sim.latestStats(s);
    EXPECT_GT(s.n_short, s.n_normal);
}

TEST(EdmSimPsuLink, LargeGapYieldsOpens) {
    SimPsuLink sim;
    sim.begin();
    sim.setGap(sim.idealGapMm() * 5.0f);
    sim.tick();
    StatsAgg s;
    sim.latestStats(s);
    EXPECT_GT(s.n_open, s.n_normal);
}

TEST(EdmSimPsuLink, WindowIdIncrementsEachTick) {
    SimPsuLink sim;
    sim.begin();
    sim.tick();
    StatsAgg a; sim.latestStats(a);
    sim.tick();
    StatsAgg b; sim.latestStats(b);
    EXPECT_EQ(b.window_id, a.window_id + 1);
}

TEST(EdmSimPsuLink, ProportionalRegionInterpolates) {
    SimPsuLink sim; sim.begin();
    sim.setGap(sim.idealGapMm() * 1.5f);
    sim.tick();
    StatsAgg s; ASSERT_TRUE(sim.latestStats(s));
    int total = s.n_normal + s.n_arc + s.n_short + s.n_open;
    EXPECT_EQ(total, 100);
    EXPECT_GT(s.n_open, 6);
    EXPECT_LT(s.n_open, 80);
}

TEST(EdmSimPsuLink, VelocityCouplingClosesGap) {
    SimPsuLink sim; sim.begin();
    sim.setVelocityCoupling(0.00002f);
    sim.setGap(sim.idealGapMm() * 3.0f);
    float start = sim.gapMm();
    for (int i = 0; i < 50; ++i) { sim.applyCommandedVelocity(+67); sim.tick(); }
    EXPECT_LT(sim.gapMm(), start);
    EXPECT_GE(sim.gapMm(), 0.0f);
}

TEST(EdmSimPsuLink, RetractOpensGapAndClampsAtZero) {
    SimPsuLink sim; sim.begin();
    sim.setVelocityCoupling(0.00002f);
    sim.setGap(0.0f);
    sim.applyCommandedVelocity(-100); sim.tick();
    EXPECT_GT(sim.gapMm(), 0.0f);
}

TEST(EdmSimPsuLink, PushEventThenPopReturnsIt) {
    SimPsuLink sim; sim.begin();
    Event in; in.kind = Event::WireBreak; in.wire_break.severity = 3;
    sim.pushEvent(in);
    Event out;
    ASSERT_TRUE(sim.popEvent(out));
    EXPECT_EQ(out.kind, Event::WireBreak);
    EXPECT_EQ(out.wire_break.severity, 3);
    EXPECT_FALSE(sim.popEvent(out));
}
