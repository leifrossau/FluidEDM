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
