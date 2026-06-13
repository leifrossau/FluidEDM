// FluidNC/tests/EdmPsuLinkTest.cpp
#include "gtest/gtest.h"
#include "EDM/Can/CanBus.h"
#include "EDM/Psu/Endian.h"
#include "EDM/Psu/PsuLink.h"

using namespace EDM;
using namespace EDM::psu;

TEST(EdmPsuLink, SetModeBoundsSendsFrameAndAssignsSeq) {
    test_support::FakeCanBus bus;
    CanPsuLink link(bus);
    link.begin();

    SetModeBounds m; m.mode_id = 1; m.peak_I_setpoint_dA = 120;
    uint16_t seq = link.setModeBounds(m);

    ASSERT_EQ(bus.sent.size(), 1u);
    EXPECT_EQ(bus.sent[0].id, ID_SET_MODE_BOUNDS);
    EXPECT_EQ(le::get_u16(&bus.sent[0].data[1]), seq);  // seq stamped into frame
}

TEST(EdmPsuLink, StatsAggUpdatesLatest) {
    test_support::FakeCanBus bus;
    CanPsuLink link(bus);
    link.begin();

    StatsAgg s; // empty snapshot before any frame
    EXPECT_FALSE(link.latestStats(s));

    CanFrame f(ID_STATS_AGG, 38);
    le::put_u16(f.data + 4, 88);  // n_normal
    bus.inject(f);

    ASSERT_TRUE(link.latestStats(s));
    EXPECT_EQ(s.n_normal, 88);
}

TEST(EdmPsuLink, WireBreakBecomesQueuedEvent) {
    test_support::FakeCanBus bus;
    CanPsuLink link(bus);
    link.begin();

    CanFrame f(ID_WIRE_BREAK, 14);
    f.data[0] = 2;  // severity
    bus.inject(f);

    Event e;
    ASSERT_TRUE(link.popEvent(e));
    EXPECT_EQ(e.kind, Event::WireBreak);
    EXPECT_EQ(e.wire_break.severity, 2);
    EXPECT_FALSE(link.popEvent(e));  // queue drained
}

TEST(EdmPsuLink, HeartbeatDrivesConnectedAndProtocol) {
    test_support::FakeCanBus bus;
    CanPsuLink link(bus);
    link.begin();
    EXPECT_FALSE(link.isConnected());

    CanFrame f(ID_PSU_STATUS, 13);
    le::put_u16(f.data + 5, kProtocolVersion);  // matching version
    bus.inject(f);

    EXPECT_TRUE(link.isConnected());
    EXPECT_TRUE(link.protocolCompatible());
}

TEST(EdmPsuLink, MismatchedProtocolFlagged) {
    test_support::FakeCanBus bus;
    CanPsuLink link(bus);
    link.begin();
    CanFrame f(ID_PSU_STATUS, 13);
    le::put_u16(f.data + 5, kProtocolVersion + 1);
    bus.inject(f);
    EXPECT_TRUE(link.isConnected());
    EXPECT_FALSE(link.protocolCompatible());
}
