// FluidNC/tests/EdmDielLinkTest.cpp
#include "gtest/gtest.h"
#include "EDM/Can/CanBus.h"
#include "EDM/Diel/CanDielLink.h"
#include "EDM/Diel/SimDielLink.h"
#include "EDM/Psu/Endian.h"
using namespace EDM;
using namespace EDM::diel;

TEST(EdmDielLink, SetDielSendsFrameAndSeq){
  test_support::FakeCanBus bus; CanDielLink l(bus); l.begin();
  SetDiel s; s.flush_level=3; uint16_t seq=l.setDiel(s);
  ASSERT_EQ(bus.sent.size(),1u); EXPECT_EQ(bus.sent[0].id, ID_SET_DIEL);
  EXPECT_EQ(le::get_u16(&bus.sent[0].data[0]), seq);
}
TEST(EdmDielLink, StatsAndHeartbeat){
  test_support::FakeCanBus bus; CanDielLink l(bus); l.begin();
  DielStats s; EXPECT_FALSE(l.latestStats(s));
  CanFrame f(ID_DIEL_STATS,20); le::put_u16(f.data+8,650); bus.inject(f);
  ASSERT_TRUE(l.latestStats(s)); EXPECT_EQ(s.flow_clpm,650);
  EXPECT_FALSE(l.present());
  CanFrame h(ID_DIEL_STATUS,9); le::put_u16(h.data+3,kProtocolVersion); bus.inject(h);
  EXPECT_TRUE(l.present()); EXPECT_TRUE(l.protocolCompatible());
}
TEST(EdmDielLink, FaultBecomesEvent){
  test_support::FakeCanBus bus; CanDielLink l(bus); l.begin();
  CanFrame f(ID_DIEL_FAULT,8); f.data[0]=5; f.data[1]=2; bus.inject(f);
  DielEvent e; ASSERT_TRUE(l.popEvent(e)); EXPECT_EQ(e.fault.fault_code,5); EXPECT_FALSE(l.popEvent(e));
}
TEST(EdmDielSim, PumpFollowsCutAndRegulates){
  SimDielLink sim; sim.begin(); sim.setCutting(true);
  SetDiel s; s.flush_level=2; sim.setDiel(s);
  for(int i=0;i<200;++i) sim.tick(0.012f);
  DielStats st; ASSERT_TRUE(sim.latestStats(st));
  EXPECT_EQ(st.pump_on,1); EXPECT_GT(st.flow_clpm,0); EXPECT_GT(st.flush_mbar,0);
}
TEST(EdmDielSim, FlushOffLowFlow){
  SimDielLink sim; sim.begin(); sim.setCutting(true);
  SetDiel s; s.flush_level=0; sim.setDiel(s);
  for(int i=0;i<200;++i) sim.tick(0.012f);
  DielStats st; sim.latestStats(st); EXPECT_LT(st.flow_clpm,100);
}
TEST(EdmDielSim, NotCuttingPumpOff){
  SimDielLink sim; sim.begin(); sim.setCutting(false);
  for(int i=0;i<100;++i) sim.tick(0.012f);
  DielStats st; sim.latestStats(st); EXPECT_EQ(st.pump_on,0); EXPECT_LT(st.flow_clpm,50);
}
