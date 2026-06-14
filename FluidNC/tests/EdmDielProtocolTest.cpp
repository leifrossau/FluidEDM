// FluidNC/tests/EdmDielProtocolTest.cpp
#include "gtest/gtest.h"
#include "EDM/Diel/DielProtocol.h"
#include "EDM/Psu/Endian.h"
using namespace EDM;
using namespace EDM::diel;

TEST(EdmDielProto, EncodeSetDielLayout){
    SetDiel s; s.seq=0x1234; s.pump_on=1; s.flush_level=2; s.temp_setpoint_dC=205; s.deioniser_enable=1; s.flags=0;
    CanFrame f=encodeSetDiel(s);
    EXPECT_EQ(f.id, ID_SET_DIEL); EXPECT_EQ(f.len, 9);
    EXPECT_EQ(le::get_u16(&f.data[0]), 0x1234);
    EXPECT_EQ(f.data[2], 1); EXPECT_EQ(f.data[3], 2);
    EXPECT_EQ(le::get_i16(&f.data[4]), 205); EXPECT_EQ(f.data[6], 1);
}
TEST(EdmDielProto, DecodeDielStatsRoundTrip){
    CanFrame f(ID_DIEL_STATS, 20); uint8_t* d=f.data;
    le::put_u32(d+0, 7); d[4]=1; d[5]=2; le::put_u16(d+6,2600); le::put_u16(d+8,650);
    le::put_i16(d+10,228); le::put_i16(d+12,220); le::put_u16(d+14,7); d[16]=88; d[17]=96; le::put_u16(d+18,0x0001);
    DielStats s; ASSERT_TRUE(decodeDielStats(f,s));
    EXPECT_EQ(s.window_id,7u); EXPECT_EQ(s.flush_mbar,2600); EXPECT_EQ(s.flow_clpm,650);
    EXPECT_EQ(s.temp_dC,228); EXPECT_EQ(s.temp_set_dC,220); EXPECT_EQ(s.conductivity_uS,7);
    EXPECT_EQ(s.level_pct,88); EXPECT_EQ(s.filter_pct,96); EXPECT_EQ(s.flags,1);
}
TEST(EdmDielProto, RejectsWrongIdAndShort){
    DielStats s; CanFrame bad(0x999,20); EXPECT_FALSE(decodeDielStats(bad,s));
    CanFrame sh(ID_DIEL_STATS,8); EXPECT_FALSE(decodeDielStats(sh,s));
}
TEST(EdmDielProto, DecodeAckRoundTrip){
    CanFrame f(ID_ACK_DIEL,7); le::put_u16(f.data+0,0x55); f.data[2]=1; f.data[3]=3; le::put_i16(f.data+4,210); f.data[6]=0;
    AckDiel a; ASSERT_TRUE(decodeAckDiel(f,a));
    EXPECT_EQ(a.seq,0x55); EXPECT_EQ(a.pump_on,1); EXPECT_EQ(a.flush_level,3); EXPECT_EQ(a.temp_setpoint_dC,210); EXPECT_EQ(a.status,0);
}
TEST(EdmDielProto, DecodeFault){
    CanFrame f(ID_DIEL_FAULT,8); f.data[0]=2; f.data[1]=3; f.data[2]=0xAA;
    DielFault flt; ASSERT_TRUE(decodeDielFault(f,flt));
    EXPECT_EQ(flt.fault_code,2); EXPECT_EQ(flt.severity,3); EXPECT_EQ(flt.detail[0],0xAA);
}
TEST(EdmDielProto, DecodeStatusVersion){
    CanFrame f(ID_DIEL_STATUS,9); f.data[0]=0; le::put_u16(f.data+1,0x0101); le::put_u16(f.data+3,kProtocolVersion); le::put_u32(f.data+5,3600);
    DielStatus st; ASSERT_TRUE(decodeDielStatus(f,st));
    EXPECT_EQ(st.protocol_version,kProtocolVersion); EXPECT_EQ(st.uptime_s,3600u);
}
