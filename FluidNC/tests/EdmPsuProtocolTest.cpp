// FluidNC/tests/EdmPsuProtocolTest.cpp
#include "gtest/gtest.h"
#include "EDM/Psu/Protocol.h"
#include "EDM/Psu/Endian.h"

using namespace EDM;
using namespace EDM::psu;

TEST(EdmPsuProto, EncodeSetModeBoundsLayout) {
    SetModeBounds m;
    m.mode_id = 1; m.seq = 0x1234;
    m.freq_max_kHz = 200; m.on_time_max_ns = 800; m.off_time_min_ns = 1200;
    m.peak_I_setpoint_dA = 120; m.peak_I_limit_hw_dA = 200;
    m.polarity = 1; m.flags = 0x000B;
    m.gap_V_arc_mV = 4000; m.gap_V_short_mV = 1000; m.ignition_timeout_us = 50;

    CanFrame f = encodeSetModeBounds(m);
    EXPECT_EQ(f.id, ID_SET_MODE_BOUNDS);
    EXPECT_TRUE(f.fd);
    // byte 0 = mode_id, bytes 1..2 = seq (LE)
    EXPECT_EQ(f.data[0], 1);
    EXPECT_EQ(le::get_u16(&f.data[1]), 0x1234);
    EXPECT_EQ(le::get_u16(&f.data[3]), 200);    // freq_max_kHz
    // polarity sits after the 6 u16 numeric fields: offset 3 + 6*2 = 15
    EXPECT_EQ(f.data[15], 1);
    EXPECT_EQ(le::get_u16(&f.data[16]), 0x000B); // flags
    EXPECT_EQ(f.len, 24);
}

TEST(EdmPsuProto, EncodeControlStartCut) {
    CanFrame f = encodeControl(CTRL_START_CUT);
    EXPECT_EQ(f.id, ID_CONTROL);
    EXPECT_EQ(f.data[0], CTRL_START_CUT);
    EXPECT_EQ(f.len, 2);
}

TEST(EdmPsuProto, EncodeControlPolarityArg) {
    CanFrame f = encodeControl(CTRL_POLARITY, 1);
    EXPECT_EQ(f.data[0], CTRL_POLARITY);
    EXPECT_EQ(f.data[1], 1);
}

TEST(EdmPsuProto, DecodeStatsAggRoundTrip) {
    CanFrame f(ID_STATS_AGG, 38);
    uint8_t* d = f.data;
    EDM::le::put_u32(d + 0, 42);       // window_id
    EDM::le::put_u16(d + 4, 90);       // n_normal
    EDM::le::put_u16(d + 6, 5);        // n_arc
    EDM::le::put_u16(d + 8, 3);        // n_short
    EDM::le::put_u16(d + 10, 2);       // n_open
    EDM::le::put_u16(d + 12, 500);     // ign delay mean
    EDM::le::put_u16(d + 14, 40);      // ign delay stddev
    EDM::le::put_u16(d + 16, 110);     // peak_I_mean_dA
    EDM::le::put_u16(d + 18, 180);     // peak_I_max_dA
    EDM::le::put_u16(d + 20, 300);     // gap_V_recovery
    EDM::le::put_u32(d + 22, 123456);  // energy_uJ
    EDM::le::put_i16(d + 26, 425);     // temp_GaN 42.5C
    EDM::le::put_i16(d + 28, 380);     // temp_L 38.0C
    EDM::le::put_u16(d + 30, 800);     // dc_link_V 80.0V
    EDM::le::put_u16(d + 32, 50);      // dc_link_I 5.0A
    d[34] = 0;                         // state running
    d[35] = 1;                         // mode_id_active
    EDM::le::put_u16(d + 36, 0);       // flags

    StatsAgg s;
    ASSERT_TRUE(decodeStatsAgg(f, s));
    EXPECT_EQ(s.window_id, 42u);
    EXPECT_EQ(s.n_normal, 90);
    EXPECT_EQ(s.n_short, 3);
    EXPECT_EQ(s.energy_delivered_uJ, 123456u);
    EXPECT_EQ(s.temp_GaN_dC, 425);
    EXPECT_EQ(s.dc_link_V_dV, 800);
    EXPECT_EQ(s.mode_id_active, 1);
}

TEST(EdmPsuProto, DecodeStatsAggRejectsWrongId) {
    CanFrame f(0x999, 38);
    StatsAgg s;
    EXPECT_FALSE(decodeStatsAgg(f, s));
}

TEST(EdmPsuProto, DecodeStatsAggRejectsShortLen) {
    CanFrame f(ID_STATS_AGG, 10);
    StatsAgg s;
    EXPECT_FALSE(decodeStatsAgg(f, s));
}

TEST(EdmPsuProto, DecodeAckRoundTrip) {
    CanFrame f(ID_ACK_MODE_BOUNDS, 13);
    EDM::le::put_u16(f.data + 0, 0x1234);  // seq
    EDM::le::put_u16(f.data + 2, 190);     // clamped freq
    EDM::le::put_u16(f.data + 4, 800);
    EDM::le::put_u16(f.data + 6, 1200);
    EDM::le::put_u16(f.data + 8, 120);
    EDM::le::put_u16(f.data + 10, 200);
    f.data[12] = 0;                        // status ok
    AckModeBounds a;
    ASSERT_TRUE(decodeAckModeBounds(f, a));
    EXPECT_EQ(a.seq, 0x1234);
    EXPECT_EQ(a.freq_max_kHz, 190);
    EXPECT_EQ(a.status, 0);
}
