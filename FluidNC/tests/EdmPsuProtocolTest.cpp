// FluidNC/tests/EdmPsuProtocolTest.cpp
#include "gtest/gtest.h"
#include "EDM/Psu/Protocol.h"
#include "EDM/Psu/Endian.h"
#include <cstring>

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

TEST(EdmPsuProto, DecodeWireBreak) {
    CanFrame f(ID_WIRE_BREAK, 14);
    f.data[0] = 3;                      // severity critical
    f.data[1] = 0x05;                   // cause flags
    EDM::le::put_u16(f.data + 2, 12);   // recent_short
    EDM::le::put_u16(f.data + 4, 7);    // recent_arc
    EDM::le::put_u32(f.data + 6, 9999); // delay_var
    EDM::le::put_u32(f.data + 10, 5000);// timestamp
    WireBreakImminent w;
    ASSERT_TRUE(decodeWireBreak(f, w));
    EXPECT_EQ(w.severity, 3);
    EXPECT_EQ(w.recent_short_count, 12);
    EXPECT_EQ(w.timestamp_ms_since_start, 5000u);
}

TEST(EdmPsuProto, DecodeFault) {
    CanFrame f(ID_FAULT, 8);
    f.data[0] = 2;  // fault_code
    f.data[1] = 3;  // severity
    f.data[2] = 0xAA;
    Fault flt;
    ASSERT_TRUE(decodeFault(f, flt));
    EXPECT_EQ(flt.fault_code, 2);
    EXPECT_EQ(flt.severity, 3);
    EXPECT_EQ(flt.detail[0], 0xAA);
}

TEST(EdmPsuProto, DecodeInfoNullTerminates) {
    CanFrame f(ID_INFO, 5);
    std::memcpy(f.data, "hello", 5);
    char buf[16] = {};
    ASSERT_TRUE(decodeInfo(f, buf, sizeof(buf)));
    EXPECT_STREQ(buf, "hello");
}

TEST(EdmPsuProto, DecodePsuStatus) {
    CanFrame f(ID_PSU_STATUS, 13);
    f.data[0] = 0;                       // state
    EDM::le::put_u16(f.data + 1, 0x0101);// fpga
    EDM::le::put_u16(f.data + 3, 0x0202);// mcu
    EDM::le::put_u16(f.data + 5, 1);     // protocol version
    EDM::le::put_u32(f.data + 7, 3600);  // uptime
    EDM::le::put_u16(f.data + 11, 0);    // fault_count
    PsuStatus st;
    ASSERT_TRUE(decodePsuStatus(f, st));
    EXPECT_EQ(st.protocol_version, 1);
    EXPECT_EQ(st.uptime_s, 3600u);
}
