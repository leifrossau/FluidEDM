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
