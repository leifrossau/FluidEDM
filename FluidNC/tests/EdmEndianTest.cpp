// FluidNC/tests/EdmEndianTest.cpp
#include "gtest/gtest.h"
#include "EDM/Psu/Endian.h"

using namespace EDM;

TEST(EdmEndian, U16RoundTrip) {
    uint8_t buf[2] = {};
    le::put_u16(buf, 0xBEEF);
    EXPECT_EQ(buf[0], 0xEF);
    EXPECT_EQ(buf[1], 0xBE);
    EXPECT_EQ(le::get_u16(buf), 0xBEEF);
}

TEST(EdmEndian, U32RoundTrip) {
    uint8_t buf[4] = {};
    le::put_u32(buf, 0x01020304u);
    EXPECT_EQ(buf[0], 0x04);
    EXPECT_EQ(buf[3], 0x01);
    EXPECT_EQ(le::get_u32(buf), 0x01020304u);
}

TEST(EdmEndian, I16Negative) {
    uint8_t buf[2] = {};
    le::put_i16(buf, int16_t(-300));
    EXPECT_EQ(le::get_i16(buf), int16_t(-300));
}
