// FluidNC/tests/EdmHx711CalibrationTest.cpp
#include "gtest/gtest.h"
#include "EDM/Feed/Hx711Calibration.h"

using namespace EDM::feed;

TEST(EdmHx711Cal, SignExtendBoundaries) {
    EXPECT_EQ(hx711SignExtend(0x800000u), -8388608);
    EXPECT_EQ(hx711SignExtend(0xFFFFFFu), -1);
    EXPECT_EQ(hx711SignExtend(0x7FFFFFu),  8388607);
    EXPECT_EQ(hx711SignExtend(0x000000u),  0);
}
TEST(EdmHx711Cal, ToNewtonsAffine) {
    Hx711Calibration cal{ 84213, 4271.5f };
    EXPECT_NEAR(cal.toNewtons(84213), 0.0f, 1e-3f);
    EXPECT_NEAR(cal.toNewtons(84213 + int32_t(4271.5f * 5)), 5.0f, 1e-2f);
}
