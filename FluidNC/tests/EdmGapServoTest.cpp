// FluidNC/tests/EdmGapServoTest.cpp
#include "gtest/gtest.h"
#include "EDM/Servo/GapServo.h"

using namespace EDM::servo;

static GapServoInput healthy() {
    GapServoInput in; in.valid = true;
    in.short_ratio = 0.08f; in.open_ratio = 0.15f; in.arc_ratio = 0.08f; in.normal_ratio = 0.69f;
    in.total_pulses = 100; in.feed_cap_mult = 1.0f;
    return in;
}

TEST(EdmGapServo, ResetYieldsHoldZeroedState) {
    GapServoState s = GapServo::reset();
    EXPECT_EQ(s.state, ServoState::Hold);
    EXPECT_FLOAT_EQ(s.e_f, 0.0f);
    EXPECT_FLOAT_EQ(s.integ, 0.0f);
    EXPECT_EQ(s.consec_short, 0);

    ServoConfig c; GapServo srv(c);
    GapServoInput in;  // valid=false
    GapServoOutput o = srv.step(s, in);
    EXPECT_FLOAT_EQ(o.v_cmd_mm_min, 0.0f);
    EXPECT_EQ(o.state, ServoState::Hold);
}
