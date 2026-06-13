// FluidNC/tests/EdmContourBufferTest.cpp
#include "gtest/gtest.h"
#include "EDM/Motion/ContourBuffer.h"
#include <cmath>

using namespace EDM::motion;

TEST(EdmContour, AppendLineCumulativeXyArcLength) {
    ContourBuffer c; c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{3,4,3,4});
    EXPECT_NEAR(c.totalLen(), 5.0f, 1e-4f);
    EXPECT_EQ(c.count(), 2u);
}
TEST(EdmContour, PoseAtInterpolatesXYUV) {
    ContourBuffer c; c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{10,0,10,2});
    Pose4 p = c.poseAt(5.0f);
    EXPECT_NEAR(p.x, 5.0f, 1e-4f); EXPECT_NEAR(p.y, 0.0f, 1e-4f);
    EXPECT_NEAR(p.u, 5.0f, 1e-4f); EXPECT_NEAR(p.v, 1.0f, 1e-4f);
}
TEST(EdmContour, PoseAtClampsEnds) {
    ContourBuffer c; c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{10,0,10,0});
    EXPECT_NEAR(c.poseAt(-1.0f).x, 0.0f, 1e-4f);
    EXPECT_NEAR(c.poseAt(99.0f).x, 10.0f, 1e-4f);
}
TEST(EdmContour, PureUvPlungeStillMonotonicS) {
    ContourBuffer c; c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{0,0,0,2});
    EXPECT_NEAR(c.totalLen(), 2.0f, 1e-4f);
    EXPECT_EQ(c.count(), 2u);
}
TEST(EdmContour, DropsDegenerateVertex) {
    ContourBuffer c; c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{5,0,5,0});
    c.appendLine(Pose4{5,0,5,0});
    EXPECT_EQ(c.count(), 2u);
}
TEST(EdmContour, AppendArcMatchesFluidncSegmentCount) {
    ContourBuffer c; c.reset(Pose4{10,0,10,0});
    float tol = 0.002f, r = 10.0f, theta = 1.5707963f;
    int expectN = int(std::floor(std::fabs(0.5f*theta*r)/std::sqrt(tol*(2*r-tol))));
    c.appendArc(Pose4{0,10,0,10}, 0.0f, 0.0f, r, theta, tol);
    EXPECT_EQ(c.count(), size_t(expectN + 1));
    Pose4 last = c.poseAt(c.totalLen());
    EXPECT_NEAR(last.x, 0.0f, 1e-2f);
    EXPECT_NEAR(last.y, 10.0f, 1e-2f);
    EXPECT_NEAR(c.totalLen(), 3.14159f*r/2.0f, 0.05f);
}
TEST(EdmContour, RetractFloorRespectsWindow) {
    ContourBuffer c; c.reset(Pose4{0,0,0,0});
    c.appendLine(Pose4{20,0,20,0});
    EXPECT_NEAR(c.retractFloor(12.0f, 3.0f), 9.0f, 1e-4f);
    EXPECT_NEAR(c.retractFloor(2.0f, 3.0f), 0.0f, 1e-4f);
}
TEST(EdmContour, EvictBelowKeepsStraddleVertex) {
    ContourBuffer c; c.reset(Pose4{0,0,0,0});
    for (int i = 1; i <= 20; ++i) c.appendLine(Pose4{float(i),0,float(i),0});
    float before = c.poseAt(9.5f).x;
    c.evictBelow(9.0f);
    EXPECT_LE(c.startS(), 9.0f);
    EXPECT_NEAR(c.poseAt(9.5f).x, before, 1e-3f);
}
