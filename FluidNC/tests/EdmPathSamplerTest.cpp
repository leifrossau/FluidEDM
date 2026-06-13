// FluidNC/tests/EdmPathSamplerTest.cpp
#include "gtest/gtest.h"
#include "EDM/Motion/PathSampler.h"

using namespace EDM::motion;

static ContourBuffer lineContour(float len) {
    ContourBuffer c; c.reset(Pose4{0,0,0,0}); c.appendLine(Pose4{len,0,len,0}); return c;
}

TEST(EdmPathSampler, AdvanceAccumulatesUntilSegMin) {
    ContourBuffer c = lineContour(1.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st; int emits = 0; SampleResult r;
    for (int i = 0; i < 400; ++i) { r = s.step(st, 66667, 0.001f); if (r.emit != Emit::None) emits++; }
    EXPECT_GE(emits, 1);
    EXPECT_LE(emits, 5);              // accumulator: not every tick
}
TEST(EdmPathSampler, ReachesEndSetsDoneNoOvershoot) {
    ContourBuffer c = lineContour(0.5f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st; bool sawDone = false;
    for (int i = 0; i < 20000 && !st.done; ++i) {
        SampleResult r = s.step(st, 100000, 0.001f);
        ASSERT_LE(r.s_target, 0.5f + 1e-3f);
        if (r.block_done) sawDone = true;
    }
    EXPECT_TRUE(sawDone);
    EXPECT_TRUE(st.done);
    EXPECT_LE(st.s, 0.5f + 1e-4f);
}
TEST(EdmPathSampler, HoldEmitsNothingNotDone) {
    ContourBuffer c = lineContour(1.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    for (int i = 0; i < 500; ++i) { SampleResult r = s.step(st, 0, 0.001f); EXPECT_EQ(r.emit, Emit::None); }
    EXPECT_FALSE(st.done);
    EXPECT_NEAR(st.s, 0.0f, 1e-6f);
}
TEST(EdmPathSampler, DeadBandBelowVFloorIsHold) {
    ContourBuffer c = lineContour(1.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    for (int i = 0; i < 10; ++i) { SampleResult r = s.step(st, 3, 0.001f); EXPECT_EQ(r.emit, Emit::None); }
    EXPECT_NEAR(st.s, 0.0f, 1e-6f);
}
TEST(EdmPathSampler, AccelClampRampsNotSteps) {
    ContourBuffer c = lineContour(10.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    s.step(st, 100000, 0.001f);
    EXPECT_LE(st.v_mm_s, cfg.a_max_mm_s2 * 0.001f + 1e-6f);
}
TEST(EdmPathSampler, SegMaxCapsOneTickJump) {
    ContourBuffer c = lineContour(10.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st; SampleResult r;
    for (int i = 0; i < 5000 && !st.done; ++i) {
        r = s.step(st, 100000, 0.100f);
        if (r.emit != Emit::None) EXPECT_LE(r.seg_len_mm, cfg.seg_max_mm + 1e-4f);
    }
}
TEST(EdmPathSampler, DtClampHiccupBounded) {
    ContourBuffer c = lineContour(10.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    for (int i = 0; i < 50; ++i) s.step(st, 100000, 0.001f);
    SampleResult r = s.step(st, 100000, 0.500f);
    if (r.emit != Emit::None) EXPECT_LE(r.seg_len_mm, cfg.seg_max_mm + 1e-4f);
}
TEST(EdmPathSampler, RetractDecreasesSClampedToFloor) {
    ContourBuffer c = lineContour(20.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    // v is capped at v_max_mm_s (0.1 mm/s); reaching ~10 mm needs ~100k ticks at dt=1ms.
    for (int i = 0; i < 110000; ++i) { s.step(st, 100000, 0.001f); if (st.s_max >= 10.0f) break; }
    float smax = st.s_max; ASSERT_GE(smax, 9.0f);
    float floor = smax - cfg.retract_max_mm;
    bool hitFloor = false;
    // retract is capped at v_max_mm_s (0.1 mm/s); reaching the 3 mm floor needs ~30k ticks.
    for (int i = 0; i < 40000; ++i) {
        SampleResult r = s.step(st, -100000, 0.001f);
        if (r.at_retract_floor) hitFloor = true;
        EXPECT_GE(st.s, floor - 1e-3f);
    }
    EXPECT_TRUE(hitFloor);
    EXPECT_NEAR(st.s_max, smax, 1e-3f);
}
TEST(EdmPathSampler, RetractNeverPastContourStart) {
    ContourBuffer c = lineContour(1.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    for (int i = 0; i < 50; ++i) s.step(st, 100000, 0.001f);
    for (int i = 0; i < 5000; ++i) { s.step(st, -100000, 0.001f); EXPECT_GE(st.s, -1e-4f); }
}
TEST(EdmPathSampler, ReversalForcesEmitAtReversalPoint) {
    ContourBuffer c = lineContour(10.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    for (int i = 0; i < 100; ++i) { SampleResult r = s.step(st, 66667, 0.001f); if (r.emit != Emit::None) break; }
    SampleResult r = s.step(st, -100000, 0.001f);
    EXPECT_EQ(st.dir, int8_t(-1));
}
TEST(EdmPathSampler, DoneOnlyWhenAdvancingAtEnd) {
    ContourBuffer c = lineContour(0.5f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st;
    for (int i = 0; i < 20000 && !st.done; ++i) s.step(st, 100000, 0.001f);
    EXPECT_TRUE(st.done);
}
TEST(EdmPathSampler, FeedMapsToMmMinWithFloor) {
    ContourBuffer c = lineContour(10.0f);
    SamplerConfig cfg; PathSampler s(c, cfg);
    SamplerState st; SampleResult r;
    for (int i = 0; i < 500; ++i) { r = s.step(st, 66667, 0.001f); if (r.emit != Emit::None) break; }
    ASSERT_NE(r.emit, Emit::None);
    EXPECT_GT(r.feed_mm_min, 0.0f);
    EXPECT_GE(r.feed_mm_min, cfg.feed_floor_mm_min - 1e-4f);
    // v_cmd 66667 um/s saturates v_max_mm_s (0.1 mm/s) -> 0.1*60 = 6.0 mm/min.
    EXPECT_NEAR(r.feed_mm_min, 6.0f, 0.6f);
}
