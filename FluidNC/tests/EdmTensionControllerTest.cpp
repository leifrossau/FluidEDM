// FluidNC/tests/EdmTensionControllerTest.cpp
#include "gtest/gtest.h"
#include "EDM/Feed/TensionController.h"

using namespace EDM::feed;

static TensionInput healthy() {
    TensionInput in;
    in.measured_N = 5.0f; in.meas_valid = true; in.feed_cap_mult = 1.0f;
    in.severity = 0; in.cutting = true; in.commanded_feed_mm_min = -1.0f; in.dt_ms = 12;
    return in;
}

TEST(EdmTension, ResetYieldsZeroedState) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.cutting = false;
    TensionOutput o = t.step(TensionController::reset(), in);
    EXPECT_NEAR(o.integrator, 0.0f, 1e-6f);
    EXPECT_FALSE(o.tension_collapse);
    EXPECT_EQ(o.feed_steps_per_s, 0);
}
TEST(EdmTension, FeedScalesWithCapHealthy) {
    TensionConfig c; TensionController t(c);
    EXPECT_EQ(t.step(TensionController::reset(), healthy()).feed_steps_per_s, 100);
}
TEST(EdmTension, FeedScalesWithCapSev1Sev2) {
    TensionConfig c; TensionController t(c);
    TensionInput a = healthy(); a.feed_cap_mult = 0.6f;
    EXPECT_EQ(t.step(TensionController::reset(), a).feed_steps_per_s, 60);
    TensionInput b = healthy(); b.feed_cap_mult = 0.3f;
    EXPECT_EQ(t.step(TensionController::reset(), b).feed_steps_per_s, 30);
}
TEST(EdmTension, FeedZeroWhenNotCutting) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.cutting = false;
    EXPECT_EQ(t.step(TensionController::reset(), in).feed_steps_per_s, 0);
}
TEST(EdmTension, FeedZeroOnSev3Holds) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.severity = 3; in.feed_cap_mult = 0.0f;
    TensionOutput o = t.step(TensionController::reset(), in);
    EXPECT_EQ(o.feed_steps_per_s, 0);
    EXPECT_EQ(o.tension_steps_per_s, 200);
}
TEST(EdmTension, FeedforwardTracksFeed) {
    TensionConfig c; c.Kp = 0.0f; c.Ki = 0.0f; TensionController t(c);
    TensionOutput o = t.step(TensionController::reset(), healthy());
    EXPECT_NEAR(o.pi_sps, 0.0f, 1e-6f);
    EXPECT_EQ(o.tension_steps_per_s, o.feed_steps_per_s);
}
TEST(EdmTension, SlackIncreasesTension) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.measured_N = 3.0f;
    TensionState s = TensionController::reset(); TensionOutput o;
    for (int i = 0; i < 5; ++i) { o = t.step(s, in); s = o.next; }
    EXPECT_GT(o.e_filtered, 0.0f);
    EXPECT_GT(o.pi_sps, 0.0f);
    EXPECT_GT(o.tension_steps_per_s, int32_t(o.ff_sps));
}
TEST(EdmTension, OverTensionReducesTension) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.measured_N = 7.0f;
    TensionState s = TensionController::reset(); TensionOutput o;
    for (int i = 0; i < 5; ++i) { o = t.step(s, in); s = o.next; }
    EXPECT_LT(o.pi_sps, 0.0f);
}
TEST(EdmTension, DeadbandHoldsNoIntegrate) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.measured_N = 5.1f;
    TensionState s = TensionController::reset(); TensionOutput o;
    for (int i = 0; i < 10; ++i) { o = t.step(s, in); s = o.next; }
    EXPECT_NEAR(o.pi_sps, 0.0f, 1e-6f);
    EXPECT_NEAR(o.integrator, 0.0f, 1e-6f);
}
TEST(EdmTension, AntiWindupClampsIntegrator) {
    TensionConfig c; TensionController t(c);
    TensionInput in = healthy(); in.measured_N = 0.0f;
    TensionState s = TensionController::reset(); TensionOutput o;
    for (int i = 0; i < 400; ++i) { o = t.step(s, in); s = o.next; }
    EXPECT_LE(o.integrator, c.pi_clamp_sps / c.Ki + 1e-3f);
    EXPECT_LE(o.pi_sps, c.pi_clamp_sps + 1e-3f);
}
TEST(EdmTension, IntegralFrozenWhenStale) {
    TensionConfig c; TensionController t(c);
    TensionState s = TensionController::reset();
    TensionInput load = healthy(); load.measured_N = 3.0f;
    for (int i = 0; i < 5; ++i) s = t.step(s, load).next;
    float integ_before = s.integ, meas_before = s.meas_f;
    TensionInput stale = healthy(); stale.measured_N = 3.0f; stale.meas_valid = false;
    for (int i = 0; i < 10; ++i) s = t.step(s, stale).next;
    EXPECT_NEAR(s.integ, integ_before, 1e-6f);
    EXPECT_NEAR(s.meas_f, meas_before, 1e-6f);
}
TEST(EdmTension, Sev3BleedsIntegratorAndHolds) {
    TensionConfig c; TensionController t(c);
    TensionState s = TensionController::reset();
    TensionInput load = healthy(); load.measured_N = 3.0f;
    for (int i = 0; i < 20; ++i) s = t.step(s, load).next;
    float integ_before = s.integ; ASSERT_GT(integ_before, 0.1f);
    TensionInput sev3 = healthy(); sev3.severity = 3; sev3.feed_cap_mult = 0.0f;
    TensionOutput o = t.step(s, sev3);
    EXPECT_EQ(o.feed_steps_per_s, 0);
    EXPECT_EQ(o.tension_steps_per_s, 200);
    EXPECT_NEAR(o.next.integ, integ_before * c.integ_bleed, 1e-3f);
}
TEST(EdmTension, Sev1TrimsTensionSetpoint) {
    TensionConfig c; TensionController t(c);
    TensionInput sev0 = healthy(); sev0.measured_N = 5.0f; sev0.severity = 0;
    TensionInput sev1 = healthy(); sev1.measured_N = 5.0f; sev1.severity = 1; sev1.feed_cap_mult = 0.6f;
    TensionOutput o0 = t.step(TensionController::reset(), sev0);
    TensionOutput o1 = t.step(TensionController::reset(), sev1);
    EXPECT_LT(o1.tension_steps_per_s, o0.tension_steps_per_s);
}
TEST(EdmTension, CollapseLatchesThreeConditionDebounced) {
    TensionConfig c; TensionController t(c);
    TensionState s0 = TensionController::reset();
    TensionInput idle = healthy(); idle.measured_N = 0.1f; idle.cutting = false;
    for (int i = 0; i < 20; ++i) s0 = t.step(s0, idle).next;
    EXPECT_FALSE(s0.collapse_latched);

    TensionState s = TensionController::reset();
    TensionInput load = healthy(); load.measured_N = 5.0f;
    for (int i = 0; i < 5; ++i) s = t.step(s, load).next;
    TensionInput drop = healthy(); drop.measured_N = 0.1f;
    TensionOutput o;
    o = t.step(s, drop); s = o.next; EXPECT_FALSE(o.tension_collapse);
    o = t.step(s, drop); s = o.next; EXPECT_FALSE(o.tension_collapse);
    o = t.step(s, drop); s = o.next; EXPECT_TRUE(o.tension_collapse);
    TensionInput rec = healthy(); rec.measured_N = 5.0f;
    o = t.step(s, rec);
    EXPECT_TRUE(o.tension_collapse);
}
