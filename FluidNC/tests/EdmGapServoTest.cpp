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

// ---- Group T3 (error / EWMA / g) ----------------------------------------
TEST(EdmGapServo, ErrorFormulaSignsCorrect) {
    ServoConfig c; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.60f; wide.short_ratio = 0.02f; wide.normal_ratio = 0.30f;
    EXPECT_GT(srv.step(GapServo::reset(), wide).e_filtered, 0.0f);
    GapServoInput tight = healthy(); tight.short_ratio = 0.25f; tight.open_ratio = 0.02f; tight.normal_ratio = 0.65f;
    EXPECT_LT(srv.step(GapServo::reset(), tight).e_filtered, 0.0f);
}
TEST(EdmGapServo, EwmaSmoothsAcrossWindows) {
    ServoConfig c; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.60f; wide.short_ratio = 0.02f; wide.normal_ratio = 0.30f;
    GapServoState s = GapServo::reset();
    float e1 = srv.step(s, wide).e_filtered;
    s = srv.step(s, wide).next;
    float e2 = srv.step(s, wide).e_filtered;
    EXPECT_GT(e2, e1);
}
TEST(EdmGapServo, GTargetMatchesFormula) {
    ServoConfig c; GapServo srv(c);
    GapServoOutput o = srv.step(GapServo::reset(), healthy());
    EXPECT_NEAR(o.g_target, c.w_open * c.open_ref - c.w_short * c.short_ref, 1e-6f);
}

// ---- Group T4 (deadband + Ki=0) -----------------------------------------
TEST(EdmGapServo, HealthyBandHoldsInDeadband) {
    ServoConfig c; GapServo srv(c);
    GapServoState s = GapServo::reset();
    for (int i = 0; i < 10; ++i) s = srv.step(s, healthy()).next;
    GapServoOutput o = srv.step(s, healthy());
    EXPECT_NEAR(o.v_cmd_mm_min, 0.0f, 1e-3f);
    EXPECT_EQ(o.state, ServoState::Hold);
    EXPECT_FLOAT_EQ(o.integrator, 0.0f);
}
TEST(EdmGapServo, KiZeroIsPureProportional) {
    ServoConfig c; c.Ki = 0.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.60f; wide.short_ratio = 0.02f; wide.normal_ratio = 0.30f;
    GapServoState s = GapServo::reset();
    for (int i = 0; i < c.pi_decimation; ++i) s = srv.step(s, wide).next;
    GapServoOutput o = srv.step(s, wide);
    EXPECT_FLOAT_EQ(o.integrator, 0.0f);
    EXPECT_GT(o.v_cmd_mm_min, 0.0f);
    EXPECT_LE(o.v_cmd_mm_min, c.v_feed_max);
}

// ---- Group T5 (advance / retract clamps) --------------------------------
TEST(EdmGapServo, WideGapAdvancesClampedToFeedMax) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.70f; wide.short_ratio = 0.0f; wide.normal_ratio = 0.30f;
    GapServoState s = GapServo::reset();
    GapServoOutput o;
    for (int i = 0; i < 200; ++i) { o = srv.step(s, wide); s = o.next; }
    EXPECT_GT(o.v_cmd_mm_min, 0.0f);
    EXPECT_LE(o.v_cmd_mm_min, c.v_feed_max + 1e-4f);
}
TEST(EdmGapServo, ModerateTightRetractsClampedToRetract) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput tight = healthy(); tight.short_ratio = 0.12f; tight.open_ratio = 0.0f; tight.arc_ratio = 0.0f; tight.normal_ratio = 0.88f;
    GapServoState s = GapServo::reset();
    GapServoOutput o;
    for (int i = 0; i < 200; ++i) { o = srv.step(s, tight); s = o.next; }
    EXPECT_LT(o.v_cmd_mm_min, 0.0f);
    EXPECT_GE(o.v_cmd_mm_min, -c.v_retract - 1e-4f);
}

// ---- Group T6 (PI integrator + anti-windup) -----------------------------
TEST(EdmGapServo, IntegratorAdvancesOnlyOnDecimation) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.60f; wide.short_ratio = 0.0f; wide.normal_ratio = 0.40f;
    GapServoState s = GapServo::reset();
    float prev = 0.0f; int changes = 0;
    for (int i = 1; i <= c.pi_decimation * 3; ++i) {
        GapServoOutput o = srv.step(s, wide); s = o.next;
        if (o.integrator != prev) { changes++; prev = o.integrator; }
    }
    EXPECT_EQ(changes, 3);
}
TEST(EdmGapServo, AntiWindupClampsInteg) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.95f; wide.short_ratio = 0.0f; wide.normal_ratio = 0.05f;
    GapServoState s = GapServo::reset();
    GapServoOutput o;
    for (int i = 0; i < 2000; ++i) { o = srv.step(s, wide); s = o.next; }
    EXPECT_LE(o.integrator, c.v_feed_max / c.Ki + 1e-3f);
    EXPECT_LE(o.v_cmd_mm_min, c.v_feed_max + 1e-4f);
}

// ---- Group T7 (short retract reflex) ------------------------------------
TEST(EdmGapServo, ShortRatioAboveThresholdForcesMaxRetract) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput sh = healthy(); sh.short_ratio = 0.40f; sh.open_ratio = 0.0f; sh.normal_ratio = 0.52f;
    GapServoOutput o = srv.step(GapServo::reset(), sh);
    EXPECT_FLOAT_EQ(o.v_cmd_mm_min, -c.v_retract);
    EXPECT_EQ(o.state, ServoState::Retract);
}
TEST(EdmGapServo, ConsecutiveShortWindowsRetract) {
    ServoConfig c; c.Ki = 0.0f; GapServo srv(c);
    GapServoInput sh = healthy(); sh.short_ratio = 0.20f; sh.open_ratio = 0.0f; sh.normal_ratio = 0.72f;
    GapServoState s = GapServo::reset();
    GapServoOutput o = srv.step(s, sh); s = o.next;
    EXPECT_NE(o.state, ServoState::Retract);
    o = srv.step(s, sh); s = o.next;
    EXPECT_NE(o.state, ServoState::Retract);
    o = srv.step(s, sh);
    EXPECT_EQ(o.state, ServoState::Retract);
    EXPECT_FLOAT_EQ(o.v_cmd_mm_min, -c.v_retract);
}

// ---- Group T8 (integ bleed) ---------------------------------------------
TEST(EdmGapServo, RetractBleedsIntegrator) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.60f; wide.short_ratio = 0.0f; wide.normal_ratio = 0.40f;
    GapServoState s = GapServo::reset();
    GapServoOutput o;
    // 400 windows (multiple of pi_decimation) so the integrator winds past 0.1
    // and decim_count is 0 entering the retract step (no extra PI accrual before bleed).
    for (int i = 0; i < 400; ++i) { o = srv.step(s, wide); s = o.next; }
    float wound = o.integrator;
    ASSERT_GT(wound, 0.1f);
    GapServoInput sh = healthy(); sh.short_ratio = 0.40f; sh.open_ratio = 0.0f; sh.normal_ratio = 0.52f;
    o = srv.step(s, sh);
    EXPECT_NEAR(o.integrator, wound * c.integ_bleed, 1e-3f);
}

// ---- Group T9 (arc brake / hold) ----------------------------------------
TEST(EdmGapServo, ArcHoldZeroesNeverRetracts) {
    ServoConfig c; GapServo srv(c);
    GapServoInput a = healthy(); a.arc_ratio = 0.50f; a.short_ratio = 0.05f; a.open_ratio = 0.10f; a.normal_ratio = 0.35f;
    GapServoOutput o = srv.step(GapServo::reset(), a);
    EXPECT_FLOAT_EQ(o.v_cmd_mm_min, 0.0f);
    EXPECT_EQ(o.state, ServoState::ArcHold);
}
TEST(EdmGapServo, ArcBrakeTapersAdvance) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    // open=0.95 saturates the un-braked PI output at v_feed_max so the arc taper
    // cap (3.5 mm/min at arc_ratio 0.30) is strictly below it and actually bites.
    GapServoInput wide = healthy(); wide.open_ratio = 0.95f; wide.short_ratio = 0.0f; wide.arc_ratio = 0.0f; wide.normal_ratio = 0.05f;
    GapServoState s = GapServo::reset();
    GapServoOutput no_arc;
    for (int i = 0; i < 200; ++i) { no_arc = srv.step(s, wide); s = no_arc.next; }
    GapServoInput braked = wide; braked.arc_ratio = 0.30f; braked.normal_ratio = 0.10f;
    GapServoOutput o = srv.step(s, braked);
    EXPECT_GT(o.v_cmd_mm_min, 0.0f);
    EXPECT_LT(o.v_cmd_mm_min, no_arc.v_cmd_mm_min);
    EXPECT_EQ(o.state, ServoState::Track);
}

// ---- Group T10 (invalid hold + touch-off) -------------------------------
TEST(EdmGapServo, LowPulseCountHolds) {
    ServoConfig c; GapServo srv(c);
    GapServoInput bad; bad.valid = false; bad.total_pulses = 5;
    GapServoState s = GapServo::reset(); s.consec_short = 2; s.integ = 1.0f;
    GapServoOutput o = srv.step(s, bad);
    EXPECT_FLOAT_EQ(o.v_cmd_mm_min, 0.0f);
    EXPECT_EQ(o.state, ServoState::Hold);
    EXPECT_FLOAT_EQ(o.integrator, 1.0f);
    EXPECT_EQ(o.next.consec_short, 2);
}
TEST(EdmGapServo, TouchOffCommandsConstantApproach) {
    ServoConfig c; GapServo srv(c);
    GapServoInput in = healthy(); in.in_touch_off = true;
    GapServoOutput o = srv.step(GapServo::reset(), in);
    EXPECT_FLOAT_EQ(o.v_cmd_mm_min, c.v_touch);
    EXPECT_EQ(o.state, ServoState::Track);
}

// ---- Group T11 (feed cap) -----------------------------------------------
TEST(EdmGapServo, FeedCapThrottlesAdvanceNotRetract) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.70f; wide.short_ratio = 0.0f; wide.normal_ratio = 0.30f;
    GapServoState s = GapServo::reset();
    GapServoOutput full;
    for (int i = 0; i < 200; ++i) { full = srv.step(s, wide); s = full.next; }
    GapServoInput capped = wide; capped.feed_cap_mult = 0.6f;
    GapServoOutput o = srv.step(s, capped);
    EXPECT_NEAR(o.v_cmd_mm_min, full.v_cmd_mm_min * 0.6f, 1e-3f);
    GapServoInput sh = healthy(); sh.short_ratio = 0.40f; sh.open_ratio = 0.0f; sh.normal_ratio = 0.52f; sh.feed_cap_mult = 0.3f;
    GapServoOutput r = srv.step(GapServo::reset(), sh);
    EXPECT_FLOAT_EQ(r.v_cmd_mm_min, -c.v_retract);
}

// ---- Group T12 (velocity conversion) ------------------------------------
TEST(EdmGapServo, VelocityConvertsToMicrometersPerSecond) {
    ServoConfig c; c.Ki = 2.0f; GapServo srv(c);
    GapServoInput wide = healthy(); wide.open_ratio = 0.95f; wide.short_ratio = 0.0f; wide.normal_ratio = 0.05f;
    GapServoState s = GapServo::reset();
    GapServoOutput o;
    for (int i = 0; i < 2000; ++i) { o = srv.step(s, wide); s = o.next; }
    EXPECT_EQ(o.v_s_um_s, 67);
    GapServoInput sh = healthy(); sh.short_ratio = 0.40f; sh.open_ratio = 0.0f; sh.normal_ratio = 0.52f;
    GapServoOutput r = srv.step(GapServo::reset(), sh);
    EXPECT_EQ(r.v_s_um_s, -100);
}
