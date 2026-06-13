// FluidNC/tests/EdmModeTableTest.cpp
#include "gtest/gtest.h"
#include "EDM/Servo/ModeTable.h"

using namespace EDM::servo;
using namespace EDM::psu;

static ModeTable makeTable() {
    ModeTable t;
    t.rough  = { 1, 200, 800, 1200, 200, 260, 0, 0x000D, 25000, 8000, 30, 10000 };
    t.finish = { 2, 400, 200,  600,  80, 120, 0, 0x000D, 25000, 8000, 20,  6000 };
    t.ignite = { 3, 100, 300, 3000,  40,  80, 0, 0x0008, 25000, 8000, 50, 16000 };
    return t;
}

TEST(EdmModeTable, SwordBandsMapToModes) {
    ModeTable t = makeTable();
    EXPECT_EQ(t.modeForSword(0),   EdmMode::Idle);
    EXPECT_EQ(t.modeForSword(50),  EdmMode::Ignite);
    EXPECT_EQ(t.modeForSword(300), EdmMode::Finish);
    EXPECT_EQ(t.modeForSword(900), EdmMode::Rough);
}

TEST(EdmModeTable, BuildEmitsModeBoundsFields) {
    ModeTable t = makeTable();
    SetModeBounds b = t.build(EdmMode::Rough, 900, 7);
    EXPECT_EQ(b.mode_id, 1);
    EXPECT_EQ(b.freq_max_kHz, 200);
    EXPECT_EQ(b.peak_I_setpoint_dA, 200);
    EXPECT_EQ(b.seq, 7);
}
TEST(EdmModeTable, AdaptiveOffUsesFixedPeakI) {
    ModeTable t = makeTable(); t.adaptive_setpoint = false;
    EXPECT_EQ(t.build(EdmMode::Rough, 500, 1).peak_I_setpoint_dA, 200);
    EXPECT_EQ(t.build(EdmMode::Rough, 999, 1).peak_I_setpoint_dA, 200);
}
TEST(EdmModeTable, AdaptiveOnLerpsClampedToLimit) {
    ModeTable t = makeTable(); t.adaptive_setpoint = true;
    uint16_t lowS  = t.build(EdmMode::Rough, 500, 1).peak_I_setpoint_dA;
    uint16_t highS = t.build(EdmMode::Rough, 1000, 1).peak_I_setpoint_dA;
    EXPECT_LT(lowS, highS);
    EXPECT_LE(highS, t.rough.peak_I_limit_hw_dA);
}
TEST(EdmModeTable, LowerEnergySteps) {
    ModeTable t = makeTable();
    EXPECT_EQ(t.lowerEnergy(EdmMode::Rough),  EdmMode::Finish);
    EXPECT_EQ(t.lowerEnergy(EdmMode::Finish), EdmMode::Ignite);
    EXPECT_EQ(t.lowerEnergy(EdmMode::Ignite), EdmMode::Ignite);
}
