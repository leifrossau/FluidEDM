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
