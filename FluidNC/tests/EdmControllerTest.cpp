// FluidNC/tests/EdmControllerTest.cpp
#include "gtest/gtest.h"
#include "EDM/Control/EdmController.h"
#include "EDM/Psu/SimPsuLink.h"

using namespace EDM;
using namespace EDM::servo;
using namespace EDM::psu;

static ModeTable makeModes() {
    ModeTable t;
    t.rough  = { 1, 200, 800, 1200, 200, 260, 0, 0x000D, 25000, 8000, 30, 10000 };
    t.finish = { 2, 400, 200,  600,  80, 120, 0, 0x000D, 25000, 8000, 20,  6000 };
    t.ignite = { 3, 100, 300, 3000,  40,  80, 0, 0x0008, 25000, 8000, 50, 16000 };
    return t;
}

TEST(EdmController, ArmSendsBoundsWaitsAckStartsCut) {
    SimPsuLink sim; sim.begin();
    ServoConfig cfg; ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);
    ctl.requestCut(900);
    ctl.tick(0);
    EXPECT_EQ(ctl.state(), EdmState::Armed);
    ctl.tick(1);
    EXPECT_EQ(ctl.state(), EdmState::TouchOff);
}

TEST(EdmController, RepeatedSameModeKeepsMode) {
    SimPsuLink sim; sim.begin();
    ServoConfig cfg; ModeTable modes = makeModes();
    EdmController ctl(sim, cfg, modes);
    ctl.requestCut(900); ctl.tick(0);
    uint8_t m1 = ctl.snapshot().active_mode_id;
    ctl.requestCut(950);
    ctl.tick(1);
    EXPECT_EQ(ctl.snapshot().active_mode_id, m1);  // still Rough
}
