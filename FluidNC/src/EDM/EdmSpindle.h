// FluidNC/src/EDM/EdmSpindle.h
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Uses the FluidNC Spindle base + FreeRTOS servo task; verified by the ESP32 build (P4).
//
// EdmSpindle is a Spindle facade over the EDM gap-servo stack: it owns the PSU
// link and the EdmController, runs the 1 kHz servo task, and maps M3/M5 + S-word
// onto requestCut()/requestStop(). Structure mirrors Spindles::Laser.
#pragma once

#include "../Spindles/Spindle.h"

#include "EDM/Control/EdmController.h"
#include "EDM/Servo/GapServo.h"
#include "EDM/Servo/ModeTable.h"
#include "EDM/Psu/IPsuLink.h"

#include <memory>

namespace Spindles {

class EdmSpindle : public Spindle {
public:
    EdmSpindle() : Spindle("EDM") {}

    EdmSpindle(const EdmSpindle&)            = delete;
    EdmSpindle(EdmSpindle&&)                 = delete;
    EdmSpindle& operator=(const EdmSpindle&) = delete;
    EdmSpindle& operator=(EdmSpindle&&)      = delete;

    void init() override;
    void setState(SpindleState state, uint32_t speed) override;
    void setSpeedfromISR(uint32_t /*dev_speed*/) override {}
    bool isRateAdjusted() override { return true; }
    void config_message() override;
    void group(Configuration::HandlerBase& handler) override;

    ~EdmSpindle() {}

private:
    EDM::servo::ServoConfig            _cfg;
    EDM::servo::ModeTable              _modes;
    bool                               _use_sim   = true;
    int32_t                            _report_hz = 10;
    std::unique_ptr<EDM::psu::IPsuLink> _link;
    std::unique_ptr<EDM::EdmController> _ctl;
};

}  // namespace Spindles
