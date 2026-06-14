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
#include "EDM/Feed/WireFeed.h"
#include "EDM/Diel/IDielLink.h"
#include "EDM/Diel/SimDielLink.h"
#include "EDM/Diel/DielProtocol.h"

#include "Configuration/Configurable.h"

#include <memory>

namespace Spindles {

// Dielectric (coolant) YAML sub-section, mirroring EDM::feed::WireFeedConfig:
// a parser-allocated Configurable with its own group(). Nullable — a missing
// `dielectric:` section leaves the spindle's built-in defaults in force.
struct EdmDielConfig : public Configuration::Configurable {
    bool     use_sim              = true;
    bool     required             = false;
    uint32_t flow_min_clpm        = 100;
    uint32_t level_min_pct        = 15;
    uint32_t conductivity_warn_uS = 20;
    int32_t  temp_setpoint_dC     = 220;   // 0.1 C -> 22.0 C
    uint32_t default_flush_level  = 2;     // 0=off,1=low,2=med,3=high

    void group(Configuration::HandlerBase& handler) override {
        handler.item("use_sim", use_sim);
        handler.item("required", required);
        handler.item("flow_min_clpm", flow_min_clpm, 0u, 65535u);
        handler.item("level_min_pct", level_min_pct, 0u, 100u);
        handler.item("conductivity_warn_uS", conductivity_warn_uS, 0u, 65535u);
        handler.item("temp_setpoint_dC", temp_setpoint_dC, -400, 1000);
        handler.item("default_flush_level", default_flush_level, 0u, 3u);
    }
};

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

    EDM::EdmController* controller() const { return _ctl.get(); }

    // Wire feed (null when no wire_feed: YAML section was parsed). Used by the
    // $EDM/Tension and $EDM/ResetWireFeed commands and by EdmReportChannel.
    EDM::feed::WireFeed* wireFeed() const { return _feed.get(); }

    // Dielectric (coolant) link. Always non-null after init() (the sim is the
    // default). Used by the $EDM/Flush and $EDM/Pump commands. The configured
    // temp setpoint + deioniser default are carried so a command that only
    // changes the flush level / pump does not clobber the other fields.
    EDM::diel::IDielLink* dielLink() const { return _diel.get(); }

    // A SetDiel pre-populated from the configured defaults (temp setpoint,
    // deioniser on, current default flush level). Command handlers start from
    // this and override only the field they change. Reads the effective values
    // resolved by init() (config section if present, else built-in defaults).
    EDM::diel::SetDiel dielDefaults() const {
        EDM::diel::SetDiel s;
        s.flush_level      = static_cast<uint8_t>(_diel_default_flush_level);
        s.temp_setpoint_dC = static_cast<int16_t>(_diel_temp_setpoint_dC);
        s.deioniser_enable = 1;
        s.pump_on          = 1;
        return s;
    }

    ~EdmSpindle() {}

private:
    EDM::servo::ServoConfig            _cfg;
    EDM::servo::ModeTable              _modes;
    bool                               _use_sim   = true;
    int32_t                            _report_hz = 10;
    std::unique_ptr<EDM::psu::IPsuLink> _link;
    std::unique_ptr<EDM::EdmController> _ctl;

    // Wire-feed hardware. _feed_cfg is parser-allocated (pointer section idiom);
    // null when the YAML has no wire_feed: section, in which case the feed stays off.
    EDM::feed::WireFeedConfig*          _feed_cfg = nullptr;
    std::unique_ptr<EDM::feed::WireFeed> _feed;

    // Dielectric (coolant) config sub-section (parser-allocated, nullable —
    // pointer-section idiom, same as _feed_cfg). Null when the YAML has no
    // dielectric: section, in which case the effective fields below keep their
    // built-in defaults.
    EdmDielConfig*                        _diel_cfg = nullptr;

    // Dielectric link. Owned for the firmware lifetime. _diel is always
    // constructed in init() (SimDielLink by default), so the controller's
    // interlock and the report always have a source. _sim_diel is a non-owning
    // alias to the concrete sim (when used) so the diel task can drive its model
    // (setCutting/tick); null when a CAN link is used.
    std::unique_ptr<EDM::diel::IDielLink> _diel;
    EDM::diel::SimDielLink*               _sim_diel = nullptr;

    // Effective dielectric defaults, resolved in init() from _diel_cfg (when
    // present) or these built-ins. Kept on the spindle so dielDefaults() and the
    // interlock build do not depend on the (nullable) config object's lifetime.
    int32_t  _diel_temp_setpoint_dC   = 220;   // 0.1 C -> 22.0 C
    uint32_t _diel_default_flush_level = 2;     // 0=off,1=low,2=med,3=high
};

}  // namespace Spindles
