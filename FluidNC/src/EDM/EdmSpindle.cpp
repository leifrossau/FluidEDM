// FluidNC/src/EDM/EdmSpindle.cpp
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Uses the FluidNC Spindle base, GenericFactory registration and the FreeRTOS
// servo task; verified by the ESP32 firmware build (P4).
//
// Registration + group()/config_message() idiom follows Spindles::Laser
// (LaserSpindle.cpp): SpindleFactory::InstanceBuilder<...> registration("Name").

#include "EdmSpindle.h"

#include "EDM/Servo/ModeTable.h"
#include "EDM/Psu/SimPsuLink.h"
#include "EDM/EdmServoTask.h"
#include "EDM/EdmWireFeedTask.h"

#include "Logging.h"  // log_info

#include <memory>

namespace Spindles {

void EdmSpindle::init() {
    // Build the PSU link. SimPsuLink is the in-firmware simulator used for
    // bring-up; the real CAN path is wired in P4.
    if (_use_sim) {
        auto sim = std::make_unique<EDM::psu::SimPsuLink>();
        sim->begin();
        _link = std::move(sim);
    } else {
        // TODO(P4): construct CanPsuLink over Mcp2518fdDriver (EDM/Can/*) here.
        // Until that path exists, fall back to the simulator so init() is safe.
        log_info(name() << " CAN PSU link not yet wired; using SimPsuLink");
        auto sim = std::make_unique<EDM::psu::SimPsuLink>();
        sim->begin();
        _link = std::move(sim);
    }

    // Controller takes the link by reference; _link (and thus *_link) is owned
    // by this spindle for the firmware lifetime, so the reference stays valid.
    _ctl = std::make_unique<EDM::EdmController>(*_link, _cfg, _modes);

    // Spin up the 1 kHz gap-servo task.
    startEdmServoTask(_ctl.get());

    // Wire feed + tension (optional: only when a wire_feed: section was parsed).
    if (_feed_cfg) {
        _feed = std::make_unique<EDM::feed::WireFeed>();
        _feed->begin(_ctl.get(), *_feed_cfg);
        startWireFeedTask(_feed.get());
    }

    config_message();
}

void EdmSpindle::setState(SpindleState state, uint32_t speed) {
    if (!_ctl) {
        return;
    }
    if (state == SpindleState::Disable || speed == 0) {
        _ctl->requestStop();
    } else {
        _ctl->requestCut(static_cast<uint16_t>(speed));
    }
    _current_state = state;
    _current_speed = speed;
}

void EdmSpindle::config_message() {
    log_info(name() << " EDM gap-servo spindle");
}

void EdmSpindle::group(Configuration::HandlerBase& handler) {
    // Representative subset of the edm: schema for P1 bring-up.
    handler.item("use_sim", _use_sim);
    handler.item("report_hz", _report_hz, 1, 100);
    // A couple of servo gains, to prove the wiring through to ServoConfig.
    handler.item("servo_kp", _cfg.Kp);
    handler.item("servo_ki", _cfg.Ki);
    handler.item("servo_deadband", _cfg.deadband);
    // TODO(P4): parse the full edm: schema -- remaining ServoConfig fields and
    // the ModeTable (rough/finish/ignite ModeParams + thresholds).

    // Wire-feed hardware sub-section (parser-allocated; optional).
    handler.section("wire_feed", _feed_cfg);

    Spindle::group(handler);
}

// Configuration registration (mirrors LaserSpindle.cpp).
namespace {
    SpindleFactory::InstanceBuilder<EdmSpindle> registration("EDM");
}

}  // namespace Spindles
