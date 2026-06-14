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
#include "EDM/EdmDielTask.h"
#include "EDM/Diel/SimDielLink.h"
// TODO(bench): #include "EDM/Diel/CanDielLink.h" when the CAN dielectric path is
// wired (see the use_sim==false branch in init()).

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

    // ---- Dielectric (coolant) link (sub-project D) ----
    // Resolve the effective config: the parsed `dielectric:` section when
    // present, else the built-in defaults. Bind by const-reference (never copy):
    // EdmDielConfig derives from Configuration::Configurable, whose copy ctor is
    // deleted, so a local default is used as the fallback target of the reference.
    const EdmDielConfig  diel_defaults{};
    const EdmDielConfig& dcfg = _diel_cfg ? *_diel_cfg : diel_defaults;
    _diel_temp_setpoint_dC    = dcfg.temp_setpoint_dC;
    _diel_default_flush_level = dcfg.default_flush_level;

    // The simulator is the default and the only live path today: the real CAN
    // PSU link is still a TODO(P4) fallback above, so the spindle does not yet
    // own a CanBus to share. When use_sim is false but no CAN bus exists, fall
    // back to the simulator (mirrors the PSU-link fallback) so init() stays safe.
    // The CAN construction + single-handler fan-out is staged below behind a
    // TODO(bench) for when the PSU CAN path is wired.
    if (dcfg.use_sim) {
        auto sim       = std::make_unique<EDM::diel::SimDielLink>();
        sim->begin();
        _sim_diel = sim.get();
        _diel     = std::move(sim);
    } else {
        // TODO(bench): construct CanDielLink over the SAME CanBus the (future)
        // CanPsuLink uses, then register ONE spindle-owned RX handler that fans
        // every frame out to BOTH links:
        //     bus.onReceive([psu, diel](const CanFrame& f) {
        //         psu->onFrame(f);   // ignores non-PSU IDs
        //         diel->onFrame(f);  // ignores non-0x7xx IDs
        //     });
        // CanBus::onReceive holds exactly one handler, and both CanPsuLink::begin
        // and CanDielLink::begin self-register (each clobbering the other), so the
        // spindle must own the combined handler instead of calling either begin().
        // Until the PSU CAN path exists (TODO(P4) above), fall back to the sim so
        // the interlock + telemetry still function.
        log_info(name() << " CAN dielectric link not yet wired; using SimDielLink");
        auto sim       = std::make_unique<EDM::diel::SimDielLink>();
        sim->begin();
        _sim_diel = sim.get();
        _diel     = std::move(sim);
    }

    // Attach to the controller's cut-interlock (built from the resolved config)
    // BEFORE the servo task starts so the very first tick() sees a valid link.
    EDM::EdmController::DielInterlock interlock;
    interlock.required             = dcfg.required;
    interlock.flow_min_clpm        = static_cast<uint16_t>(dcfg.flow_min_clpm);
    interlock.level_min_pct        = static_cast<uint8_t>(dcfg.level_min_pct);
    interlock.conductivity_warn_uS = static_cast<uint16_t>(dcfg.conductivity_warn_uS);
    _ctl->attachDielectric(_diel.get(), interlock);

    // Push the configured defaults to the link once (temp setpoint, deioniser on,
    // default flush level) so the module/sim starts in a known state.
    _diel->setDiel(dielDefaults());

    // Spin up the 1 kHz gap-servo task.
    startEdmServoTask(_ctl.get());

    // Drive the in-firmware coolant simulator (setCutting + tick) from a slow
    // periodic task when a sim link is used. The real CAN module produces its
    // own telemetry, so no tick task is needed for the CAN path.
    if (_sim_diel) {
        startEdmDielTask(_sim_diel, _ctl.get());
    }

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

    // Dielectric (coolant) sub-section (parser-allocated, optional). Mirrors the
    // wire_feed pointer-section idiom: EdmDielConfig is a Configurable with its
    // own group(). A missing `dielectric:` section leaves _diel_cfg null and the
    // spindle's built-in defaults apply.
    handler.section("dielectric", _diel_cfg);

    Spindle::group(handler);
}

// Configuration registration (mirrors LaserSpindle.cpp).
namespace {
    SpindleFactory::InstanceBuilder<EdmSpindle> registration("EDM");
}

}  // namespace Spindles
