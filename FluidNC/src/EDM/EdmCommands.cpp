// FluidNC/src/EDM/EdmCommands.cpp
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Verified by the ESP32 firmware build (P4).
//
// Registers the `$EDM/*` runtime commands that the embedded WebUI
// (embedded/edm-webui/index.html) sends, and routes them to the active EDM
// spindle's EdmController / WireFeed. make_edm_commands() is called once from
// ProcessSettings.cpp::make_user_commands() (which is NOT host-built, so this
// stays out of the native googletest binary).
//
// Command match: do_command_or_setting() compares the typed key against both the
// command grblName and name (case-insensitive), splitting on '='. So registering
// name="EDM/Status" matches `$EDM/Status`, and name="EDM/Tension" with the user
// typing `$EDM/Tension=5.0` delivers value="5.0".

#include "Settings.h"  // UserCommand, anyState/notIdleOrAlarm, Error, Channel
#include "Logging.h"   // log_info, log_error

#include "EDM/EdmSpindle.h"
#include "EDM/EdmReportChannel.h"
#include "EDM/Control/EdmController.h"
#include "EDM/Feed/WireFeed.h"
#include "EDM/Servo/ModeTable.h"

#include "Spindles/Spindle.h"  // global Spindles::Spindle* spindle
#include "string_util.h"       // string_util::equal_ignore_case

#include <cstdlib>  // strtof, strtol
#include <cstring>  // std::strcmp

namespace {

// Resolve the active spindle to an EdmSpindle, or nullptr if EDM is not active.
// Mirrors the existing GCode.cpp idiom: strcmp(spindle->name(), "EDM") == 0.
Spindles::EdmSpindle* active_edm_spindle() {
    if (spindle && std::strcmp(spindle->name(), "EDM") == 0) {
        return static_cast<Spindles::EdmSpindle*>(spindle);
    }
    return nullptr;
}

EDM::EdmController* edm_controller() {
    auto* s = active_edm_spindle();
    return s ? s->controller() : nullptr;
}

EDM::feed::WireFeed* edm_wirefeed() {
    auto* s = active_edm_spindle();
    return s ? s->wireFeed() : nullptr;
}

// $EDM/Status -> full JSON dump of the latest controller snapshot.
Error edm_status(const char* value, AuthenticationLevel, Channel& out) {
    auto* ctl = edm_controller();
    if (!ctl) {
        log_error("$EDM/Status: EDM spindle not active");
        return Error::InvalidStatement;
    }
    edm_status_dump(out, ctl->snapshot(), edm_wirefeed());
    return Error::Ok;
}

// $EDM/TouchOff -> request the controller's touch-off / edge-find sequence.
// NOTE (TODO(P4)): EdmController has no standalone requestTouchOff() entry point
// today -- touch-off is an internal state reached as part of the arm/cut sequence
// (requestCut). Rather than fabricate a new control path here, this logs the
// request and returns Ok; wiring a real EdmController::requestTouchOff() (and the
// matching servo TouchOff mode without striking) is a focused follow-up.
Error edm_touchoff(const char* value, AuthenticationLevel, Channel& out) {
    auto* ctl = edm_controller();
    if (!ctl) {
        log_error("$EDM/TouchOff: EDM spindle not active");
        return Error::InvalidStatement;
    }
    log_info("$EDM/TouchOff requested (TODO(P4): EdmController::requestTouchOff not yet wired)");
    return Error::Ok;
}

// $EDM/Tension=<N> -> set the wire-feed tension setpoint in Newtons.
Error edm_tension(const char* value, AuthenticationLevel, Channel& out) {
    auto* feed = edm_wirefeed();
    if (!feed) {
        log_error("$EDM/Tension: wire feed not configured");
        return Error::InvalidStatement;
    }
    if (!value || !*value) {
        // No value: report the current setpoint.
        log_info("$EDM/Tension current setpoint = " << feed->tensionSetpointN() << " N");
        return Error::Ok;
    }
    char* end = nullptr;
    float n   = std::strtof(value, &end);
    if (end == value || n < 0.0f) {
        log_error("$EDM/Tension: invalid value '" << value << "'");
        return Error::InvalidValue;
    }
    feed->setTensionSetpointN(n);
    log_info("$EDM/Tension setpoint -> " << n << " N");
    return Error::Ok;
}

// $EDM/ResetWireFeed -> clear a latched tension collapse so the feed can re-enable.
Error edm_reset_wirefeed(const char* value, AuthenticationLevel, Channel& out) {
    auto* feed = edm_wirefeed();
    if (!feed) {
        log_error("$EDM/ResetWireFeed: wire feed not configured");
        return Error::InvalidStatement;
    }
    feed->resetCollapse();
    log_info("$EDM/ResetWireFeed: tension collapse cleared");
    return Error::Ok;
}

// $EDM/Mode=<name|S-word> -> select an EDM cutting mode. Accepts a mode name
// (Rough/Finish/Ignite/Idle) or a raw S-word; routed through requestCut() so the
// ModeTable maps it exactly like an M3 S<word> would.
Error edm_mode(const char* value, AuthenticationLevel, Channel& out) {
    auto* ctl = edm_controller();
    if (!ctl) {
        log_error("$EDM/Mode: EDM spindle not active");
        return Error::InvalidStatement;
    }
    if (!value || !*value) {
        log_error("$EDM/Mode requires a mode name or S-word");
        return Error::InvalidValue;
    }

    // Representative S-words per named mode (match ModeTable's banding used by the
    // WebUI: S>=900 Ignite, S>=100 Finish/Rough). Names are a convenience; the
    // canonical selector remains the S-word via requestCut().
    uint16_t s_word = 0;
    if (string_util::equal_ignore_case(value, "Idle")) {
        ctl->requestStop();
        log_info("$EDM/Mode -> Idle (stop)");
        return Error::Ok;
    } else if (string_util::equal_ignore_case(value, "Rough")) {
        s_word = 100;
    } else if (string_util::equal_ignore_case(value, "Finish")) {
        s_word = 300;
    } else if (string_util::equal_ignore_case(value, "Ignite")) {
        s_word = 900;
    } else {
        // Parse as a raw S-word.
        char* end = nullptr;
        long  v   = std::strtol(value, &end, 10);
        if (end == value || v < 0 || v > 65535) {
            log_error("$EDM/Mode: unknown mode/S-word '" << value << "'");
            return Error::InvalidValue;
        }
        s_word = static_cast<uint16_t>(v);
    }

    ctl->requestCut(s_word);
    log_info("$EDM/Mode -> S" << s_word);
    return Error::Ok;
}

}  // namespace

// Called once from make_user_commands() (ProcessSettings.cpp).
void make_edm_commands() {
    new UserCommand("", "EDM/Status", edm_status, anyState);
    new UserCommand("", "EDM/TouchOff", edm_touchoff, notIdleOrAlarm);
    new UserCommand("", "EDM/Tension", edm_tension, anyState);
    new UserCommand("", "EDM/ResetWireFeed", edm_reset_wirefeed, anyState);
    new UserCommand("", "EDM/Mode", edm_mode, notIdleOrAlarm);
}
