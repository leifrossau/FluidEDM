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
#include "EDM/Diel/IDielLink.h"
#include "EDM/Diel/DielProtocol.h"

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

EDM::diel::IDielLink* edm_diellink() {
    auto* s = active_edm_spindle();
    return s ? s->dielLink() : nullptr;
}

// Parse a flush level from a name or 0..3. Returns false on an unknown token.
bool parse_flush_level(const char* value, uint8_t& level_out) {
    if (string_util::equal_ignore_case(value, "Off"))    { level_out = 0; return true; }
    if (string_util::equal_ignore_case(value, "Low"))    { level_out = 1; return true; }
    if (string_util::equal_ignore_case(value, "Medium")) { level_out = 2; return true; }
    if (string_util::equal_ignore_case(value, "Med"))    { level_out = 2; return true; }
    if (string_util::equal_ignore_case(value, "High"))   { level_out = 3; return true; }
    char* end = nullptr;
    long  v   = std::strtol(value, &end, 10);
    if (end == value || v < 0 || v > 3) {
        return false;
    }
    level_out = static_cast<uint8_t>(v);
    return true;
}

// Parse an on/off token (on|off|1|0|true|false). Returns false on an unknown token.
bool parse_on_off(const char* value, bool& on_out) {
    if (string_util::equal_ignore_case(value, "on")  || string_util::equal_ignore_case(value, "true")  ||
        string_util::equal_ignore_case(value, "1"))  { on_out = true;  return true; }
    if (string_util::equal_ignore_case(value, "off") || string_util::equal_ignore_case(value, "false") ||
        string_util::equal_ignore_case(value, "0"))  { on_out = false; return true; }
    return false;
}

// $EDM/Status -> full JSON dump of the latest controller snapshot, plus a short
// human-readable dielectric (coolant) line for terminal use.
Error edm_status(const char* value, AuthenticationLevel, Channel& out) {
    auto* ctl = edm_controller();
    if (!ctl) {
        log_error("$EDM/Status: EDM spindle not active");
        return Error::InvalidStatement;
    }
    edm_status_dump(out, ctl->snapshot(), edm_wirefeed());

    // Compact coolant summary read from the live diel stats (the WebUI consumes
    // the JSON `dielectric` object above; this line is for a plain terminal).
    auto* diel = edm_diellink();
    EDM::diel::DielStats ds{};
    if (diel && diel->latestStats(ds)) {
        log_info("$EDM/Status diel: pump=" << static_cast<int>(ds.pump_on)
                 << " flush=" << static_cast<int>(ds.flush_level)
                 << " flow_clpm=" << ds.flow_clpm
                 << " temp_dC=" << ds.temp_dC
                 << " level_pct=" << static_cast<int>(ds.level_pct)
                 << " filter_pct=" << static_cast<int>(ds.filter_pct));
    } else {
        log_info("$EDM/Status diel: not present");
    }
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

// $EDM/Flush=Off|Low|Medium|High (or 0|1|2|3) -> set the dielectric flush level.
// Carries the configured temp setpoint + deioniser default so the level change
// does not clobber the other SetDiel fields. The pump follows the level: any
// non-Off level turns it on, Off turns it off ($EDM/Pump independently overrides).
Error edm_flush(const char* value, AuthenticationLevel, Channel& out) {
    auto* s = active_edm_spindle();
    if (!s) {
        log_error("$EDM/Flush: EDM spindle not active");
        return Error::InvalidStatement;
    }
    auto* diel = s->dielLink();
    if (!diel) {
        log_error("$EDM/Flush: dielectric link not available");
        return Error::InvalidStatement;
    }
    if (!value || !*value) {
        log_error("$EDM/Flush requires Off|Low|Medium|High or 0..3");
        return Error::InvalidValue;
    }
    uint8_t level = 0;
    if (!parse_flush_level(value, level)) {
        log_error("$EDM/Flush: invalid level '" << value << "'");
        return Error::InvalidValue;
    }
    EDM::diel::SetDiel sd = s->dielDefaults();
    sd.flush_level = level;
    sd.pump_on     = (level > 0) ? 1 : 0;
    diel->setDiel(sd);
    log_info("$EDM/Flush -> level " << static_cast<int>(level));
    return Error::Ok;
}

// $EDM/Pump=on|off (or 1|0) -> turn the dielectric pump on/off. Carries the
// configured defaults so the temp setpoint / deioniser / flush level are
// preserved; only pump_on changes.
Error edm_pump(const char* value, AuthenticationLevel, Channel& out) {
    auto* s = active_edm_spindle();
    if (!s) {
        log_error("$EDM/Pump: EDM spindle not active");
        return Error::InvalidStatement;
    }
    auto* diel = s->dielLink();
    if (!diel) {
        log_error("$EDM/Pump: dielectric link not available");
        return Error::InvalidStatement;
    }
    if (!value || !*value) {
        log_error("$EDM/Pump requires on|off or 1|0");
        return Error::InvalidValue;
    }
    bool on = false;
    if (!parse_on_off(value, on)) {
        log_error("$EDM/Pump: invalid value '" << value << "'");
        return Error::InvalidValue;
    }
    EDM::diel::SetDiel sd = s->dielDefaults();
    sd.pump_on = on ? 1 : 0;
    diel->setDiel(sd);
    log_info("$EDM/Pump -> " << (on ? "on" : "off"));
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
    new UserCommand("", "EDM/Flush", edm_flush, anyState);
    new UserCommand("", "EDM/Pump", edm_pump, anyState);
}
