// FluidNC/src/EDM/EdmReportChannel.h
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Uses the FluidNC Channel / JSONencoder APIs; verified by the ESP32 build (P4).
//
// Emits an EDM::EdmReport over a Channel as an encapsulated [MSG:JSON:...] line.
#pragma once

#include "EDM/Servo/EdmReport.h"

class Channel;

// Compact periodic status line (the key servo/gap/pulse fields).
void report_edm_stats(Channel& channel, const EDM::EdmReport& r);

// Full dump of every EdmReport field (for $EDM/Status, wired in P4).
void edm_status_dump(Channel& channel, const EDM::EdmReport& r);

// TODO(P4): hook report_edm_stats into Channel::autoReport() at the configured
// report_hz, and register a "$EDM/Status" UserCommand that calls
// edm_status_dump(). Both touch Report.cpp / Channel.cpp / SettingsDefinitions,
// which are intentionally NOT modified in this phase.
