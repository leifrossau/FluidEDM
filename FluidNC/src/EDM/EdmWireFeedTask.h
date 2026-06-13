// FluidNC/src/EDM/EdmWireFeedTask.h
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Verified by the ESP32 firmware build in a later phase (P4).
//
// Starts a FreeRTOS task that drives an EDM::feed::WireFeed at ~83 Hz.
#pragma once

namespace EDM { namespace feed {
class WireFeed;
}}  // namespace EDM::feed

// Launch the periodic wire-feed task. The task owns no memory; `wf` must outlive
// the task (the EdmSpindle holds it for the firmware lifetime).
void startWireFeedTask(EDM::feed::WireFeed* wf);
