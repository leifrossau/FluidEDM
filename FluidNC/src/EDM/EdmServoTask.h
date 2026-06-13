// FluidNC/src/EDM/EdmServoTask.h
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Verified by the ESP32 firmware build in a later phase (P4).
//
// Starts a FreeRTOS task that drives an EDM::EdmController at ~1 kHz.
#pragma once

namespace EDM {
class EdmController;
}

// Launch the periodic gap-servo task. The task owns no memory; `ctl` must
// outlive the task (the EdmSpindle holds it for the firmware lifetime).
void startEdmServoTask(EDM::EdmController* ctl);
