// FluidNC/src/EDM/EdmDielTask.h
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Verified by the ESP32 firmware build in a later phase (P4).
//
// Starts a FreeRTOS task that drives the in-firmware dielectric (coolant)
// simulator. The real CAN dielectric module produces its own telemetry, so this
// task only exists for the SimDielLink bring-up path.
#pragma once

namespace EDM {
class EdmController;
namespace diel { class SimDielLink; }
}  // namespace EDM

// Launch the periodic dielectric-simulator task. Each tick it mirrors the
// controller's cut state onto the sim (setCutting) and advances the coolant
// model (tick). Neither pointer is owned by the task; both must outlive it (the
// EdmSpindle holds them for the firmware lifetime).
void startEdmDielTask(EDM::diel::SimDielLink* sim, EDM::EdmController* ctl);
