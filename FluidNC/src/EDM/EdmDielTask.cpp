// FluidNC/src/EDM/EdmDielTask.cpp
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Uses FreeRTOS task APIs; verified by the ESP32 firmware build (P4).
//
// Periodic-task idiom mirrors EdmWireFeedTask.cpp: stack, priority and
// SUPPORT_TASK_CORE come from there; add_watchdog_to_task() registers the task
// with the hardware watchdog. Runs at the dielectric module's ~10 Hz stats rate
// (vTaskDelay(100)) — fast enough for the WebUI's coolant panel, slow enough to
// leave the 1 kHz gap servo undisturbed.

#include "EdmDielTask.h"

#include "EDM/Control/EdmController.h"   // EdmController, EdmState
#include "EDM/Diel/SimDielLink.h"

#include "Config.h"            // SUPPORT_TASK_CORE, get_ms()
#include "Driver/watchdog.h"   // add_watchdog_to_task

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

// Context handed to the FreeRTOS task. Heap-allocated and never freed: the task
// runs for the firmware lifetime, so the allocation lives as long as the program
// (matches the "task owns no memory it must free" idiom of the sibling tasks).
struct DielTaskCtx {
    EDM::diel::SimDielLink* sim;
    EDM::EdmController*     ctl;
};

constexpr uint32_t kPeriodMs = 100;                       // ~10 Hz
constexpr float    kDtSec    = kPeriodMs / 1000.0f;       // sim::tick ignores dt today

// True while the controller is in an active-cut phase (pump should run). Mirrors
// the controller's own dielectric interlock window (Cutting/Hold/BreakRelief)
// plus TouchOff, which is the gap-approach part of the same cut.
bool isCutting(EDM::EdmState s) {
    return s == EDM::EdmState::TouchOff || s == EDM::EdmState::Cutting ||
           s == EDM::EdmState::Hold     || s == EDM::EdmState::BreakRelief;
}

void edmDielLoop(void* arg) {
    add_watchdog_to_task();

    auto* ctx = static_cast<DielTaskCtx*>(arg);
    for (;;) {
        // TODO(bench): ctl->state() is written by the 1 kHz servo task on the
        // other core; this is a benign racy read of a single-byte enum used only
        // to drive the simulator's pump model (no control decision is made here).
        ctx->sim->setCutting(isCutting(ctx->ctl->state()));
        ctx->sim->tick(kDtSec);
        vTaskDelay(kPeriodMs);
    }
}

}  // namespace

void startEdmDielTask(EDM::diel::SimDielLink* sim, EDM::EdmController* ctl) {
    auto* ctx = new DielTaskCtx{ sim, ctl };
    xTaskCreatePinnedToCore(edmDielLoop,         // task
                            "edm_diel",          // name for task
                            4096,                // size of task stack
                            ctx,                 // parameters
                            2,                   // priority
                            nullptr,             // task handle (not retained)
                            SUPPORT_TASK_CORE);  // core
}
