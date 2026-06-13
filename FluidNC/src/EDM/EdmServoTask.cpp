// FluidNC/src/EDM/EdmServoTask.cpp
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Uses FreeRTOS task APIs; verified by the ESP32 firmware build (P4).
//
// Periodic-task idiom mirrors Protocol.cpp (start_polling): stack, priority and
// SUPPORT_TASK_CORE come from there; add_watchdog_to_task() registers the task
// with the hardware watchdog exactly as polling_loop() does.

#include "EdmServoTask.h"

#include "EDM/Control/EdmController.h"

#include "Config.h"            // SUPPORT_TASK_CORE, Platform.h, FreeRTOS, get_ms()
#include "Driver/watchdog.h"   // add_watchdog_to_task

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static void edmServoLoop(void* arg) {
    add_watchdog_to_task();

    auto* ctl = static_cast<EDM::EdmController*>(arg);
    // ~1 kHz: vTaskDelay(1) yields one RTOS tick (1 ms at the default tick rate).
    // get_ms() is the FluidNC-portable millisecond clock (see NutsBolts.h).
    for (;;) {
        ctl->tick(get_ms());
        vTaskDelay(1);
    }
}

void startEdmServoTask(EDM::EdmController* ctl) {
    xTaskCreatePinnedToCore(edmServoLoop,       // task
                            "edm_servo",         // name for task
                            8192,                // size of task stack
                            ctl,                 // parameters
                            2,                   // priority
                            nullptr,             // task handle (not retained)
                            SUPPORT_TASK_CORE);  // core
}
