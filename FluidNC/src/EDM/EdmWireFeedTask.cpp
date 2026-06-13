// FluidNC/src/EDM/EdmWireFeedTask.cpp
//
// ESP32-COUPLED WIRING (NOT host-testable; NOT in the native test build_src_filter).
// Uses FreeRTOS task APIs; verified by the ESP32 firmware build (P4).
//
// Periodic-task idiom mirrors EdmServoTask.cpp: stack, priority and
// SUPPORT_TASK_CORE come from there; add_watchdog_to_task() registers the task
// with the hardware watchdog. Runs slower than the 1 kHz gap servo: the HX711
// settles at ~80 sps, so a ~83 Hz tick (vTaskDelay(12)) matches the sensor.

#include "EdmWireFeedTask.h"

#include "EDM/Feed/WireFeed.h"

#include "Config.h"            // SUPPORT_TASK_CORE, get_ms()
#include "Driver/watchdog.h"   // add_watchdog_to_task

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static void wireFeedLoop(void* arg) {
    add_watchdog_to_task();

    auto* wf = static_cast<EDM::feed::WireFeed*>(arg);
    // ~83 Hz: vTaskDelay(12) yields 12 RTOS ticks (12 ms at the default tick rate),
    // matching the HX711 conversion rate so dataReady() is usually true each tick.
    for (;;) {
        wf->tick(get_ms());
        vTaskDelay(12);
    }
}

void startWireFeedTask(EDM::feed::WireFeed* wf) {
    xTaskCreatePinnedToCore(wireFeedLoop,       // task
                            "wire_feed",         // name for task
                            4096,                // size of task stack
                            wf,                  // parameters
                            2,                   // priority
                            nullptr,             // task handle (not retained)
                            SUPPORT_TASK_CORE);  // core
}
