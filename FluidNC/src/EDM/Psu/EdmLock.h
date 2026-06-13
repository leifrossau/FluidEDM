// FluidNC/src/EDM/Psu/EdmLock.h
#pragma once
//
// Portable critical-section lock used to protect CanPsuLink shared state that is
// touched from two cores on target (CAN-RX task vs. 1 kHz servo task).
//
// On ESP32 (ESP-IDF / Arduino) it is a FreeRTOS portMUX spinlock; on the native
// host test build (which defines neither ESP_PLATFORM nor ARDUINO and has no
// FreeRTOS) it compiles to a no-op so the single-threaded googletests build and
// behave exactly as before. The rest of the EDM ESP32-only sources rely on
// build_src_filter to stay out of the host build, but EdmLock.h is pulled in
// transitively by PsuLink.h, which IS host-built -- hence the real #if guard
// here rather than a filter exclusion.
//
// Detection macro rationale: ESP_PLATFORM is defined by the ESP-IDF build system
// and ARDUINO by the Arduino core; the FluidNC esp32 envs build under ESP-IDF, so
// ESP_PLATFORM is the load-bearing guard. ARDUINO is included for belt-and-braces.

#if defined(ESP_PLATFORM) || defined(ARDUINO)
  #include "freertos/FreeRTOS.h"  // portMUX_TYPE, portENTER/EXIT_CRITICAL, portMUX_INITIALIZER_UNLOCKED
  namespace EDM { namespace psu {
    struct EdmLock {
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
        void enter() { portENTER_CRITICAL(&mux); }
        void exit()  { portEXIT_CRITICAL(&mux); }
    };
    struct EdmLockGuard {
        EdmLock& l;
        explicit EdmLockGuard(EdmLock& x) : l(x) { l.enter(); }
        ~EdmLockGuard() { l.exit(); }
        EdmLockGuard(const EdmLockGuard&)            = delete;
        EdmLockGuard& operator=(const EdmLockGuard&) = delete;
    };
  }}  // namespace EDM::psu
#else
  // Native host test build: single-threaded, no FreeRTOS -- the lock is a no-op so
  // CanPsuLink behaves identically to the pre-P4 code under googletest.
  namespace EDM { namespace psu {
    struct EdmLock {
        void enter() {}
        void exit()  {}
    };
    struct EdmLockGuard {
        explicit EdmLockGuard(EdmLock&) {}
        EdmLockGuard(const EdmLockGuard&)            = delete;
        EdmLockGuard& operator=(const EdmLockGuard&) = delete;
    };
  }}  // namespace EDM::psu
#endif
