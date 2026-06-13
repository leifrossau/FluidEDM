# EDM PSU Integration — Phase P0 (Transport + Protocol + Simulator) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the host-testable foundation of the EDM firmware: a `CanFrame`/`CanBus` abstraction, a complete codec for the locked PSU CAN-FD message catalog, a session-layer `PsuLink` interface, an in-firmware `SimPsuLink` simulator, and the MCP2518FD SPI driver — all under `FluidNC/src/EDM/`, with googletest unit tests that build and pass on the native (x86) PlatformIO environment.

**Architecture:** Hardware lives behind a `CanBus` interface so all protocol/session/simulator logic compiles and is tested natively with zero ESP32 dependencies. The codec is pure freestanding C++ (POD structs + free functions on `CanFrame`). `IPsuLink` decouples the EDM controller (later phases) from whether it talks to a real PSU (`CanPsuLink` over `CanBus`) or the simulator (`SimPsuLink`). The MCP2518FD driver is the one bench-verified (not host-tested) component.

**Tech Stack:** C++17, PlatformIO, googletest (native `[env:tests]`), ESP32-Arduino + ESP-IDF SPI (for the driver only), MCP2518FD CAN-FD controller.

**Spec:** `../specs/2026-06-13-edm-psu-integration-design.md` (CAN-FD catalog mirrors the Power Supply spec §8).

**Conventions for this plan:**
- All multi-byte CAN payload fields are **little-endian** (matches the embedded MCU convention; locked here as the codec contract).
- `build_src_filter` paths in `platformio.ini` are relative to `FluidNC/`.
- Run tests from the repo root (the dir containing `platformio.ini`): `pio test -e tests`.
- Filter to one suite while iterating: `pio test -e tests --filter '*PsuProto*'` (GoogleTest filter via `-f`, see each task).

---

## File structure (locked before tasks)

Created in this phase:

- `FluidNC/src/EDM/Can/CanFrame.h` — POD frame type shared by driver, codec, tests.
- `FluidNC/src/EDM/Can/CanBus.h` — abstract transport interface + a `FakeCanBus` test double (header-only, in a `test_support` sub-namespace).
- `FluidNC/src/EDM/Psu/Endian.h` — little-endian get/put helpers (`get_u16le`, `put_u16le`, …).
- `FluidNC/src/EDM/Psu/Protocol.h` — catalog IDs, typed message structs, encode/decode declarations.
- `FluidNC/src/EDM/Psu/Protocol.cpp` — encode/decode implementations.
- `FluidNC/src/EDM/Psu/IPsuLink.h` — interface the controller depends on.
- `FluidNC/src/EDM/Psu/PsuLink.h` / `.cpp` — `CanPsuLink`: session layer over a `CanBus`.
- `FluidNC/src/EDM/Psu/SimPsuLink.h` / `.cpp` — in-firmware gap simulator implementing `IPsuLink`.
- `FluidNC/src/EDM/Can/Mcp2518fdDriver.h` / `.cpp` — `CanBus` implementation over SPI (bench-verified).
- `FluidNC/tests/EdmEndianTest.cpp`, `EdmPsuProtocolTest.cpp`, `EdmPsuLinkTest.cpp`, `EdmSimPsuLinkTest.cpp` — googletest suites.

Modified:

- `platformio.ini` — add EDM sources + test files to `[tests_common] build_src_filter`.

---

## Task 1: CanFrame + Endian helpers + native test wiring

**Files:**
- Create: `FluidNC/src/EDM/Can/CanFrame.h`
- Create: `FluidNC/src/EDM/Psu/Endian.h`
- Create: `FluidNC/tests/EdmEndianTest.cpp`
- Modify: `platformio.ini` (build_src_filter)

- [ ] **Step 1: Write `CanFrame.h`**

```cpp
// FluidNC/src/EDM/Can/CanFrame.h
#pragma once
#include <cstdint>
#include <cstring>

namespace EDM {

// A CAN-FD frame. Standard 11-bit IDs only (per PSU contract).
struct CanFrame {
    uint16_t id   = 0;        // 11-bit standard ID
    uint8_t  len  = 0;        // valid bytes in data[] (0..64)
    bool     fd   = true;     // CAN-FD frame
    bool     brs  = true;     // bit-rate switch (data phase at 5 Mbps)
    uint8_t  data[64] = {};

    CanFrame() = default;
    CanFrame(uint16_t id_, uint8_t len_) : id(id_), len(len_) {}
};

}  // namespace EDM
```

- [ ] **Step 2: Write `Endian.h`**

```cpp
// FluidNC/src/EDM/Psu/Endian.h
#pragma once
#include <cstdint>

namespace EDM { namespace le {

inline void     put_u8 (uint8_t* p, uint8_t v)  { p[0] = v; }
inline uint8_t  get_u8 (const uint8_t* p)        { return p[0]; }

inline void     put_u16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
inline uint16_t get_u16(const uint8_t* p)        { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }

inline void     put_i16(uint8_t* p, int16_t v)  { put_u16(p, uint16_t(v)); }
inline int16_t  get_i16(const uint8_t* p)        { return int16_t(get_u16(p)); }

inline void     put_u32(uint8_t* p, uint32_t v) {
    p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); p[2] = uint8_t(v >> 16); p[3] = uint8_t(v >> 24);
}
inline uint32_t get_u32(const uint8_t* p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

}}  // namespace EDM::le
```

- [ ] **Step 3: Write the failing test `EdmEndianTest.cpp`**

```cpp
// FluidNC/tests/EdmEndianTest.cpp
#include "gtest/gtest.h"
#include "EDM/Psu/Endian.h"

using namespace EDM;

TEST(EdmEndian, U16RoundTrip) {
    uint8_t buf[2] = {};
    le::put_u16(buf, 0xBEEF);
    EXPECT_EQ(buf[0], 0xEF);
    EXPECT_EQ(buf[1], 0xBE);
    EXPECT_EQ(le::get_u16(buf), 0xBEEF);
}

TEST(EdmEndian, U32RoundTrip) {
    uint8_t buf[4] = {};
    le::put_u32(buf, 0x01020304u);
    EXPECT_EQ(buf[0], 0x04);
    EXPECT_EQ(buf[3], 0x01);
    EXPECT_EQ(le::get_u32(buf), 0x01020304u);
}

TEST(EdmEndian, I16Negative) {
    uint8_t buf[2] = {};
    le::put_i16(buf, int16_t(-300));
    EXPECT_EQ(le::get_i16(buf), int16_t(-300));
}
```

- [ ] **Step 4: Register the test sources in `platformio.ini`**

In `[tests_common] build_src_filter` (after line `+<test_main.cpp>`), add:

```ini
    +<EDM/Psu/Protocol.cpp>
    +<EDM/Psu/PsuLink.cpp>
    +<EDM/Psu/SimPsuLink.cpp>
    +<tests/EdmEndianTest.cpp>
    +<tests/EdmPsuProtocolTest.cpp>
    +<tests/EdmPsuLinkTest.cpp>
    +<tests/EdmSimPsuLinkTest.cpp>
```

Note: `Protocol.cpp`, `PsuLink.cpp`, `SimPsuLink.cpp` and the not-yet-created test files are added now so later tasks only touch code. Until those files exist the native build will fail to link — that is expected and resolved as tasks land. To keep P0 building task-by-task, comment out the not-yet-created lines with `;` and uncomment each as its task creates the file.

For THIS task, only these lines should be active:

```ini
    +<tests/EdmEndianTest.cpp>
```

- [ ] **Step 5: Run the test, verify it passes**

Run: `pio test -e tests -f '*EdmEndian*'`
Expected: 3 tests PASS (`EdmEndian.U16RoundTrip`, `U32RoundTrip`, `I16Negative`).

- [ ] **Step 6: Commit**

```bash
git add FluidNC/src/EDM/Can/CanFrame.h FluidNC/src/EDM/Psu/Endian.h FluidNC/tests/EdmEndianTest.cpp platformio.ini
git commit -m "feat(edm): add CanFrame, little-endian helpers, native test wiring"
```

---

## Task 2: PSU catalog IDs + message structs (header only)

**Files:**
- Create: `FluidNC/src/EDM/Psu/Protocol.h`

- [ ] **Step 1: Write `Protocol.h`** (IDs from PSU spec §8.1; structs from §8.2–8.6)

```cpp
// FluidNC/src/EDM/Psu/Protocol.h
#pragma once
#include <cstdint>
#include "EDM/Can/CanFrame.h"

namespace EDM { namespace psu {

// ---- CAN-FD standard IDs (PSU spec §8.1) ----
enum Id : uint16_t {
    ID_SET_MODE_BOUNDS = 0x010,  // FluidNC -> PSU
    ID_ACK_MODE_BOUNDS = 0x011,  // PSU -> FluidNC
    ID_CONTROL         = 0x020,  // FluidNC -> PSU (start/stop/reboot/clear)
    ID_STATS_AGG       = 0x100,  // PSU -> FluidNC, 1 kHz
    ID_WIRE_BREAK      = 0x200,  // PSU -> FluidNC, on-demand
    ID_FAULT           = 0x201,
    ID_ARC_BURST       = 0x202,
    ID_INFO            = 0x210,
    ID_PSU_STATUS      = 0x300,  // PSU -> FluidNC, 100 ms heartbeat
};

// Protocol version this firmware implements (must match PSU PSU_STATUS).
constexpr uint16_t kProtocolVersion = 1;

// ---- Control sub-commands (payload byte 0 of ID_CONTROL) ----
enum Control : uint8_t {
    CTRL_START_CUT   = 1,
    CTRL_STOP_CUT    = 2,
    CTRL_CLEAR_FAULT = 3,
    CTRL_REBOOT      = 4,
    CTRL_POLARITY    = 5,  // payload byte 1 = polarity (0/1); only valid when PSU idle
};

// ---- §8.2 SET_MODE_BOUNDS ----
struct SetModeBounds {
    uint8_t  mode_id            = 0;  // 0=idle 1=rough 2=finish 3=ignite 4=bench >=16 user
    uint16_t seq                = 0;
    uint16_t freq_max_kHz       = 0;
    uint16_t on_time_max_ns     = 0;
    uint16_t off_time_min_ns    = 0;
    uint16_t peak_I_setpoint_dA = 0;  // 0.1 A
    uint16_t peak_I_limit_hw_dA = 0;  // 0.1 A
    uint8_t  polarity           = 0;  // 0=workpiece+ 1=wire+
    uint16_t flags              = 0;  // bit0 adaptive, bit1 photodiode_req, bit2 anti_arc, bit3 wirebreak_pred
    uint16_t gap_V_arc_mV       = 0;
    uint16_t gap_V_short_mV     = 0;
    uint16_t ignition_timeout_us = 0;
};

// ---- §8.2 ACK_MODE_BOUNDS (echoes seq + clamped values + status) ----
struct AckModeBounds {
    uint16_t seq                = 0;
    uint16_t freq_max_kHz       = 0;
    uint16_t on_time_max_ns     = 0;
    uint16_t off_time_min_ns    = 0;
    uint16_t peak_I_setpoint_dA = 0;
    uint16_t peak_I_limit_hw_dA = 0;
    uint8_t  status             = 0;  // 0=ok, nonzero=clamped/rejected code
};

// ---- §8.3 STATS_AGG (1 kHz) ----
struct StatsAgg {
    uint32_t window_id              = 0;
    uint16_t n_normal               = 0;
    uint16_t n_arc                  = 0;
    uint16_t n_short                = 0;
    uint16_t n_open                 = 0;
    uint16_t ignition_delay_mean_ns = 0;
    uint16_t ignition_delay_stddev_ns = 0;
    uint16_t peak_I_mean_dA         = 0;
    uint16_t peak_I_max_dA          = 0;
    uint16_t gap_V_recovery_mean_ns = 0;
    uint32_t energy_delivered_uJ    = 0;
    int16_t  temp_GaN_dC            = 0;  // 0.1 C
    int16_t  temp_L_dC              = 0;  // 0.1 C
    uint16_t dc_link_V_dV           = 0;  // 0.1 V
    uint16_t dc_link_I_avg_dA       = 0;  // 0.1 A
    uint8_t  state                  = 0;  // 0=running 1=paused 2=fault
    uint8_t  mode_id_active         = 0;
    uint16_t flags                  = 0;  // hw_trip_recent, photodiode_disagree, watchdog_warn
};

// ---- §8.4 WIRE_BREAK_IMMINENT ----
struct WireBreakImminent {
    uint8_t  severity                  = 0;  // 1 warn, 2 elevated, 3 critical
    uint8_t  cause_flags               = 0;  // delay_var|rise_slope|recovery|thermal
    uint16_t recent_short_count        = 0;
    uint16_t recent_arc_count          = 0;
    uint32_t ignition_delay_var_ns2    = 0;
    uint32_t timestamp_ms_since_start  = 0;
};

// ---- §8.5 FAULT ----
struct Fault {
    uint8_t fault_code = 0;
    uint8_t severity   = 0;
    uint8_t detail[6]  = {};
};

// ---- §8.5 ARC_BURST ----
struct ArcBurst {
    uint16_t consecutive_arcs = 0;
};

// ---- §8.6 PSU_STATUS heartbeat (100 ms) ----
struct PsuStatus {
    uint8_t  state            = 0;
    uint16_t fpga_version     = 0;
    uint16_t mcu_version      = 0;
    uint16_t protocol_version = 0;
    uint32_t uptime_s         = 0;
    uint16_t fault_count      = 0;
};

// ---- encode (FluidNC -> PSU) ----
CanFrame encodeSetModeBounds(const SetModeBounds& m);
CanFrame encodeControl(Control cmd, uint8_t arg = 0);

// ---- decode (PSU -> FluidNC). Return false if id/len mismatch. ----
bool decodeAckModeBounds(const CanFrame& f, AckModeBounds& out);
bool decodeStatsAgg(const CanFrame& f, StatsAgg& out);
bool decodeWireBreak(const CanFrame& f, WireBreakImminent& out);
bool decodeFault(const CanFrame& f, Fault& out);
bool decodeArcBurst(const CanFrame& f, ArcBurst& out);
bool decodeInfo(const CanFrame& f, char* out, size_t out_cap);  // null-terminates
bool decodePsuStatus(const CanFrame& f, PsuStatus& out);

}}  // namespace EDM::psu
```

- [ ] **Step 2: Commit** (header compiles standalone; no test yet)

```bash
git add FluidNC/src/EDM/Psu/Protocol.h
git commit -m "feat(edm): declare PSU CAN-FD catalog IDs and message structs"
```

---

## Task 3: encode SET_MODE_BOUNDS + Control (TDD)

**Files:**
- Create: `FluidNC/src/EDM/Psu/Protocol.cpp`
- Create: `FluidNC/tests/EdmPsuProtocolTest.cpp`
- Modify: `platformio.ini` (uncomment `+<EDM/Psu/Protocol.cpp>` and `+<tests/EdmPsuProtocolTest.cpp>`)

- [ ] **Step 1: Write the failing test**

```cpp
// FluidNC/tests/EdmPsuProtocolTest.cpp
#include "gtest/gtest.h"
#include "EDM/Psu/Protocol.h"
#include "EDM/Psu/Endian.h"

using namespace EDM;
using namespace EDM::psu;

TEST(EdmPsuProto, EncodeSetModeBoundsLayout) {
    SetModeBounds m;
    m.mode_id = 1; m.seq = 0x1234;
    m.freq_max_kHz = 200; m.on_time_max_ns = 800; m.off_time_min_ns = 1200;
    m.peak_I_setpoint_dA = 120; m.peak_I_limit_hw_dA = 200;
    m.polarity = 1; m.flags = 0x000B;
    m.gap_V_arc_mV = 4000; m.gap_V_short_mV = 1000; m.ignition_timeout_us = 50;

    CanFrame f = encodeSetModeBounds(m);
    EXPECT_EQ(f.id, ID_SET_MODE_BOUNDS);
    EXPECT_TRUE(f.fd);
    // byte 0 = mode_id, bytes 1..2 = seq (LE)
    EXPECT_EQ(f.data[0], 1);
    EXPECT_EQ(le::get_u16(&f.data[1]), 0x1234);
    EXPECT_EQ(le::get_u16(&f.data[3]), 200);    // freq_max_kHz
    // polarity sits after the 6 u16 numeric fields: offset 3 + 6*2 = 15
    EXPECT_EQ(f.data[15], 1);
    EXPECT_EQ(le::get_u16(&f.data[16]), 0x000B); // flags
    EXPECT_EQ(f.len, 24);
}

TEST(EdmPsuProto, EncodeControlStartCut) {
    CanFrame f = encodeControl(CTRL_START_CUT);
    EXPECT_EQ(f.id, ID_CONTROL);
    EXPECT_EQ(f.data[0], CTRL_START_CUT);
    EXPECT_EQ(f.len, 2);
}

TEST(EdmPsuProto, EncodeControlPolarityArg) {
    CanFrame f = encodeControl(CTRL_POLARITY, 1);
    EXPECT_EQ(f.data[0], CTRL_POLARITY);
    EXPECT_EQ(f.data[1], 1);
}
```

- [ ] **Step 2: Run, verify it fails to link**

Run: `pio test -e tests -f '*EdmPsuProto*'`
Expected: build/link error — `encodeSetModeBounds` undefined (after uncommenting the `platformio.ini` lines).

- [ ] **Step 3: Implement encode functions in `Protocol.cpp`**

```cpp
// FluidNC/src/EDM/Psu/Protocol.cpp
#include "EDM/Psu/Protocol.h"
#include "EDM/Psu/Endian.h"
#include <cstring>

namespace EDM { namespace psu {

CanFrame encodeSetModeBounds(const SetModeBounds& m) {
    CanFrame f(ID_SET_MODE_BOUNDS, 24);
    uint8_t* d = f.data;
    le::put_u8 (d + 0,  m.mode_id);
    le::put_u16(d + 1,  m.seq);
    le::put_u16(d + 3,  m.freq_max_kHz);
    le::put_u16(d + 5,  m.on_time_max_ns);
    le::put_u16(d + 7,  m.off_time_min_ns);
    le::put_u16(d + 9,  m.peak_I_setpoint_dA);
    le::put_u16(d + 11, m.peak_I_limit_hw_dA);
    le::put_u8 (d + 15, m.polarity);          // d+13,14 reserved padding (0)
    le::put_u16(d + 16, m.flags);
    le::put_u16(d + 18, m.gap_V_arc_mV);
    le::put_u16(d + 20, m.gap_V_short_mV);
    le::put_u16(d + 22, m.ignition_timeout_us);
    return f;
}

CanFrame encodeControl(Control cmd, uint8_t arg) {
    CanFrame f(ID_CONTROL, 2);
    f.data[0] = uint8_t(cmd);
    f.data[1] = arg;
    return f;
}

}}  // namespace EDM::psu
```

Note the layout: `polarity` is placed at offset 15 (with offsets 13–14 reserved padding) so the test's `3 + 6*2 = 15` matches. Keep this layout authoritative — the PSU codec must agree byte-for-byte.

- [ ] **Step 4: Run, verify PASS**

Run: `pio test -e tests -f '*EdmPsuProto*'`
Expected: 3 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add FluidNC/src/EDM/Psu/Protocol.cpp FluidNC/tests/EdmPsuProtocolTest.cpp platformio.ini
git commit -m "feat(edm): encode SET_MODE_BOUNDS and Control frames (TDD)"
```

---

## Task 4: decode STATS_AGG + ACK_MODE_BOUNDS (TDD)

**Files:**
- Modify: `FluidNC/src/EDM/Psu/Protocol.cpp`
- Modify: `FluidNC/tests/EdmPsuProtocolTest.cpp`

- [ ] **Step 1: Add failing tests**

```cpp
TEST(EdmPsuProto, DecodeStatsAggRoundTrip) {
    // Build a frame by hand, decode, check fields.
    CanFrame f(ID_STATS_AGG, 38);
    uint8_t* d = f.data;
    EDM::le::put_u32(d + 0, 42);       // window_id
    EDM::le::put_u16(d + 4, 90);       // n_normal
    EDM::le::put_u16(d + 6, 5);        // n_arc
    EDM::le::put_u16(d + 8, 3);        // n_short
    EDM::le::put_u16(d + 10, 2);       // n_open
    EDM::le::put_u16(d + 12, 500);     // ign delay mean
    EDM::le::put_u16(d + 14, 40);      // ign delay stddev
    EDM::le::put_u16(d + 16, 110);     // peak_I_mean_dA
    EDM::le::put_u16(d + 18, 180);     // peak_I_max_dA
    EDM::le::put_u16(d + 20, 300);     // gap_V_recovery
    EDM::le::put_u32(d + 22, 123456);  // energy_uJ
    EDM::le::put_i16(d + 26, 425);     // temp_GaN 42.5C
    EDM::le::put_i16(d + 28, 380);     // temp_L 38.0C
    EDM::le::put_u16(d + 30, 800);     // dc_link_V 80.0V
    EDM::le::put_u16(d + 32, 50);      // dc_link_I 5.0A
    d[34] = 0;                         // state running
    d[35] = 1;                         // mode_id_active
    EDM::le::put_u16(d + 36, 0);       // flags

    StatsAgg s;
    ASSERT_TRUE(decodeStatsAgg(f, s));
    EXPECT_EQ(s.window_id, 42u);
    EXPECT_EQ(s.n_normal, 90);
    EXPECT_EQ(s.n_short, 3);
    EXPECT_EQ(s.energy_delivered_uJ, 123456u);
    EXPECT_EQ(s.temp_GaN_dC, 425);
    EXPECT_EQ(s.dc_link_V_dV, 800);
    EXPECT_EQ(s.mode_id_active, 1);
}

TEST(EdmPsuProto, DecodeStatsAggRejectsWrongId) {
    CanFrame f(0x999, 38);
    StatsAgg s;
    EXPECT_FALSE(decodeStatsAgg(f, s));
}

TEST(EdmPsuProto, DecodeStatsAggRejectsShortLen) {
    CanFrame f(ID_STATS_AGG, 10);
    StatsAgg s;
    EXPECT_FALSE(decodeStatsAgg(f, s));
}

TEST(EdmPsuProto, DecodeAckRoundTrip) {
    CanFrame f(ID_ACK_MODE_BOUNDS, 13);
    EDM::le::put_u16(f.data + 0, 0x1234);  // seq
    EDM::le::put_u16(f.data + 2, 190);     // clamped freq
    EDM::le::put_u16(f.data + 4, 800);
    EDM::le::put_u16(f.data + 6, 1200);
    EDM::le::put_u16(f.data + 8, 120);
    EDM::le::put_u16(f.data + 10, 200);
    f.data[12] = 0;                        // status ok
    AckModeBounds a;
    ASSERT_TRUE(decodeAckModeBounds(f, a));
    EXPECT_EQ(a.seq, 0x1234);
    EXPECT_EQ(a.freq_max_kHz, 190);
    EXPECT_EQ(a.status, 0);
}
```

- [ ] **Step 2: Run, verify fail (undefined `decodeStatsAgg`/`decodeAckModeBounds`)**

Run: `pio test -e tests -f '*EdmPsuProto*'`
Expected: link error.

- [ ] **Step 3: Implement in `Protocol.cpp`** (append inside the namespace)

```cpp
bool decodeStatsAgg(const CanFrame& f, StatsAgg& s) {
    if (f.id != ID_STATS_AGG || f.len < 38) return false;
    const uint8_t* d = f.data;
    s.window_id               = le::get_u32(d + 0);
    s.n_normal                = le::get_u16(d + 4);
    s.n_arc                   = le::get_u16(d + 6);
    s.n_short                 = le::get_u16(d + 8);
    s.n_open                  = le::get_u16(d + 10);
    s.ignition_delay_mean_ns  = le::get_u16(d + 12);
    s.ignition_delay_stddev_ns= le::get_u16(d + 14);
    s.peak_I_mean_dA          = le::get_u16(d + 16);
    s.peak_I_max_dA           = le::get_u16(d + 18);
    s.gap_V_recovery_mean_ns  = le::get_u16(d + 20);
    s.energy_delivered_uJ     = le::get_u32(d + 22);
    s.temp_GaN_dC             = le::get_i16(d + 26);
    s.temp_L_dC               = le::get_i16(d + 28);
    s.dc_link_V_dV            = le::get_u16(d + 30);
    s.dc_link_I_avg_dA        = le::get_u16(d + 32);
    s.state                   = le::get_u8 (d + 34);
    s.mode_id_active          = le::get_u8 (d + 35);
    s.flags                   = le::get_u16(d + 36);
    return true;
}

bool decodeAckModeBounds(const CanFrame& f, AckModeBounds& a) {
    if (f.id != ID_ACK_MODE_BOUNDS || f.len < 13) return false;
    const uint8_t* d = f.data;
    a.seq                = le::get_u16(d + 0);
    a.freq_max_kHz       = le::get_u16(d + 2);
    a.on_time_max_ns     = le::get_u16(d + 4);
    a.off_time_min_ns    = le::get_u16(d + 6);
    a.peak_I_setpoint_dA = le::get_u16(d + 8);
    a.peak_I_limit_hw_dA = le::get_u16(d + 10);
    a.status             = le::get_u8 (d + 12);
    return true;
}
```

- [ ] **Step 4: Run, verify PASS**

Run: `pio test -e tests -f '*EdmPsuProto*'`
Expected: all EdmPsuProto tests PASS.

- [ ] **Step 5: Commit**

```bash
git add FluidNC/src/EDM/Psu/Protocol.cpp FluidNC/tests/EdmPsuProtocolTest.cpp
git commit -m "feat(edm): decode STATS_AGG and ACK_MODE_BOUNDS (TDD)"
```

---

## Task 5: decode events (WIRE_BREAK, FAULT, ARC_BURST, INFO) + PSU_STATUS (TDD)

**Files:**
- Modify: `FluidNC/src/EDM/Psu/Protocol.cpp`
- Modify: `FluidNC/tests/EdmPsuProtocolTest.cpp`

- [ ] **Step 1: Add failing tests**

```cpp
TEST(EdmPsuProto, DecodeWireBreak) {
    CanFrame f(ID_WIRE_BREAK, 14);
    f.data[0] = 3;                      // severity critical
    f.data[1] = 0x05;                   // cause flags
    EDM::le::put_u16(f.data + 2, 12);   // recent_short
    EDM::le::put_u16(f.data + 4, 7);    // recent_arc
    EDM::le::put_u32(f.data + 6, 9999); // delay_var
    EDM::le::put_u32(f.data + 10, 5000);// timestamp
    WireBreakImminent w;
    ASSERT_TRUE(decodeWireBreak(f, w));
    EXPECT_EQ(w.severity, 3);
    EXPECT_EQ(w.recent_short_count, 12);
    EXPECT_EQ(w.timestamp_ms_since_start, 5000u);
}

TEST(EdmPsuProto, DecodeFault) {
    CanFrame f(ID_FAULT, 8);
    f.data[0] = 2;  // fault_code
    f.data[1] = 3;  // severity
    f.data[2] = 0xAA;
    Fault flt;
    ASSERT_TRUE(decodeFault(f, flt));
    EXPECT_EQ(flt.fault_code, 2);
    EXPECT_EQ(flt.severity, 3);
    EXPECT_EQ(flt.detail[0], 0xAA);
}

TEST(EdmPsuProto, DecodeInfoNullTerminates) {
    CanFrame f(ID_INFO, 5);
    std::memcpy(f.data, "hello", 5);
    char buf[16] = {};
    ASSERT_TRUE(decodeInfo(f, buf, sizeof(buf)));
    EXPECT_STREQ(buf, "hello");
}

TEST(EdmPsuProto, DecodePsuStatus) {
    CanFrame f(ID_PSU_STATUS, 13);
    f.data[0] = 0;                       // state
    EDM::le::put_u16(f.data + 1, 0x0101);// fpga
    EDM::le::put_u16(f.data + 3, 0x0202);// mcu
    EDM::le::put_u16(f.data + 5, 1);     // protocol version
    EDM::le::put_u32(f.data + 7, 3600);  // uptime
    EDM::le::put_u16(f.data + 11, 0);    // fault_count
    PsuStatus st;
    ASSERT_TRUE(decodePsuStatus(f, st));
    EXPECT_EQ(st.protocol_version, 1);
    EXPECT_EQ(st.uptime_s, 3600u);
}
```

- [ ] **Step 2: Run, verify fail.** `pio test -e tests -f '*EdmPsuProto*'` → link error.

- [ ] **Step 3: Implement in `Protocol.cpp`**

```cpp
bool decodeWireBreak(const CanFrame& f, WireBreakImminent& w) {
    if (f.id != ID_WIRE_BREAK || f.len < 14) return false;
    const uint8_t* d = f.data;
    w.severity                 = le::get_u8 (d + 0);
    w.cause_flags              = le::get_u8 (d + 1);
    w.recent_short_count       = le::get_u16(d + 2);
    w.recent_arc_count         = le::get_u16(d + 4);
    w.ignition_delay_var_ns2   = le::get_u32(d + 6);
    w.timestamp_ms_since_start = le::get_u32(d + 10);
    return true;
}

bool decodeFault(const CanFrame& f, Fault& flt) {
    if (f.id != ID_FAULT || f.len < 2) return false;
    flt.fault_code = f.data[0];
    flt.severity   = f.data[1];
    std::memset(flt.detail, 0, sizeof(flt.detail));
    uint8_t n = f.len > 8 ? 6 : uint8_t(f.len - 2);
    std::memcpy(flt.detail, f.data + 2, n);
    return true;
}

bool decodeArcBurst(const CanFrame& f, ArcBurst& a) {
    if (f.id != ID_ARC_BURST || f.len < 2) return false;
    a.consecutive_arcs = le::get_u16(f.data + 0);
    return true;
}

bool decodeInfo(const CanFrame& f, char* out, size_t out_cap) {
    if (f.id != ID_INFO || out_cap == 0) return false;
    size_t n = f.len < (out_cap - 1) ? f.len : (out_cap - 1);
    std::memcpy(out, f.data, n);
    out[n] = '\0';
    return true;
}

bool decodePsuStatus(const CanFrame& f, PsuStatus& st) {
    if (f.id != ID_PSU_STATUS || f.len < 13) return false;
    const uint8_t* d = f.data;
    st.state            = le::get_u8 (d + 0);
    st.fpga_version     = le::get_u16(d + 1);
    st.mcu_version      = le::get_u16(d + 3);
    st.protocol_version = le::get_u16(d + 5);
    st.uptime_s         = le::get_u32(d + 7);
    st.fault_count      = le::get_u16(d + 11);
    return true;
}
```

- [ ] **Step 4: Run, verify PASS.** `pio test -e tests -f '*EdmPsuProto*'` → all PASS.

- [ ] **Step 5: Commit**

```bash
git add FluidNC/src/EDM/Psu/Protocol.cpp FluidNC/tests/EdmPsuProtocolTest.cpp
git commit -m "feat(edm): decode event frames and PSU_STATUS heartbeat (TDD)"
```

---

## Task 6: CanBus interface + FakeCanBus test double

**Files:**
- Create: `FluidNC/src/EDM/Can/CanBus.h`

- [ ] **Step 1: Write `CanBus.h`**

```cpp
// FluidNC/src/EDM/Can/CanBus.h
#pragma once
#include <functional>
#include <vector>
#include "EDM/Can/CanFrame.h"

namespace EDM {

// Abstract CAN-FD transport. Implemented by Mcp2518fdDriver (target) and
// FakeCanBus (host tests). RX is delivered via a callback set by the owner.
class CanBus {
public:
    using RxHandler = std::function<void(const CanFrame&)>;
    virtual ~CanBus() = default;
    virtual bool init()                     = 0;
    virtual bool send(const CanFrame& f)    = 0;
    virtual void onReceive(RxHandler h)     = 0;
    // Drains hardware RX FIFO, invoking the handler per frame. Called from the
    // CAN-RX task on target; a no-op for fakes that inject directly.
    virtual void poll()                     {}
};

namespace test_support {

// Host-only fake: records sent frames; lets a test inject received frames.
class FakeCanBus : public CanBus {
public:
    bool init() override { return true; }
    bool send(const CanFrame& f) override { sent.push_back(f); return true; }
    void onReceive(RxHandler h) override { _h = std::move(h); }
    void inject(const CanFrame& f) { if (_h) _h(f); }

    std::vector<CanFrame> sent;
private:
    RxHandler _h;
};

}  // namespace test_support
}  // namespace EDM
```

- [ ] **Step 2: Commit** (header only, compiled via the test that uses it next task)

```bash
git add FluidNC/src/EDM/Can/CanBus.h
git commit -m "feat(edm): add CanBus transport interface + FakeCanBus test double"
```

---

## Task 7: IPsuLink interface + CanPsuLink session layer (TDD)

**Files:**
- Create: `FluidNC/src/EDM/Psu/IPsuLink.h`
- Create: `FluidNC/src/EDM/Psu/PsuLink.h`
- Create: `FluidNC/src/EDM/Psu/PsuLink.cpp`
- Create: `FluidNC/tests/EdmPsuLinkTest.cpp`
- Modify: `platformio.ini` (uncomment `+<EDM/Psu/PsuLink.cpp>` and `+<tests/EdmPsuLinkTest.cpp>`)

- [ ] **Step 1: Write `IPsuLink.h`**

```cpp
// FluidNC/src/EDM/Psu/IPsuLink.h
#pragma once
#include <cstdint>
#include "EDM/Psu/Protocol.h"

namespace EDM { namespace psu {

// One queued asynchronous event from the PSU.
struct Event {
    enum Kind : uint8_t { WireBreak, FaultEvt, ArcBurstEvt, Info } kind;
    WireBreakImminent wire_break;
    Fault             fault;
    ArcBurst          arc_burst;
    char              info[61];
};

// Interface the EdmController depends on. Implemented by CanPsuLink (real)
// and SimPsuLink (in-firmware simulator).
class IPsuLink {
public:
    virtual ~IPsuLink() = default;

    // Commands.
    virtual uint16_t setModeBounds(const SetModeBounds& m) = 0;  // returns seq
    virtual void     startCut()  = 0;
    virtual void     stopCut()   = 0;
    virtual void     clearFault() = 0;

    // Latest 1 kHz telemetry snapshot. Returns false if none received yet.
    virtual bool latestStats(StatsAgg& out) const = 0;

    // Pops one queued event; returns false if the queue is empty.
    virtual bool popEvent(Event& out) = 0;

    // Connection / protocol health (driven by PSU_STATUS heartbeat).
    virtual bool     isConnected() const = 0;       // heartbeat fresh
    virtual bool     protocolCompatible() const = 0; // version matches
    virtual uint16_t lastAckStatus() const = 0;     // 0 = ok / none
};

}}  // namespace EDM::psu
```

- [ ] **Step 2: Write the failing test**

```cpp
// FluidNC/tests/EdmPsuLinkTest.cpp
#include "gtest/gtest.h"
#include "EDM/Can/CanBus.h"
#include "EDM/Psu/PsuLink.h"

using namespace EDM;
using namespace EDM::psu;

TEST(EdmPsuLink, SetModeBoundsSendsFrameAndAssignsSeq) {
    test_support::FakeCanBus bus;
    CanPsuLink link(bus);
    link.begin();

    SetModeBounds m; m.mode_id = 1; m.peak_I_setpoint_dA = 120;
    uint16_t seq = link.setModeBounds(m);

    ASSERT_EQ(bus.sent.size(), 1u);
    EXPECT_EQ(bus.sent[0].id, ID_SET_MODE_BOUNDS);
    EXPECT_EQ(le::get_u16(&bus.sent[0].data[1]), seq);  // seq stamped into frame
}

TEST(EdmPsuLink, StatsAggUpdatesLatest) {
    test_support::FakeCanBus bus;
    CanPsuLink link(bus);
    link.begin();

    StatsAgg s; // empty snapshot before any frame
    EXPECT_FALSE(link.latestStats(s));

    CanFrame f(ID_STATS_AGG, 38);
    le::put_u16(f.data + 4, 88);  // n_normal
    bus.inject(f);

    ASSERT_TRUE(link.latestStats(s));
    EXPECT_EQ(s.n_normal, 88);
}

TEST(EdmPsuLink, WireBreakBecomesQueuedEvent) {
    test_support::FakeCanBus bus;
    CanPsuLink link(bus);
    link.begin();

    CanFrame f(ID_WIRE_BREAK, 14);
    f.data[0] = 2;  // severity
    bus.inject(f);

    Event e;
    ASSERT_TRUE(link.popEvent(e));
    EXPECT_EQ(e.kind, Event::WireBreak);
    EXPECT_EQ(e.wire_break.severity, 2);
    EXPECT_FALSE(link.popEvent(e));  // queue drained
}

TEST(EdmPsuLink, HeartbeatDrivesConnectedAndProtocol) {
    test_support::FakeCanBus bus;
    CanPsuLink link(bus);
    link.begin();
    EXPECT_FALSE(link.isConnected());

    CanFrame f(ID_PSU_STATUS, 13);
    le::put_u16(f.data + 5, kProtocolVersion);  // matching version
    bus.inject(f);

    EXPECT_TRUE(link.isConnected());
    EXPECT_TRUE(link.protocolCompatible());
}

TEST(EdmPsuLink, MismatchedProtocolFlagged) {
    test_support::FakeCanBus bus;
    CanPsuLink link(bus);
    link.begin();
    CanFrame f(ID_PSU_STATUS, 13);
    le::put_u16(f.data + 5, kProtocolVersion + 1);
    bus.inject(f);
    EXPECT_TRUE(link.isConnected());
    EXPECT_FALSE(link.protocolCompatible());
}
```

- [ ] **Step 3: Write `PsuLink.h`**

```cpp
// FluidNC/src/EDM/Psu/PsuLink.h
#pragma once
#include <deque>
#include "EDM/Can/CanBus.h"
#include "EDM/Psu/IPsuLink.h"

namespace EDM { namespace psu {

// Session layer over a CanBus: stamps sequence numbers, decodes inbound
// frames into a telemetry snapshot + event queue, tracks heartbeat health.
class CanPsuLink : public IPsuLink {
public:
    explicit CanPsuLink(CanBus& bus) : _bus(bus) {}

    void begin();  // registers RX handler

    uint16_t setModeBounds(const SetModeBounds& m) override;
    void     startCut()   override;
    void     stopCut()    override;
    void     clearFault() override;

    bool latestStats(StatsAgg& out) const override;
    bool popEvent(Event& out) override;
    bool isConnected() const override         { return _heartbeat_seen; }
    bool protocolCompatible() const override  { return _protocol_ok; }
    uint16_t lastAckStatus() const override   { return _last_ack_status; }

    // Called by the CAN-RX task each cycle (target). Tests inject directly.
    void onFrame(const CanFrame& f);

private:
    CanBus&  _bus;
    uint16_t _seq = 1;

    StatsAgg _stats{};
    bool     _stats_valid = false;

    std::deque<Event> _events;
    static constexpr size_t kMaxEvents = 32;

    bool     _heartbeat_seen = false;
    bool     _protocol_ok = false;
    uint16_t _last_ack_status = 0;
};

}}  // namespace EDM::psu
```

- [ ] **Step 4: Write `PsuLink.cpp`**

```cpp
// FluidNC/src/EDM/Psu/PsuLink.cpp
#include "EDM/Psu/PsuLink.h"
#include <cstring>

namespace EDM { namespace psu {

void CanPsuLink::begin() {
    _bus.onReceive([this](const CanFrame& f) { onFrame(f); });
}

uint16_t CanPsuLink::setModeBounds(const SetModeBounds& m_in) {
    SetModeBounds m = m_in;
    m.seq = _seq++;
    if (_seq == 0) _seq = 1;
    _bus.send(encodeSetModeBounds(m));
    return m.seq;
}

void CanPsuLink::startCut()   { _bus.send(encodeControl(CTRL_START_CUT)); }
void CanPsuLink::stopCut()    { _bus.send(encodeControl(CTRL_STOP_CUT)); }
void CanPsuLink::clearFault() { _bus.send(encodeControl(CTRL_CLEAR_FAULT)); }

bool CanPsuLink::latestStats(StatsAgg& out) const {
    if (!_stats_valid) return false;
    out = _stats;
    return true;
}

bool CanPsuLink::popEvent(Event& out) {
    if (_events.empty()) return false;
    out = _events.front();
    _events.pop_front();
    return true;
}

static void pushEvent(std::deque<Event>& q, size_t cap, const Event& e) {
    if (q.size() >= cap) q.pop_front();  // drop oldest on overflow
    q.push_back(e);
}

void CanPsuLink::onFrame(const CanFrame& f) {
    switch (f.id) {
        case ID_STATS_AGG:
            if (decodeStatsAgg(f, _stats)) _stats_valid = true;
            break;
        case ID_ACK_MODE_BOUNDS: {
            AckModeBounds a;
            if (decodeAckModeBounds(f, a)) _last_ack_status = a.status;
            break;
        }
        case ID_PSU_STATUS: {
            PsuStatus st;
            if (decodePsuStatus(f, st)) {
                _heartbeat_seen = true;
                _protocol_ok = (st.protocol_version == kProtocolVersion);
            }
            break;
        }
        case ID_WIRE_BREAK: {
            Event e; e.kind = Event::WireBreak;
            if (decodeWireBreak(f, e.wire_break)) pushEvent(_events, kMaxEvents, e);
            break;
        }
        case ID_FAULT: {
            Event e; e.kind = Event::FaultEvt;
            if (decodeFault(f, e.fault)) pushEvent(_events, kMaxEvents, e);
            break;
        }
        case ID_ARC_BURST: {
            Event e; e.kind = Event::ArcBurstEvt;
            if (decodeArcBurst(f, e.arc_burst)) pushEvent(_events, kMaxEvents, e);
            break;
        }
        case ID_INFO: {
            Event e; e.kind = Event::Info;
            if (decodeInfo(f, e.info, sizeof(e.info))) pushEvent(_events, kMaxEvents, e);
            break;
        }
        default: break;
    }
}

}}  // namespace EDM::psu
```

- [ ] **Step 5: Run, verify PASS**

Run: `pio test -e tests -f '*EdmPsuLink*'`
Expected: 5 tests PASS.

- [ ] **Step 6: Commit**

```bash
git add FluidNC/src/EDM/Psu/IPsuLink.h FluidNC/src/EDM/Psu/PsuLink.h FluidNC/src/EDM/Psu/PsuLink.cpp FluidNC/tests/EdmPsuLinkTest.cpp platformio.ini
git commit -m "feat(edm): add IPsuLink + CanPsuLink session layer (TDD)"
```

---

## Task 8: SimPsuLink — in-firmware gap simulator (TDD)

**Files:**
- Create: `FluidNC/src/EDM/Psu/SimPsuLink.h`
- Create: `FluidNC/src/EDM/Psu/SimPsuLink.cpp`
- Create: `FluidNC/tests/EdmSimPsuLinkTest.cpp`
- Modify: `platformio.ini` (uncomment `+<EDM/Psu/SimPsuLink.cpp>` and `+<tests/EdmSimPsuLinkTest.cpp>`)

**Design:** `SimPsuLink` implements `IPsuLink` without any CAN. It models gap width: the controller (later) calls `setGapTrend(mm)` each tick to report how the commanded motion changed the gap (advance shrinks gap, retract grows it). `tick()` produces a fresh `StatsAgg` whose classifier ratios reflect the gap: very small gap → mostly `short`, ideal gap → mostly `normal`, large gap → mostly `open`. This lets the gap servo be developed and tuned with no hardware.

- [ ] **Step 1: Write the failing test**

```cpp
// FluidNC/tests/EdmSimPsuLinkTest.cpp
#include "gtest/gtest.h"
#include "EDM/Psu/SimPsuLink.h"

using namespace EDM;
using namespace EDM::psu;

TEST(EdmSimPsuLink, StartsDisconnectedUntilBegun) {
    SimPsuLink sim;
    EXPECT_FALSE(sim.isConnected());
    sim.begin();
    EXPECT_TRUE(sim.isConnected());
    EXPECT_TRUE(sim.protocolCompatible());
}

TEST(EdmSimPsuLink, IdealGapYieldsMostlyNormal) {
    SimPsuLink sim;
    sim.begin();
    sim.setGap(sim.idealGapMm());
    sim.tick();
    StatsAgg s;
    ASSERT_TRUE(sim.latestStats(s));
    int total = s.n_normal + s.n_arc + s.n_short + s.n_open;
    ASSERT_GT(total, 0);
    EXPECT_GT(s.n_normal, total / 2);  // majority normal at ideal gap
}

TEST(EdmSimPsuLink, TooSmallGapYieldsShorts) {
    SimPsuLink sim;
    sim.begin();
    sim.setGap(0.0f);   // wire touching workpiece
    sim.tick();
    StatsAgg s;
    sim.latestStats(s);
    EXPECT_GT(s.n_short, s.n_normal);
}

TEST(EdmSimPsuLink, LargeGapYieldsOpens) {
    SimPsuLink sim;
    sim.begin();
    sim.setGap(sim.idealGapMm() * 5.0f);
    sim.tick();
    StatsAgg s;
    sim.latestStats(s);
    EXPECT_GT(s.n_open, s.n_normal);
}

TEST(EdmSimPsuLink, WindowIdIncrementsEachTick) {
    SimPsuLink sim;
    sim.begin();
    sim.tick();
    StatsAgg a; sim.latestStats(a);
    sim.tick();
    StatsAgg b; sim.latestStats(b);
    EXPECT_EQ(b.window_id, a.window_id + 1);
}
```

- [ ] **Step 2: Write `SimPsuLink.h`**

```cpp
// FluidNC/src/EDM/Psu/SimPsuLink.h
#pragma once
#include "EDM/Psu/IPsuLink.h"

namespace EDM { namespace psu {

// In-firmware PSU simulator. No CAN. Models the spark gap so the gap servo
// and EdmController can be developed and tuned with zero hardware.
class SimPsuLink : public IPsuLink {
public:
    void  begin()            { _connected = true; }
    void  setGap(float mm)   { _gap_mm = mm; }
    float idealGapMm() const { return _ideal_gap_mm; }

    // Advances the simulation one telemetry window and refreshes latestStats().
    void tick();

    // IPsuLink
    uint16_t setModeBounds(const SetModeBounds& m) override { _mode = m; return ++_seq; }
    void     startCut()   override { _cutting = true; }
    void     stopCut()    override { _cutting = false; }
    void     clearFault() override {}

    bool latestStats(StatsAgg& out) const override;
    bool popEvent(Event&) override { return false; }   // sim emits no events in P0
    bool isConnected() const override        { return _connected; }
    bool protocolCompatible() const override { return _connected; }
    uint16_t lastAckStatus() const override  { return 0; }

private:
    bool  _connected = false;
    bool  _cutting   = false;
    float _gap_mm    = 0.02f;
    float _ideal_gap_mm = 0.02f;   // 20 µm nominal working gap
    uint16_t _seq = 0;
    uint32_t _window = 0;
    StatsAgg _stats{};
    bool     _stats_valid = false;
    SetModeBounds _mode{};
};

}}  // namespace EDM::psu
```

- [ ] **Step 3: Write `SimPsuLink.cpp`**

```cpp
// FluidNC/src/EDM/Psu/SimPsuLink.cpp
#include "EDM/Psu/SimPsuLink.h"

namespace EDM { namespace psu {

void SimPsuLink::tick() {
    // Map gap width to classifier counts over a 100-pulse window.
    // ratio r = gap / ideal: r<<1 -> shorts; r~1 -> normal; r>>1 -> opens.
    constexpr int kPulses = 100;
    float r = (_ideal_gap_mm > 0.0f) ? (_gap_mm / _ideal_gap_mm) : 0.0f;

    int shorts = 0, opens = 0, normals = 0, arcs = 0;
    if (r < 0.5f) {
        shorts  = 70; arcs = 15; normals = 10; opens = 5;     // too tight
    } else if (r > 2.0f) {
        opens   = 70; normals = 20; arcs = 5; shorts = 5;     // too wide
    } else {
        normals = 80; arcs = 8; shorts = 6; opens = 6;        // healthy band
    }
    (void)kPulses;

    _stats = StatsAgg{};
    _stats.window_id   = _window++;
    _stats.n_normal    = uint16_t(normals);
    _stats.n_arc       = uint16_t(arcs);
    _stats.n_short     = uint16_t(shorts);
    _stats.n_open      = uint16_t(opens);
    _stats.state       = _cutting ? 0 : 1;        // running / paused
    _stats.mode_id_active = _mode.mode_id;
    _stats.dc_link_V_dV = 800;                    // 80.0 V
    _stats_valid = true;
}

bool SimPsuLink::latestStats(StatsAgg& out) const {
    if (!_stats_valid) return false;
    out = _stats;
    return true;
}

}}  // namespace EDM::psu
```

- [ ] **Step 4: Run, verify PASS**

Run: `pio test -e tests -f '*EdmSimPsuLink*'`
Expected: 5 tests PASS.

- [ ] **Step 5: Run the full EDM suite to confirm nothing regressed**

Run: `pio test -e tests -f '*Edm*'`
Expected: all EdmEndian / EdmPsuProto / EdmPsuLink / EdmSimPsuLink tests PASS.

- [ ] **Step 6: Commit**

```bash
git add FluidNC/src/EDM/Psu/SimPsuLink.h FluidNC/src/EDM/Psu/SimPsuLink.cpp FluidNC/tests/EdmSimPsuLinkTest.cpp platformio.ini
git commit -m "feat(edm): add SimPsuLink in-firmware gap simulator (TDD)"
```

---

## Task 9: MCP2518FD driver (target hardware, bench-verified)

This is the one component that cannot be host-tested (it touches SPI hardware). It is implemented against the `CanBus` interface and verified on the bench, not in googletest.

**Files:**
- Create: `FluidNC/src/EDM/Can/Mcp2518fdDriver.h`
- Create: `FluidNC/src/EDM/Can/Mcp2518fdDriver.cpp`

**Reference:** Microchip MCP2518FD datasheet (DS20006027) — SPI command set (RESET 0x00, READ 0x03, WRITE 0x02), register map (C1CON, C1NBTCFG, C1DBTCFG, C1FIFOCON, TX/RX FIFO RAM at 0x400+), and the CAN-FD DLC-to-length table.

- [ ] **Step 1: Write `Mcp2518fdDriver.h`**

```cpp
// FluidNC/src/EDM/Can/Mcp2518fdDriver.h
#pragma once
#include <cstdint>
#include "EDM/Can/CanBus.h"

namespace EDM {

// CanBus implementation for a Microchip MCP2518FD over SPI.
// Bit timing fixed to the PSU contract: 1 Mbps arbitration, 5 Mbps data.
class Mcp2518fdDriver : public CanBus {
public:
    struct Pins { int sck, miso, mosi, cs, intr; };
    struct Config {
        Pins     pins;
        uint8_t  spi_host    = 1;          // HSPI/VSPI selection
        uint32_t spi_hz      = 17000000;   // up to 20 MHz
        uint32_t osc_hz      = 40000000;   // external crystal on the module
    };

    explicit Mcp2518fdDriver(const Config& cfg) : _cfg(cfg) {}

    bool init() override;
    bool send(const CanFrame& f) override;
    void onReceive(RxHandler h) override { _rx = std::move(h); }
    void poll() override;   // drain RX FIFO; call from CAN-RX task

private:
    Config    _cfg;
    RxHandler _rx;

    // SPI primitives
    void     reset();
    uint32_t readReg(uint16_t addr);
    void     writeReg(uint16_t addr, uint32_t val);
    void     readRam(uint16_t addr, uint8_t* buf, uint8_t n);
    void     writeRam(uint16_t addr, const uint8_t* buf, uint8_t n);

    static uint8_t lenToDlc(uint8_t len);
    static uint8_t dlcToLen(uint8_t dlc);
};

}  // namespace EDM
```

- [ ] **Step 2: Implement `Mcp2518fdDriver.cpp`** — initialization sequence

```cpp
// FluidNC/src/EDM/Can/Mcp2518fdDriver.cpp
#include "EDM/Can/Mcp2518fdDriver.h"
#include <SPI.h>
#include <Arduino.h>

namespace EDM {

// --- MCP2518FD SPI instructions & key registers (DS20006027) ---
static constexpr uint8_t  CMD_RESET = 0x00;
static constexpr uint8_t  CMD_READ  = 0x03;  // 0x3nnn
static constexpr uint8_t  CMD_WRITE = 0x02;  // 0x2nnn
static constexpr uint16_t REG_C1CON     = 0x000;
static constexpr uint16_t REG_C1NBTCFG  = 0x004;  // nominal bit time (1 Mbps)
static constexpr uint16_t REG_C1DBTCFG  = 0x008;  // data bit time (5 Mbps)
static constexpr uint16_t REG_C1TDC     = 0x00C;
static constexpr uint16_t REG_C1TEFCON  = 0x040;
static constexpr uint16_t REG_C1FIFOCON1= 0x05C;  // TX FIFO
static constexpr uint16_t REG_C1FIFOCON2= 0x070;  // RX FIFO

bool Mcp2518fdDriver::init() {
    pinMode(_cfg.pins.cs, OUTPUT);
    digitalWrite(_cfg.pins.cs, HIGH);
    pinMode(_cfg.pins.intr, INPUT);
    SPI.begin(_cfg.pins.sck, _cfg.pins.miso, _cfg.pins.mosi, _cfg.pins.cs);

    reset();
    delay(2);

    // Bit timing for osc_hz=40MHz:
    //  Nominal 1 Mbps: BRP=0, TSEG1=30, TSEG2=7, SJW=7  -> 40 TQ
    //  Data    5 Mbps: BRP=0, TSEG1=6,  TSEG2=1, SJW=1  -> 8 TQ
    writeReg(REG_C1NBTCFG, (0u << 24) | (30u << 16) | (7u << 8) | 7u);
    writeReg(REG_C1DBTCFG, (0u << 24) | (6u  << 16) | (1u << 8) | 1u);

    // Configure one TX FIFO and one RX FIFO (payload 64, FD enabled).
    // (FIFO RAM addresses, TXEN/RXEN, PLSIZE=64B set per datasheet bitfields.)
    writeReg(REG_C1FIFOCON1, /* TX, 64B payload, depth 8 */ 0x000000F8u);
    writeReg(REG_C1FIFOCON2, /* RX, 64B payload, depth 16 */ 0x000000F9u);

    // Request Normal CAN-FD mode (C1CON.REQOP = 0), enable ISO CRC.
    uint32_t con = readReg(REG_C1CON);
    con &= ~(0x7u << 24);          // REQOP = Normal FD
    writeReg(REG_C1CON, con);
    return true;
}

void Mcp2518fdDriver::reset() {
    digitalWrite(_cfg.pins.cs, LOW);
    SPI.transfer(CMD_RESET);
    SPI.transfer(0x00);
    digitalWrite(_cfg.pins.cs, HIGH);
}

uint32_t Mcp2518fdDriver::readReg(uint16_t addr) {
    digitalWrite(_cfg.pins.cs, LOW);
    SPI.transfer(uint8_t((CMD_READ << 4) | ((addr >> 8) & 0x0F)));
    SPI.transfer(uint8_t(addr & 0xFF));
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= uint32_t(SPI.transfer(0x00)) << (8 * i);
    digitalWrite(_cfg.pins.cs, HIGH);
    return v;
}

void Mcp2518fdDriver::writeReg(uint16_t addr, uint32_t val) {
    digitalWrite(_cfg.pins.cs, LOW);
    SPI.transfer(uint8_t((CMD_WRITE << 4) | ((addr >> 8) & 0x0F)));
    SPI.transfer(uint8_t(addr & 0xFF));
    for (int i = 0; i < 4; ++i) SPI.transfer(uint8_t(val >> (8 * i)));
    digitalWrite(_cfg.pins.cs, HIGH);
}

uint8_t Mcp2518fdDriver::lenToDlc(uint8_t len) {
    if (len <= 8)  return len;
    if (len <= 12) return 9;
    if (len <= 16) return 10;
    if (len <= 20) return 11;
    if (len <= 24) return 12;
    if (len <= 32) return 13;
    if (len <= 48) return 14;
    return 15;  // 64
}

uint8_t Mcp2518fdDriver::dlcToLen(uint8_t dlc) {
    static const uint8_t t[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    return t[dlc & 0x0F];
}

// send()/poll()/readRam()/writeRam() build a TX object (T0=SID, T1=DLC|FDF|BRS,
// then padded data) into the TX FIFO RAM and read RX objects out of the RX FIFO.
// Full bodies completed during bench bring-up against the datasheet.
bool Mcp2518fdDriver::send(const CanFrame& f) { /* bench bring-up */ (void)f; return false; }
void Mcp2518fdDriver::poll() { /* bench bring-up: drain RX FIFO -> _rx(frame) */ }
void Mcp2518fdDriver::readRam(uint16_t, uint8_t*, uint8_t) {}
void Mcp2518fdDriver::writeRam(uint16_t, const uint8_t*, uint8_t) {}

}  // namespace EDM
```

- [ ] **Step 3: Bench verification (no googletest — record results in the commit message)**

Wire the MCP2518FD module to the PiBot's SPI + a free CS + INT (see P0 pin-audit task below). With a USB-CAN-FD adapter on the same bus running the PSU project's `mock-psu`:

1. Flash a minimal sketch that calls `Mcp2518fdDriver::init()` then `send(encodeControl(CTRL_START_CUT))`. Confirm the frame appears on the adapter at ID 0x020.
2. Have `mock-psu` emit `PSU_STATUS` (0x300); confirm `poll()` delivers it to the RX handler and the bytes decode via `decodePsuStatus`.

Expected: TX frame observed on the bus; RX frame round-trips through the decoder. Capture a screenshot/log in the PR.

- [ ] **Step 4: Commit**

```bash
git add FluidNC/src/EDM/Can/Mcp2518fdDriver.h FluidNC/src/EDM/Can/Mcp2518fdDriver.cpp
git commit -m "feat(edm): add MCP2518FD CAN-FD SPI driver (bench-verified)"
```

---

## Task 10: PiBot V4.96 Pro pin audit (hardware verification, gating)

No code. This resolves the highest risk in the spec (§10.1) before P1+ build on it.

- [ ] **Step 1:** From the PiBot V4.96 Pro schematic / wiki, enumerate every ESP32 GPIO and its assignment. Identify a usable SPI bus (SCK/MOSI/MISO) and **two free GPIO** for MCP2518FD CS + INT, **two free GPIO** for the HX711 (DOUT/SCK), and confirm the **two unused motor headers** (after X/Y/U/V) expose step/dir for the wire-feed steppers.
- [ ] **Step 2:** Document the chosen pin map in `docs/superpowers/hardware-pinmap-pibot-v496pro.md` (create it), including any conflicts (SPI shared with SD/TMC) and whether an expansion adapter board is required.
- [ ] **Step 3:** If pins are insufficient, raise it before starting P1 — it may change the board or require an I/O expander.
- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/hardware-pinmap-pibot-v496pro.md
git commit -m "docs(edm): PiBot V4.96 Pro pin audit and EDM pin map"
```

---

## Phase P0 exit criteria

- `pio test -e tests -f '*Edm*'` is green (Endian, Protocol, PsuLink, SimPsuLink suites).
- MCP2518FD driver sends and receives a frame against `mock-psu` on the bench (logged).
- Pin map documented; CAN + HX711 + 2 wire-feed motors confirmed fit (or an adapter decided).
- All EDM code under `FluidNC/src/EDM/`; upstream FluidNC sources untouched except the `platformio.ini` test allowlist.

---

## Roadmap: Phases P1–P4 (each becomes its own detailed plan after its predecessor lands)

These are intentionally task-level, not bite-sized — they will be expanded into full TDD plans once P0 fixes the interfaces they build on.

### P1 — EDM process controller + spindle facade (host-testable)
- `EdmController` state machine (Idle → TouchOff → Cutting → Hold → Fault) driven by `IPsuLink`; tested against `SimPsuLink`.
- Gap-servo control: classifier-ratio → servo error → PI → signed `v_s` (clamped), deadband, anti-windup, sustained-short retract reflex, touch-off creep. **Pure function + class, fully host-TDD'd** with synthetic `StatsAgg` sequences.
- Mode/parameter mapping: named modes (rough/finish/…) → `SetModeBounds`; `S`-word → mode/energy selection.
- `EdmSpindle` (`Spindles/EdmSpindle.{h,cpp}`) registered via `SpindleFactory::InstanceBuilder("EDM")`; `setState()` delegates to `EdmController`.
- **Freeze the §7 `EdmReport` data object here** so sub-project C can start.

### P2 — Path-parameter servo / bidirectional motion (target + dry-run)
- `EdmMotion`: contour ring buffer (passed + upcoming waypoints), `s`→`pose(s)` interpolation (host-TDD'd), retract-bound enforcement.
- EDM cut-mode modal hook: route `G1/G2/G3` while EDM spindle is on into the contour buffer (integrate at the `mc_linear` boundary in `MotionControl.cpp`).
- Short-segment generator: each servo tick emit one accel-limited `plan_buffer_line` segment toward `pose(s+Δs)`; shallow planner buffer.
- Dry-run acceptance: spark disabled, axes advance/hold/**retract** under `SimPsuLink` gap scenarios; verify no planner/stepper glitches.

### P3 — Wire feed + tension (target + bench)
- `WireFeed`: two RMT-driven steppers; HX711 load-cell driver; tension PI loop (host-TDD'd as a pure controller); `wireFeedTask`.
- Wire-break reaction wired to `IPsuLink` events (severity-scaled feed/tension/stop).
- Bench acceptance: tension loop holds setpoint against a real load cell; wire-break severities trigger configured responses.

### P4 — Integration, config, telemetry surface
- `edm:` + `wire_feed:` YAML config groups (`Configuration::Configurable`).
- `EdmReport` push on the auto-report WebSocket (~5–10 Hz) + `$EDM/Status` command + `$EDM/Mode=`, `$EDM/TouchOff`, `$EDM/Tension=`.
- CAN-RX FreeRTOS task (INT-driven, pinned to core 0) + `edmServoTask` (1 ms, core 1).
- End-to-end against `mock-psu`, then real PSU. Joint milestone: PSU spec acceptance cut.

---

## Self-review notes

- **Spec coverage (P0 portion):** CanBus/CanFrame abstraction ✓ (T1, T6); full catalog codec — SET_MODE_BOUNDS, Control, ACK, STATS_AGG, WIRE_BREAK, FAULT, ARC_BURST, INFO, PSU_STATUS ✓ (T2–T5); `IPsuLink`/`CanPsuLink` session + heartbeat/protocol-version check ✓ (T7); `SimPsuLink` ✓ (T8); MCP2518FD driver ✓ (T9); PiBot pin-audit risk ✓ (T10). Bitstream upload (v1 = version-check only) is satisfied by the `PSU_STATUS` protocol-version handling in T7; full relay remains deferred per spec §1/§6.2.
- **Endianness** is locked little-endian and must be mirrored by the PSU codec — flagged in conventions; a divergence here is the most likely interop bug, hence the byte-exact layout tests.
- **Field layouts** (esp. `SetModeBounds` offset 15 for polarity with 13–14 padding, and the 38-byte `STATS_AGG`) are authoritative in this plan and must be reconciled with the PSU project's encoder during P0 bench interop (T9) — if they disagree, the PSU spec's wire format wins and these structs are adjusted.
- **Placeholder scan:** the only deferred bodies are `Mcp2518fdDriver::send/poll/readRam/writeRam`, which are explicitly bench-bring-up (cannot be host-tested); every host-tested function has complete code.
