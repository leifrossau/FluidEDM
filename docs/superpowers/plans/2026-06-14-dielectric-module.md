# Dielectric Module — PiBot Firmware Integration — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Integrate the dielectric (coolant) CAN-FD module into the FluidEDM firmware: a `DielLink` (real + simulated), the EDM cut **interlock** (no spark without flushing), dielectric **telemetry** through `EdmReport` → the WebUI, plus config and `$EDM/Flush|Pump` commands — all developed mock-first and host-tested.

**Architecture:** The dielectric is a separate CAN-FD node (IDs 0x700–0x7FF) on the bus the firmware already drives. This plan mirrors the proven `EDM/Psu/*` stack: a pure codec, an `IDielLink` interface with `CanDielLink` (real) + `SimDielLink` (mock), consumed by `EdmController`. The physical module board + its MCU firmware are a **separate project**; this plan is firmware-only.

**Tech Stack:** C++17, PlatformIO, googletest (`[env:tests]`), the existing `EDM/Can` + `EDM/Psu` + `EDM/Control` modules.

**Spec:** `../specs/2026-06-14-dielectric-module-design.md`.

**Builds on (merged to `main`):** `EDM/Can/{CanFrame,CanBus}`, `EDM/Psu/{Protocol,Endian,IPsuLink,PsuLink,SimPsuLink,EdmLock}`, `EDM/Control/EdmController`, `EDM/Servo/{EdmReport,FaultReason}`, `EDM/EdmReportChannel`, `EDM/EdmSpindle`, `EDM/EdmCommands`.

**Conventions (same as P0–P3):**
- Test cmd: `export PATH="/c/ProgramData/mingw64/mingw64/bin:$HOME/.platformio/penv/Scripts:$PATH" && pio test -e tests -f '<filter>'`.
- `build_src_filter` paths relative to `FluidNC/`. Add sources/tests as each task creates them.
- Little-endian payloads; integer wire encoding ("No floats" boundary).
- `[H]` = host-tested googletest (RED→GREEN). `[B]` = ESP32-coupled, bench-verified, NOT host-built.

---

## File structure

```
FluidNC/src/EDM/Diel/
  DielProtocol.h / .cpp      [H]  catalog IDs + structs + encode/decode (mirrors Psu/Protocol)
  IDielLink.h                [H]  interface (DielEvent, IDielLink)
  CanDielLink.h / .cpp       [H]  session layer over CanBus (EdmLock-guarded) — host-built (no Arduino)
  SimDielLink.h / .cpp       [H]  in-firmware coolant simulator
Tests: tests/EdmDielProtocolTest.cpp, EdmDielLinkTest.cpp, EdmDielControllerTest.cpp
```
Edits: `EDM/Servo/EdmReport.h` (+diel fields), `EDM/Servo/FaultReason.h` (+2), `EDM/Control/EdmController.{h,cpp}` (attach + interlock + telemetry), `EDM/EdmReportChannel.cpp` (emit), `EDM/EdmSpindle.{h,cpp}` (config + wiring) [B], `EDM/EdmCommands.cpp` ($EDM/Flush|Pump) [B], `platformio.ini`.

---

## Task D1 [H]: DielProtocol — catalog + codec

**Files:** Create `FluidNC/src/EDM/Diel/DielProtocol.h`, `.cpp`, `FluidNC/tests/EdmDielProtocolTest.cpp`. Modify `platformio.ini` (add `+<EDM/Diel/DielProtocol.cpp>`, `+<tests/EdmDielProtocolTest.cpp>`).

- [ ] **Step 1: `DielProtocol.h`**
```cpp
// FluidNC/src/EDM/Diel/DielProtocol.h
#pragma once
#include <cstdint>
#include "EDM/Can/CanFrame.h"

namespace EDM { namespace diel {

enum Id : uint16_t {
    ID_SET_DIEL     = 0x710,  // FluidNC -> module
    ID_ACK_DIEL     = 0x711,  // module -> FluidNC
    ID_DIEL_STATS   = 0x720,  // module -> FluidNC, ~10 Hz
    ID_DIEL_FAULT   = 0x730,
    ID_DIEL_STATUS  = 0x740,  // module -> FluidNC, 100 ms heartbeat
};
constexpr uint16_t kProtocolVersion = 1;

struct SetDiel {
    uint16_t seq = 0;
    uint8_t  pump_on = 0;
    uint8_t  flush_level = 0;        // 0..3
    int16_t  temp_setpoint_dC = 220; // 0.1 C
    uint8_t  deioniser_enable = 1;
    uint16_t flags = 0;
};
struct AckDiel {
    uint16_t seq = 0; uint8_t pump_on = 0; uint8_t flush_level = 0;
    int16_t temp_setpoint_dC = 0; uint8_t status = 0;
};
struct DielStats {
    uint32_t window_id = 0;
    uint8_t  pump_on = 0, flush_level = 0;
    uint16_t flush_mbar = 0, flow_clpm = 0;       // cL/min
    int16_t  temp_dC = 0, temp_set_dC = 0;
    uint16_t conductivity_uS = 0;
    uint8_t  level_pct = 0, filter_pct = 0;
    uint16_t flags = 0;                            // bit0 chiller,1 low_flow,2 low_level,3 high_cond,4 filter_clog
};
struct DielFault { uint8_t fault_code = 0, severity = 0; uint8_t detail[6] = {}; };
struct DielStatus { uint8_t state = 0; uint16_t fw_version = 0, protocol_version = 0; uint32_t uptime_s = 0; };

CanFrame encodeSetDiel(const SetDiel& s);
bool decodeAckDiel(const CanFrame& f, AckDiel& out);
bool decodeDielStats(const CanFrame& f, DielStats& out);
bool decodeDielFault(const CanFrame& f, DielFault& out);
bool decodeDielStatus(const CanFrame& f, DielStatus& out);

}}  // namespace EDM::diel
```

- [ ] **Step 2: Failing test `EdmDielProtocolTest.cpp`** — round-trip each message, byte-exact, plus id/len rejection. Use `EDM::le` helpers (from `EDM/Psu/Endian.h`).
```cpp
#include "gtest/gtest.h"
#include "EDM/Diel/DielProtocol.h"
#include "EDM/Psu/Endian.h"
using namespace EDM; using namespace EDM::diel;

TEST(EdmDielProto, EncodeSetDielLayout){
    SetDiel s; s.seq=0x1234; s.pump_on=1; s.flush_level=2; s.temp_setpoint_dC=205; s.deioniser_enable=1;
    CanFrame f=encodeSetDiel(s);
    EXPECT_EQ(f.id, ID_SET_DIEL);
    EXPECT_EQ(le::get_u16(&f.data[0]), 0x1234);
    EXPECT_EQ(f.data[2], 1); EXPECT_EQ(f.data[3], 2);
    EXPECT_EQ(le::get_i16(&f.data[4]), 205); EXPECT_EQ(f.data[6], 1);
}
TEST(EdmDielProto, DecodeDielStatsRoundTrip){
    CanFrame f(ID_DIEL_STATS, 19); uint8_t* d=f.data;
    le::put_u32(d+0, 7); d[4]=1; d[5]=2; le::put_u16(d+6,2600); le::put_u16(d+8,650);
    le::put_i16(d+10,228); le::put_i16(d+12,220); le::put_u16(d+14,7); d[16]=88; d[17]=96; le::put_u16(d+17? d+17:d+17,0);
    le::put_u16(d+17, 0); // placeholder, see note
    DielStats s; ASSERT_TRUE(decodeDielStats(f,s));
    EXPECT_EQ(s.flow_clpm,650); EXPECT_EQ(s.temp_dC,228); EXPECT_EQ(s.level_pct,88);
}
TEST(EdmDielProto, RejectsWrongIdAndShort){
    CanFrame bad(0x999,19); DielStats s; EXPECT_FALSE(decodeDielStats(bad,s));
    CanFrame shortf(ID_DIEL_STATS,4); EXPECT_FALSE(decodeDielStats(shortf,s));
}
TEST(EdmDielProto, DecodeStatusVersion){
    CanFrame f(ID_DIEL_STATUS,9); le::put_u16(f.data+1,kProtocolVersion); /* fields per layout */
    DielStatus st; ASSERT_TRUE(decodeDielStatus(f,st)); EXPECT_EQ(st.protocol_version,kProtocolVersion);
}
```
> Implementer note: pick a single authoritative byte layout for `DielStats` and `DielStatus` and make the test match it exactly (the placeholder math above is illustrative). Recommended `DielStats` offsets: window_id@0(u32), pump_on@4, flush_level@5, flush_mbar@6(u16), flow_clpm@8(u16), temp_dC@10(i16), temp_set_dC@12(i16), conductivity_uS@14(u16), level_pct@16, filter_pct@17, flags@18(u16) → len 20. `DielStatus`: state@0, fw_version@1(u16), protocol_version@3(u16), uptime_s@5(u32) → len 9. `SetDiel`: seq@0(u16), pump_on@2, flush_level@3, temp_setpoint_dC@4(i16), deioniser_enable@6, flags@7(u16) → len 9. `AckDiel`: seq@0(u16), pump_on@2, flush_level@3, temp_setpoint_dC@4(i16), status@6 → len 7.

- [ ] **Step 3: Run RED**, then implement `DielProtocol.cpp` with `EDM::le` helpers exactly mirroring `EDM/Psu/Protocol.cpp` (id+len guard on each decode; little-endian put/get at the offsets above).
- [ ] **Step 4: Run GREEN** (`pio test -e tests -f '*EdmDielProto*'`). **Step 5: Commit** `feat(edm): dielectric CAN-FD protocol catalog + codec (TDD)`.

---

## Task D2 [H]: IDielLink + CanDielLink + SimDielLink

**Files:** Create `FluidNC/src/EDM/Diel/IDielLink.h`, `CanDielLink.{h,cpp}`, `SimDielLink.{h,cpp}`, `FluidNC/tests/EdmDielLinkTest.cpp`. Modify `platformio.ini` (+`CanDielLink.cpp`, +`SimDielLink.cpp`, +test).

- [ ] **Step 1: `IDielLink.h`**
```cpp
// FluidNC/src/EDM/Diel/IDielLink.h
#pragma once
#include <cstdint>
#include "EDM/Diel/DielProtocol.h"
namespace EDM { namespace diel {
struct DielEvent { enum Kind:uint8_t{FaultEvt} kind; DielFault fault; DielEvent(){} };
class IDielLink {
public:
  virtual ~IDielLink() = default;
  virtual uint16_t setDiel(const SetDiel& s) = 0;       // returns seq
  virtual bool latestStats(DielStats& out) const = 0;
  virtual bool popEvent(DielEvent& out) = 0;
  virtual bool isConnected() const = 0;                  // heartbeat fresh
  virtual bool protocolCompatible() const = 0;
  virtual bool present() const = 0;                      // a module responded
};
}}  // namespace EDM::diel
```

- [ ] **Step 2: `CanDielLink.{h,cpp}`** — mirror `EDM/Psu/PsuLink.{h,cpp}` exactly: hold `CanBus&`, `begin()` registers `onReceive`, `onFrame(f)` switches on `f.id` (ignores non-0x7xx), decodes `DIEL_STATS`→snapshot, `DIEL_FAULT`→event deque, `DIEL_STATUS`→heartbeat/protocol/present; `setDiel` stamps `seq` + sends `encodeSetDiel`; guard shared state with `EDM::psu::EdmLock` (reuse `EDM/Psu/EdmLock.h`). `present()` = heartbeat seen within a freshness window (track last-status tick; for host tests, true once a status frame arrives).

- [ ] **Step 3: `SimDielLink.{h,cpp}`** — in-firmware simulator implementing `IDielLink`. State: `pump_on, flush_level, flush_mbar, flow_clpm, temp_dC, temp_set_dC, conductivity_uS, level_pct, filter_pct`. `setDiel` applies commands. `tick()` advances the model (port the WebUI mock: pump tracks an external "cutting" flag set via `setCutting(bool)`, flush/flow track level, temp regulates to setpoint, conductivity creeps, level/filter deplete). `present()` returns true after `begin()`. `latestStats` returns the model.

- [ ] **Step 4: Failing test `EdmDielLinkTest.cpp`** (FakeCanBus for CanDielLink; direct for SimDielLink):
```cpp
TEST(EdmDielLink, SetDielSendsFrameAndSeq){ FakeCanBus bus; CanDielLink l(bus); l.begin();
  SetDiel s; s.flush_level=3; uint16_t seq=l.setDiel(s);
  ASSERT_EQ(bus.sent.size(),1u); EXPECT_EQ(bus.sent[0].id,ID_SET_DIEL);
  EXPECT_EQ(EDM::le::get_u16(&bus.sent[0].data[0]),seq); }
TEST(EdmDielLink, StatsAndHeartbeat){ FakeCanBus bus; CanDielLink l(bus); l.begin();
  DielStats s; EXPECT_FALSE(l.latestStats(s));
  CanFrame f(ID_DIEL_STATS,20); EDM::le::put_u16(f.data+8,650); bus.inject(f);
  ASSERT_TRUE(l.latestStats(s)); EXPECT_EQ(s.flow_clpm,650);
  EXPECT_FALSE(l.present());
  CanFrame h(ID_DIEL_STATUS,9); EDM::le::put_u16(h.data+3,kProtocolVersion); bus.inject(h);
  EXPECT_TRUE(l.present()); EXPECT_TRUE(l.protocolCompatible()); }
TEST(EdmDielSim, PumpFollowsCutAndRegulates){ SimDielLink sim; sim.begin();
  sim.setCutting(true); SetDiel s; s.flush_level=2; sim.setDiel(s);
  for(int i=0;i<200;++i) sim.tick(0.012f);
  DielStats st; ASSERT_TRUE(sim.latestStats(st));
  EXPECT_EQ(st.pump_on,1); EXPECT_GT(st.flow_clpm,0); EXPECT_GT(st.flush_mbar,0); }
TEST(EdmDielSim, FlushOffNoFlow){ SimDielLink sim; sim.begin(); sim.setCutting(true);
  SetDiel s; s.flush_level=0; sim.setDiel(s); for(int i=0;i<200;++i) sim.tick(0.012f);
  DielStats st; sim.latestStats(st); EXPECT_LT(st.flow_clpm,50); }
```

- [ ] **Step 5: Run GREEN**, **Step 6: Commit** `feat(edm): IDielLink + CanDielLink session + SimDielLink simulator (TDD)`.

---

## Task D3 [H]: EdmController interlock + dielectric telemetry

**Files:** Modify `EDM/Servo/FaultReason.h`, `EDM/Servo/EdmReport.h`, `EDM/Control/EdmController.{h,cpp}`; extend `FluidNC/tests/EdmControllerTest.cpp`.

- [ ] **Step 1:** `FaultReason.h` — add `DielectricNotReady, DielectricLost` to the enum.
- [ ] **Step 2:** `EdmReport.h` — add the diel fields (§5.3 of the spec):
```cpp
    bool     diel_present = false;
    uint8_t  diel_pump_on = 0, diel_flush_level = 0, diel_level_pct = 0, diel_filter_pct = 0;
    uint16_t diel_flush_mbar = 0, diel_flow_clpm = 0, diel_conductivity_uS = 0;
    int16_t  diel_temp_dC = 0, diel_temp_set_dC = 0;
    uint16_t diel_flags = 0;
```
- [ ] **Step 3:** `EdmController.h` — add `#include "EDM/Diel/IDielLink.h"`, a config struct for interlock floors, a setter, and a member:
```cpp
    struct DielInterlock { bool required=false; uint16_t flow_min_clpm=100; uint8_t level_min_pct=15;
                           uint16_t conductivity_warn_uS=20; };
    void attachDielectric(EDM::diel::IDielLink* d, const DielInterlock& cfg) { _diel=d; _dielCfg=cfg; }
  private: // additions
    EDM::diel::IDielLink* _diel = nullptr;
    DielInterlock _dielCfg;
    bool dielReadyToCut(const EDM::diel::DielStats& s) const;
```
- [ ] **Step 4:** `EdmController.cpp` — in `tick()`:
  - Near the top, read dielectric telemetry: `EDM::diel::DielStats ds; bool dgot = _diel && _diel->latestStats(ds);` and populate `_report.diel_*` (present = `_diel && _diel->present()`).
  - **Gate (Armed→TouchOff):** change the ack-accept branch (cpp:126) so it ALSO requires the dielectric: `else if (connected && _link.lastAckStatus()==0 && (!dielRequired() || (dgot && dielReadyToCut(ds)))) { startCut... }`. Keep the `_arm_deadline` → `AckTimeout` path, and add: if armed && dielRequired && !ready && deadline passed → `enterFault(FaultReason::DielectricNotReady)`.
  - **Mid-cut loss:** in the active-states fault block (cpp:116-121) add: `if (dielRequired() && dgot && !dielReadyToCut(ds)) enterFault(FaultReason::DielectricLost);` (and if required but `!_diel->present()` → same).
  - Helper: `dielRequired()` = `_diel && _dielCfg.required`; `dielReadyToCut(s)` = `s.flow_clpm>=_dielCfg.flow_min_clpm && s.level_pct>=_dielCfg.level_min_pct`. Conductivity over warn → set a `_report.diel_flags` warning bit (no fault).
- [ ] **Step 5: Append tests** to `EdmControllerTest.cpp` (a `SimDielLink` wired via `attachDielectric` with `required=true`):
```cpp
TEST(EdmController, DielectricGatesStartCut){
  SimPsuLink psu; psu.begin(); ServoConfig cfg; ModeTable modes=makeModes();
  EdmController ctl(psu,cfg,modes);
  EDM::diel::SimDielLink diel; diel.begin();   // present, but pump off / no flow until cutting+flush
  EdmController::DielInterlock di; di.required=true; ctl.attachDielectric(&diel,di);
  ctl.requestCut(900);
  uint32_t t=0; for(int i=0;i<10;++i){ psu... ctl.tick(t++); }   // (drive psu sim as in other tests)
  EXPECT_NE(ctl.state(), EdmState::Cutting);                      // blocked: no flushing yet
  // turn flushing on (operator/Workbench would do this); diel reaches flow
  diel.setCutting(true); EDM::diel::SetDiel s; s.flush_level=2; diel.setDiel(s);
  for(int i=0;i<300;++i){ diel.tick(0.012f); ctl.tick(t++); }
  EXPECT_NE(ctl.state(), EdmState::Fault);
  EXPECT_TRUE(ctl.state()==EdmState::TouchOff||ctl.state()==EdmState::Cutting||ctl.state()==EdmState::Hold);
}
TEST(EdmController, DielectricLossFaultsMidCut){ /* reach cutting with flow, then flush_level=0 -> flow drops -> DielectricLost */ }
TEST(EdmController, NoDielectricNoGate){ /* no attachDielectric -> startCut proceeds as before */ }
TEST(EdmController, ReportPopulatesDielFields){ /* attach sim, tick, assert _report.diel_* reflect sim stats */ }
```
  Make the `SimDielLink` expose `setCutting`/`tick` for the test to drive flow. Tune iteration counts so flow crosses `flow_min_clpm`. Investigate failures: prefer fixing impl for clear-intent (gate/loss) issues; adjust test thresholds only if mis-calibrated.
- [ ] **Step 6: Run GREEN** (`pio test -e tests -f '*EdmController*'` + full `*Edm*`), **coverage** ≥80% on new logic. **Step 7: Commit** `feat(edm): dielectric cut-interlock + telemetry in EdmController (TDD)`.

---

## Task D4 [B]: EdmReportChannel emit + EdmSpindle wiring + commands (ESP32)

ESP32-coupled; NOT host-built. Verified by firmware build + bench.

- [ ] **Step 1:** `EdmReportChannel.cpp` — add a `dielectric` JSON object to `report_edm_stats`/`edm_status_dump` from the `EdmReport.diel_*` fields, converting to the WebUI keys: `present`, `pump_on`, `flush_level`, `flush_bar=flush_mbar/1000`, `flow_lpm=flow_clpm/100`, `temp_c=temp_dC/10`, `temp_set=temp_set_dC/10`, `conductivity_us`, `level_pct`, `filter_pct`. (JSONencoder is int-only → emit `flush_bar`/`flow_lpm`/`temp_c` as the same bare-number trick used for the existing float fields.)
- [ ] **Step 2:** `EdmSpindle.h/.cpp` — add a `dielectric:` config section (`use_sim, required, flow_min_clpm, level_min_pct, conductivity_warn_uS, temp_setpoint_dC, default_flush_level`) parsed in `group()`. In `init()`, construct `SimDielLink` (when `use_sim`/no CAN) or `CanDielLink` over the shared `CanBus`; call `_ctl->attachDielectric(_diel.get(), interlock_from_cfg)`. When the CAN path lands, set the bus RX handler to fan out to **both** links: `bus.onReceive([p,d](const CanFrame& f){ p->onFrame(f); d->onFrame(f); })`. Start a ~10 Hz tick for `SimDielLink` (or fold its `tick()` into the existing servo/report cadence).
- [ ] **Step 3:** `EdmCommands.cpp` — add `$EDM/Flush=Off|Low|Medium|High` and `$EDM/Pump=on|off` → build a `SetDiel` and call the spindle's diel link; include the dielectric block in `$EDM/Status`.
- [ ] **Step 4:** Confirm native `*Edm*` suite still green (these files aren't host-built). **Step 5: Commit** `feat(edm): dielectric telemetry emit + spindle wiring + $EDM/Flush|Pump (ESP32)`.

---

## Task D5 [B]: On-target bench acceptance (when the module exists)

No code. Record results in the PR. With `dielectric.use_sim=false` and a real module on the bus:
- [ ] Discovery: `DIEL_STATUS` heartbeat seen; `$EDM/Status` shows the dielectric block; protocol-version checked.
- [ ] **Interlock:** `M3` while flushing off / tank low → cut refuses to start (`DielectricNotReady`); with flushing on + level OK → cut proceeds.
- [ ] **Mid-cut loss:** kill flow during a cut → `DielectricLost` fault, feed-hold + spark-off.
- [ ] `$EDM/Flush=` changes pressure/flow on the module; temperature holds setpoint; conductivity-high raises the warning (not a fault).
- [ ] PSU 1 kHz telemetry unaffected by the dielectric traffic on the shared bus.

---

## Exit criteria
- `pio test -e tests -f '*Edm*'` green: DielProtocol + DielLink + DielController tests + all prior; ≥80% on new logic.
- Interlock blocks `startCut` without flushing and faults on mid-cut loss (host, via `SimDielLink`).
- `EdmReport.diel_*` populated; `EdmReportChannel` emits the `dielectric` object the WebUI already renders; `$EDM/Flush|Pump` route to the link.
- Two links share one `CanBus` (ID-range fan-out); `IPsuLink`/PSU path unchanged.

## Self-review notes
- **Spec coverage:** catalog (D1), links + sim (D2), interlock + telemetry (D3), emit + wiring + commands (D4), bench (D5). The board + its MCU firmware are out of scope (separate project) per the spec §1/§3.
- **Mirrors:** DielProtocol↔Psu/Protocol, CanDielLink↔PsuLink (incl. `EdmLock`), SimDielLink↔SimPsuLink, the interlock reuses the existing `Armed→TouchOff` gate + fault machinery.
- **No new UI work:** the WebUI dielectric panel + `$EDM/Flush=` already consume this exact contract.
- **Placeholders:** D1–D3 give the dielectric-specific structs/fields/interlock in full; the codec/session bodies that are byte-identical in pattern to the committed PSU code are specified by exact offsets + "mirror Psu/Protocol" rather than re-pasted — the live PSU module is the reference. D4–D5 are ESP32/bench (verified by firmware build), consistent with how P0–P3 handled hardware-coupled tasks.
