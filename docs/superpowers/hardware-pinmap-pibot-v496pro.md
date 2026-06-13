# PiBot FluidNC V4.96 Pro — Pin Audit & Proposed EDM Pin Map

**Status of this document:** research + feasibility assessment. No code.
**Date:** 2026-06-13
**Board:** PiBot FluidNC ESP32 GRBL Laser CNC Controller **V4.96 Pro** (ESP32‑WROOM, 4‑layer PCB, 6 external stepper-driver headers, FluidNC "6x" pin configuration).

---

## 1. Headline feasibility verdict

**YES — there is enough I/O, with high confidence on the architecture but pin numbers requiring hardware confirmation.**

The single most important finding that resolves this project's highest-risk open question:

> **On this board the 6 stepper motors do NOT consume native ESP32 GPIO.** All step/direction/enable signals are driven through an **I2S shift‑register chain (`I2SO.x` virtual pins)**, clocked out over just **3 native GPIO** (`gpio.22` BCK, `gpio.21` DATA, `gpio.17` WS). The shift register exposes ≥32 virtual output pins, of which only ~24 are used by the 6 motors + user outputs.

Consequences for the EDM build:

- The **2 spare motor headers** (axes A/B in the stock 6‑axis map) are already wired to `I2SO` pins and free terminal blocks — the two wire‑feed steppers drop straight in with **zero additional native GPIO cost**. This is the cleanest possible outcome.
- A **shared hardware SPI bus is broken out** (`gpio.18` SCK, `gpio.23` MOSI, `gpio.19` MISO) and is currently used only by the SD card (`gpio.5` CS). The **MCP2518FD CAN‑FD controller can join this SPI bus**; it needs 2 dedicated native GPIO for **CS** and **INT**.
- The **HX711** needs 2 native GPIO (DOUT in, SCK out), bit‑banged.

So the native‑GPIO budget the EDM hardware actually has to find is small: **CS + INT + HX711_DOUT + HX711_SCK = 4 native GPIO** (the 2 steppers cost 0). That budget exists, *but* free general-purpose native GPIO on this board is genuinely scarce once spindle/laser/RS485/relay/OLED functions are accounted for, and several candidate pins are ESP32 strapping or input‑only pins. **This is the real constraint, not the motor count.** See §4.

**Confidence:** The *architecture* (I2S motors, shared SPI+SD, 6 headers) is well corroborated across the vendor pages and the official FluidNC 6‑Pack External config. The *exact native GPIO numbers* are taken from the official FluidNC `6_Pack_External` YAML, which PiBot states its board follows ("adopts the pin configuration of FluidNC 6x"). Treat every native GPIO number below as **VERIFY ON HARDWARE** until checked against the physical board's silkscreen/schematic — PiBot has not published a full machine-readable schematic, and V4.96 Pro added optimized signal routing that *may* reassign a pin or two relative to the generic 6‑Pack.

---

## 2. Known / best-effort GPIO assignments

Source basis: PiBot product + wiki pages for the V4.96 Pro, and the canonical FluidNC **6‑Pack External** config YAML (`6_Pext_XYZABC.yaml`), which the PiBot board is documented to follow. Native ESP32 GPIO are `gpio.N`; shift‑register virtual pins are `I2SO.N`.

### 2.1 Native ESP32 GPIO

| GPIO | Function (stock) | Direction | Notes / confidence |
|------|------------------|-----------|--------------------|
| `gpio.22` | I2S **BCK** (shift-register bit clock) | out | Core — drives all motor signals. Do **not** repurpose. |
| `gpio.21` | I2S **DATA** | out | Core — do not repurpose. |
| `gpio.17` | I2S **WS** (latch) | out | Core — do not repurpose. |
| `gpio.18` | **SPI SCK** | out | Shared SPI bus (SD card today). Reusable for MCP2518FD. |
| `gpio.23` | **SPI MOSI** | out | Shared SPI bus. |
| `gpio.19` | **SPI MISO** | in | Shared SPI bus. |
| `gpio.5`  | **SD card CS** | out | ESP32 strapping pin; already committed to SD. |
| `gpio.16` | **Probe** input | in | Stock probe pin. Could be repurposed if probe unused on a wire-EDM (the EDM "touch" sense is the gap, not a probe). |
| `gpio.34` | X limit (`limit_neg`) | in (input‑only) | Input‑only pin (34–39 cannot be outputs, no pull‑ups). |
| `gpio.35` | Y limit | in (input‑only) | Input‑only. |
| `gpio.32` | Z limit | in | |
| `gpio.33` | A limit | in | Used by wire-feed axis if homed; usually not for feed/tension. |
| `gpio.36` | B limit | in (input‑only) | Input‑only (a.k.a. SVP/SENSOR_VP). |
| `gpio.39` | C limit | in (input‑only) | Input‑only (a.k.a. SVN). |
| `gpio.25` | PWM/spindle output **or** OLED SCL **or** I/O‑expander TXD | out | Multiplexed by jumper/config. Vendor lists it for OLED SCL and for the STM32 I/O expander UART. |
| `gpio.27` | OLED SDA **or** I/O‑expander RXD | i/o | Multiplexed. |
| `gpio.26` | **Relay** output (jumper‑selectable to input) | out (default) | Default relay; can be freed if relay unused. |
| `gpio.13` | 0–10V spindle (DAC/analog path) | out | Per V4.96 Pro spindle wiki. |
| `gpio.12` | Laser PWM output | out | ESP32 strapping pin (boot voltage select) — caution if reused. |
| `gpio.4`  | RC‑servo / BESC output ("test" header) | out | Listed as a test/aux output. Strong free‑pin candidate. |
| `gpio.2`  | (referenced; PWM enable in 6‑Pack YAML) | out | Strapping pin (boot mode). In `6_Pext` YAML it is `PWM.enable_pin`. |
| `gpio.14` | RS485 (UART) | i/o | RS485 spindle comms. |
| `gpio.15` | RS485 (UART) | i/o | ESP32 strapping pin. |
| `gpio.0`  | (referenced; boot/strapping) | — | Strapping pin — avoid. |

> **Caveat:** `gpio.25` appears in different vendor docs as PWM output, OLED SCL, *and* I/O‑expander TXD; `gpio.27` as OLED SDA / expander RXD. These are mux/jumper‑configured roles, not simultaneous. Which role is active depends on the chosen config and jumpers — **VERIFY ON HARDWARE**.

### 2.2 Shift-register (I2SO) virtual pins — stock 6‑axis use

| I2SO | Function | | I2SO | Function |
|------|----------|---|------|----------|
| I2SO.0 | X disable | | I2SO.12 | A direction |
| I2SO.1 | X direction | | I2SO.13 | A step |
| I2SO.2 | X step | | I2SO.15 | A disable |
| I2SO.4 | Y direction | | I2SO.16 | B disable |
| I2SO.5 | Y step | | I2SO.17 | B direction |
| I2SO.7 | Y disable | | I2SO.18 | B step |
| I2SO.8 | Z disable | | I2SO.20 | C direction |
| I2SO.9 | Z direction | | I2SO.21 | C step |
| I2SO.10 | Z step | | I2SO.23 | C disable (also cited as a relay trigger) |
| | | | I2SO.24–27 | `user_outputs` digital0–3 |

Axes **A and B (and C)** are the spare headers. Wire‑feed + wire‑tension steppers map onto **A and B** with no native‑GPIO impact.

---

## 3. Proposed EDM pin map

| EDM peripheral | Signal | Proposed pin | Type | Rationale / risk |
|----------------|--------|--------------|------|------------------|
| **Wire‑feed stepper** | step/dir/disable | `I2SO.13 / I2SO.12 / I2SO.15` (axis **A** header) | shift‑reg | Spare motor header. No native GPIO. **Lowest risk.** |
| **Wire‑tension stepper** | step/dir/disable | `I2SO.18 / I2SO.17 / I2SO.16` (axis **B** header) | shift‑reg | Spare motor header. No native GPIO. **Lowest risk.** |
| **MCP2518FD CAN‑FD** | SCK | `gpio.18` (shared) | SPI | Join existing SPI bus. |
| | MOSI | `gpio.23` (shared) | SPI | Shared with SD. |
| | MISO | `gpio.19` (shared) | SPI | Shared with SD. |
| | **CS** | `gpio.4` (RC‑servo/test header) — *candidate* | native out | Needs a free output. `gpio.4` is the best documented free output. **VERIFY.** Fallbacks: `gpio.26` (if relay unused), `gpio.13` (if 0–10V unused). |
| | **INT** | `gpio.16` (probe) — *candidate* | native in | Probe likely unused on wire‑EDM. Input‑capable, interrupt‑capable. **VERIFY.** Fallback: a freed input‑only pin (`gpio.34/35` if those limits go unused). |
| **HX711 load cell** | DOUT (data in) | `gpio.35` or `gpio.36`/`gpio.39` (a freed input‑only limit pin) | native in | Input‑only is fine for DOUT. Requires giving up a limit input. **VERIFY which limits the EDM actually uses.** |
| | SCK (clock out) | `gpio.26` (relay, if unused) or `gpio.13` | native out | Must be an output‑capable pin (so NOT 34–39). |

**Net native‑GPIO demand beyond stock motion:** CS (1 out) + INT (1 in) + HX711 DOUT (1 in) + HX711 SCK (1 out) = **2 outputs + 2 inputs**. Achievable by reclaiming pins from unused stock functions (probe, relay, one spindle path, and 1–2 limit inputs), provided the wire‑EDM does not need those functions.

> All native GPIO selections above are **proposals**, not confirmed-free pins. Confirm against the physical board and a working FluidNC config dump before committing PCB/wiring.

---

## 4. Conflicts & risks

1. **Native GPIO scarcity is the true bottleneck (not motor count).** The ESP32‑WROOM exposes ~25 usable GPIO; on this board a large share is already committed (3× I2S, 3× SPI, SD CS, 6 limit inputs, probe, plus the spindle/laser/RS485/relay/OLED block). Finding 2 spare output‑capable pins + 2 input‑capable pins requires **repurposing stock functions** the EDM doesn't use (probe, relay, one spindle output path, 1–2 limits). **MEDIUM risk** — feasible but tight.

2. **SPI is shared SD ↔ MCP2518FD.** Bus is fine to share (distinct CS), **but**: (a) FluidNC's SD driver and a CAN driver must cooperate on the SPI peripheral — there can be transfer‑timing/contention issues; (b) FluidNC has historically been finicky about SPI/SD on this hardware. If conflict arises, **drop the SD card** (CS `gpio.5` frees up too) or move CAN to the second VSPI/HSPI controller. **MEDIUM risk.**

3. **Strapping / input‑only pin hazards.** Several attractive-looking "free" pins are ESP32 **strapping pins** (`gpio.0, 2, 5, 12, 15`) — pulling them at boot can brick the boot sequence — or **input‑only** (`gpio.34–39`, no output drive, no internal pull‑ups, so HX711 **SCK cannot** live there and any input needs an external pull‑up). The proposed map avoids strapping pins for new signals, but every choice must be checked. **MEDIUM risk.**

4. **No official full schematic published.** PiBot publishes product pages, a wiki, and example YAMLs but (as of this research) not a complete machine‑readable schematic/pinout for V4.96 Pro. The **V4.96 Pro added "optimized signal design,"** so it may differ from the generic 6‑Pack in a pin or two. All native numbers are best‑effort. **MEDIUM–HIGH uncertainty on exact numbers**, LOW on architecture.

5. **Pin‑role multiplexing.** `gpio.25/27` (PWM vs OLED vs I/O‑expander UART) and `gpio.26` (relay vs input) are jumper/config‑selected. A wrong assumption here could silently steal a pin you planned to use. **VERIFY jumper state.**

6. **Escape hatch exists if GPIO truly runs out:** PiBot sells an **STM32 I/O Expander (RJ12 UART, +18 I/O)** using just 2 ESP32 pins (`gpio.25/27`). If the native budget can't cover CS/INT/HX711, peripherals (especially the load cell, latency‑tolerant) could move behind the expander. Note this *consumes* the OLED/UART pins. This is why the verdict is "yes" rather than "yes‑with‑mandatory‑adapter" — the adapter is a fallback, not a requirement. **LOW risk (mitigation available).**

---

## 5. Open items to confirm on physical hardware

- [ ] Obtain/measure the **actual V4.96 Pro pinout** (silkscreen + continuity) and diff against the generic FluidNC 6‑Pack External map used here.
- [ ] Confirm I2S pins are exactly `gpio.22/21/17` (BCK/DATA/WS) on V4.96 Pro.
- [ ] Confirm SPI pins `gpio.18/23/19` and SD CS `gpio.5`; confirm the SPI header is physically broken out to a connector usable for the MCP2518FD.
- [ ] Confirm the **A and B motor headers** are wired to `I2SO.12/13/15` and `I2SO.16/17/18` respectively, and that their enable lines and 5V step/dir levels suit the wire‑feed drivers.
- [ ] Identify **which limit inputs the wire‑EDM will actually use**; the rest become candidate HX711/INT inputs. Remember `gpio.34–39` are input‑only and need external pull‑ups.
- [ ] Confirm a free, **output‑capable, non‑strapping** GPIO is genuinely available for **CS** (test `gpio.4`) and for **HX711 SCK** (test `gpio.26`/`gpio.13` after disabling relay/0–10V).
- [ ] Confirm **probe pin `gpio.16`** is unused by the EDM and usable as MCP2518FD **INT** (interrupt‑capable, with pull‑up).
- [ ] Verify jumper/config state of `gpio.25/27` (PWM/OLED/expander) and `gpio.26` (relay/input) so a planned pin isn't already claimed.
- [ ] Bench‑test **SD + MCP2518FD coexistence on the shared SPI bus**; if unstable, decide between dropping SD or using the second SPI controller.
- [ ] Decide whether to keep the **STM32 I/O Expander** in reserve as the overflow path for the load cell.

---

## 6. Sources

- [PiBot FluidNC ESP32 GRBL Laser CNC Controller V4.96 Pro — product page](https://www.pibot.com/pibot-fluidnc-grbl-cnc-controller-v4-9)
- [PiBot Wiki — V4.96 Pro: Connecting Spindles (GPIO.13 0‑10V, GPIO.12 laser PWM, GPIO.4 RC‑servo, GPIO.26 relay)](https://wiki.pibot.com/doku.php?id=pibot_cnc_laser_series:v496_pro:connect_spindles:start)
- [PiBot Wiki — start / index](https://wiki.pibot.com/doku.php?id=start)
- [PiBot blog — "V4.96 Pro optimized signal design details"](https://www.pibot.com/blog/v496-pro-signal-design-optimizations)
- [Elecrow — PiBot FluidNC V4.9PB listing (I2S motor outputs, MS3 pin, GPIO 25/27 OLED)](https://www.elecrow.com/pibot-fluidnc-grbl-cnc-controller-v4-9.html)
- [PiBot — V4.9 Plus B 2025 documentation](https://www.pibot.com/V4-9-Plus-B-2025-Version-old-Documentation)
- [PiBot STM32 I/O Expander for FluidNC V4 (RJ12 UART, +18 I/O, uses gpio.25/27)](https://www.pibot.com/pibot-stm32-io-expander-for-fluidnc-v4)
- [FluidNC official config — `6_Pack_External/6_Pext_XYZABC.yaml` (authoritative native GPIO + I2SO map)](https://github.com/bdring/fluidnc-config-files/blob/main/contributed/6_Pack_External/6_Pext_XYZABC.yaml)
- [FluidNC config repo root](https://github.com/bdring/fluidnc-config-files)
- [FluidNC firmware](https://github.com/bdring/FluidNC)
- [PiBot FluidNC V4.9 on Tindie](https://www.tindie.com/products/pibot/pibot-fluidnc-esp32-grbl-laser-cnc-controller-v49/)
