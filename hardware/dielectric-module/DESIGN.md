# Microspark Dielectric Controller — Board Design (MS-DIEL-CTRL)

> **Status: AI-generated first-pass design. NOT fabrication-ready.** Every part value,
> pin assignment, and footprint must be reviewed against the real component datasheets
> and bench-validated before ordering. Decisions marked **[REVIEW]** are engineering
> choices made autonomously that you should confirm.

## 1. Purpose
A dedicated CAN-FD node that runs the EDM dielectric (coolant) loop and reports to the
PiBot over the bus. It implements the firmware contract already frozen in
`docs/superpowers/specs/2026-06-14-dielectric-module-design.md` (CAN IDs **0x710–0x740**):
receives `SET_DIEL` (pump on/off, flush level, temp setpoint, deioniser enable), drives
the pump/flush/chiller/deioniser, senses flow / pressure / temperature / level /
conductivity, and emits `DIEL_STATS` (~10 Hz), `DIEL_STATUS` (heartbeat), `DIEL_FAULT`,
and `ACK_DIEL`.

## 2. Key decisions [REVIEW]
| # | Decision | Rationale | Alternative |
|---|----------|-----------|-------------|
| D1 | **MCU = ESP32-S3-WROOM-1** (N4, no PSRAM needed) | Reuses the existing FluidEDM/Arduino-ESP32 codebase and the already-written MCP2518FD driver; native USB for programming; plenty of GPIO/ADC | STM32G0/G4 (native FDCAN, cheaper, but new firmware stack) |
| D2 | **CAN-FD = external MCP2518FD + TJA1051T/3 transceiver** over SPI | Classic/S3 ESP32 has no native CAN-FD; mirrors the PiBot's own CAN front-end exactly (same driver, same bring-up) | STM32 with native FDCAN |
| D3 | **Input power = 24 VDC** from the machine supply | Common industrial rail; drives 24 V solenoids directly; one buck to 5 V, LDO to 3.3 V | 12 V in |
| D4 | **Pump + chiller = dry-contact SSR/relay outputs** (board switches a contactor coil / SSR input, NOT the motor directly) | Pump/chiller are mains AC and high power — must NOT be switched on this logic board; keep mains off-board | On-board relays (rejected: mains creepage on a logic PCB) |
| D5 | **Flush valves = 24 V solenoids via low-side N-MOSFET** (3 channels: low/med/high or proportional) | Solenoids are inductive 24 V loads; low-side MOSFET + flyback diode is standard | High-side smart switch |
| D6 | **Conductivity = AC-excited 2-electrode cell**, ~1 kHz bipolar drive, synchronous rectify → ADC | DC excitation electroplates/polarises the electrodes and reads wrong; AC is mandatory for dielectric/DI water | Commercial 4-20 mA conductivity transmitter (then this board just reads analog — simpler, **strongly consider**) |
| D7 | **Temp = 2× DS18B20 (1-Wire)** for tank + return | Digital, no ADC calibration, daisy-chainable on one GPIO, ±0.5 °C | NTC + divider into ADC |
| D8 | **Flow = Hall pulse flow sensor** into a GPIO edge counter (RC-filtered) | Cheap, robust, digital | Analog turbine / ultrasonic |
| D9 | **Level = analog (eTape/pressure) + 2 digital float switches** (low-low and high) | Analog for the % gauge, floats for hard interlocks/redundancy | Single capacitive sensor |
| D10 | **Pressure = ratiometric 0.5–4.5 V transducer** into ADC via divider | Standard automotive/industrial pressure sender interface | 4–20 mA loop |
| D11 | **4-layer PCB** (Sig / GND / PWR / Sig) | Clean ground for the analog conductivity front-end + CAN; modest area | 2-layer (cheaper, noisier analog) |

> **D6 note:** the single biggest simplification available is to buy a packaged 4–20 mA or
> 0–5 V conductivity transmitter and have this board just read its analog output (like the
> pressure sender). That removes the trickiest analog section. The design below keeps the
> discrete AC front-end but isolates it to one block so it can be swapped for a 1-pin ADC read.

## 3. Block diagram
```
 24V IN ─┬─ EMI/TVS ─┬─ Buck (24→5V, 2A) ─┬─ 3.3V LDO ─── MCU / MCP2518FD / sensors
         │           └─ 24V rail ───────── solenoid drivers, sensor excitation
         │
 CAN-FD ─┴─ TJA1051 ── MCP2518FD ── SPI ── ESP32-S3 ──┬─ DS18B20 x2 (1-Wire)
                                                      ├─ flow pulse (GPIO cnt)
                                                      ├─ ADC: pressure, level, conductivity, Vsense
                                                      ├─ float switches x2 (GPIO)
                                                      ├─ MOSFET low-side x3  → flush valves
                                                      ├─ SSR/relay drive x3  → pump / chiller / deioniser
                                                      └─ conductivity AC drive + synchro-rectify
```

## 4. Power budget (estimate)
| Rail | Load | Notes |
|------|------|-------|
| 3.3 V | ESP32-S3 (~300 mA peak TX), MCP2518FD (~10 mA), sensors (~50 mA) | LDO from 5 V, ~0.5 A |
| 5 V | transceiver, level/pressure sensor excitation, fan | buck from 24 V, ~1 A |
| 24 V | 3× solenoids (~0.5 A ea), SSR LEDs | sized for ~2 A peak; off-board contactors carry pump/chiller |

## 5. Connectors
| Ref | Type | Pins | Function |
|-----|------|------|----------|
| J1 | Phoenix 5.08 mm, 2P | 2 | 24 VDC input (+ , GND) |
| J2 | Phoenix 5.08 mm, 3P | 3 | CAN-FD (CANH, CANL, GND) + term jumper JP1 |
| J3 | Phoenix 3.5 mm, 6P | 6 | Flush solenoids (3× drive + 24 V commons) |
| J4 | Phoenix 3.5 mm, 6P | 6 | SSR/relay outputs: pump, chiller, deioniser (drive + GND) |
| J5 | JST-XH 6P | 6 | Analog sensors: pressure, level, conductivity-cell (2), +5 V, GND |
| J6 | JST-XH 4P | 4 | 1-Wire temp (DS18B20 ×2): 3.3 V, DATA, GND, spare |
| J7 | JST-XH 4P | 4 | Flow sensor + 2 float switches: 5 V, FLOW, FLOAT_LO, FLOAT_HI, GND |
| J8 | USB-C | — | ESP32-S3 native USB (program/debug) |

## 6. Core nets (the connectivity the PCB is built from)
Logic: `+3V3`, `+5V`, `+24V`, `GND`, `AGND` (star-tied to GND under the MCU).
SPI: `SPI_SCK`, `SPI_MOSI`, `SPI_MISO`, `MCP_CS`, `MCP_INT`.
CAN: `CANH`, `CANL`, `MCP_TXCAN`, `MCP_RXCAN` (MCP2518FD↔transceiver), `CAN_TERM`.
Sensors: `FLOW_PULSE`, `TEMP_1W`, `PRESS_AIN`, `LEVEL_AIN`, `COND_AIN`, `FLOAT_LO`, `FLOAT_HI`, `COND_DRV`.
Drives: `VLV_LOW`, `VLV_MED`, `VLV_HIGH` (gate nets → solenoid outputs `SOL1..3`), `SSR_PUMP`, `SSR_CHILL`, `SSR_DEION`.
MCU housekeeping: `EN`, `IO0` (boot), `TXD0`, `RXD0`, `USB_DP`, `USB_DM`.

A full pin-by-pin net table is emitted by the generator (`generate_board.py`) and exported
to `MS-DIEL-CTRL.net` for review.

## 7. Firmware ↔ hardware mapping (the CAN contract)
| Contract field (`DielStats`) | Hardware source |
|------------------------------|-----------------|
| `flow_clpm` | FLOW_PULSE Hz × K-factor |
| `flush_mbar` | PRESS_AIN (transducer) |
| `temp_dC`, return | DS18B20 #1 / #2 |
| `level_pct` | LEVEL_AIN; `FLOAT_LO/HI` set low-level fault + high warning |
| `conductivity_uS` | COND front-end |
| `pump_on`/`flush_level` | echo of the drive state |
| flags bit2 low_level | `FLOAT_LO` |
| `SET_DIEL` → drives | VLV_*, SSR_* |

## 8. What this board deliberately does NOT do
- It does **not** switch mains pump/chiller motors directly — only their contactor/SSR coils.
- It does **not** generate the EDM spark (that's the PSU board).
- No on-board high voltage; all rails ≤ 24 V.

## 9. Open items for review
- D6 conductivity: discrete AC front-end vs. packaged transmitter.
- Solenoid count/type: 3 discrete flush levels vs. one proportional valve (PWM + current sense).
- Whether the chiller is on this loop at all (some machines use ambient/return-line cooling only).
- Fusing/TVS rating on 24 V in; reverse-polarity protection (P-MOSFET) — included in the design.
