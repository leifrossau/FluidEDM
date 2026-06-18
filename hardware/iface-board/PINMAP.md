# PiBot V4.96 Pro — Pin / Connector Map (for MS-EDM-IFACE)

> Compiled from two **agreeing PRIMARY vendor sources** (so the electrical map is HIGH confidence)
> plus the repo's `example_configs/wire_edm_4axis.yaml` audit:
> - grblHAL board header `pibot_i2s_6_axis_map.h` (vendor-contributed)
> - vendor FluidNC YAML `V49p-XYZABC-factory-test-V1.yaml`
> - PiBot product/doc pages (pibot.com, wiki.pibot.com)
>
> **Mechanical** details (PCB outline, mounting holes, connector XY, exact rail pads) are NOT
> published → **VERIFY ON HARDWARE**. Treat MS-EDM-IFACE as a **flying-lead / pluggable adapter**
> that taps the PiBot's SPI/module header + screw terminals, **not** a board-to-board stacking shield.

## Key fact: the PiBot is FULLY pin-committed
There are **no free, broken-out, output-capable GPIO**. The adapter must reuse pins the
wire-EDM build does not need.

## Bus / signal map (HIGH confidence)
| Signal | GPIO / I2S | Notes |
|---|---|---|
| SPI SCK | gpio.18 | shared bus (SD + drivers) |
| SPI MOSI | gpio.23 | shared |
| SPI MISO | gpio.19 | shared |
| SD CS | gpio.5 | SD's own CS — **MCP2518FD only needs a NEW CS** |
| I2S BCK/DATA/WS | gpio.22 / 21 / 17 | drives all 6 motor headers |
| Limits X/Y/Z/A/B/C | gpio.35/34/39/36/**32/33** | B/C limits unused in WEDM → free for HX711 |
| Probe | gpio.2 | strapping pin (boot-low) — caution if used as CAN INT |
| Relay out | gpio.26 | candidate CAN CS |
| 0–10 V analog out | gpio.13 | candidate CAN CS/INT |
| Spindle PWM / Laser | gpio.4 / gpio.12 | used by EDM facade — do not steal |

## Adapter pin assignments (what MS-EDM-IFACE taps) — all **[VERIFY ON HARDWARE]**
| Adapter signal | PiBot pin | PiBot terminal it appears on |
|---|---|---|
| SPI SCK / MOSI / MISO | gpio.18 / 23 / 19 | SPI / module header |
| MCP_CS | gpio.26 (relay) *(alt: gpio.13)* | relay terminal |
| MCP_INT | gpio.2 (probe) *(alt: gpio.13)* | probe terminal — **strapping, add series isolation / pick alt** |
| HX711 PD_SCK | gpio.33 | C-axis (M6) limit terminal — output-capable |
| HX711 DOUT | gpio.32 | B-axis (M5) limit terminal — input |
| +3V3 / +5V / GND | rails | confirm tappable pads; logic-level jumper sets driver rail |

Power the MCP2518FD + HX711 **digital** side from **+3V3** (same as ESP32) so MISO/DOUT stay 3.3 V
(ESP32 is not 5 V tolerant). HX711 **analog** (VSUP/AVDD excitation) may run from +5V for better
load-cell SNR — its DVDD (digital I/O) stays at +3V3.

## Wire feed + tension steppers
Use the **PiBot's own motor sockets 5 & 6** (populate StepStick drivers there) — the adapter does
**not** carry stepper drivers. I2S bits (vendor map):
- Motor 5 (feed):    STEP i2so.18, DIR i2so.17, EN i2so.16
- Motor 6 (tension): STEP **i2so.21**, DIR **i2so.20**, EN **i2so.23**

⚠️ **Repo bug to fix:** `wire_edm_4axis.yaml` tension motor uses STEP i2so.22 / DIR i2so.21 /
DIS i2so.20 — **shifted by one** (i2so.22 is the header's spare CS bit, not STEP). Correct to
STEP i2so.21 / DIR i2so.20 / DIS i2so.23 and scope-verify on hardware.

## Other notes
- FluidNC issue #831: enabling I2S BCK on gpio.22 can interact with WiFi — relevant to this board.
- Vendor recommends 47 Ω series R on SPI MOSI/MISO/SCK for signal integrity (keep adapter stub short).

## Sources
- grblHAL `pibot_i2s_6_axis_map.h`; vendor FluidNC `V49p-XYZABC-factory-test-V1.yaml`
- pibot.com V4.9 / V4.9 Plus B docs; wiki.pibot.com I/O Expander V4
