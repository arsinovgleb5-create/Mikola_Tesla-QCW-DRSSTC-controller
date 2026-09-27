# QCW DRSSTC controller v5.9 (ESP32) — firmware, display and control board

**Author: Mikola_Tesla · License: [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)** — share and adapt freely, just credit the author (see `LICENSE`).

A hand-held controller ("interrupter + ramp generator") for a QCW DRSSTC Tesla coil.
An ESP32 DevKit generates the ramp envelope for the buck modulator and the interrupter
gate signal, reads seven knobs and three buttons, and shows the ramp shape on a small
touch-screen board (ESP32-2432S028R, "Cheap Yellow Display"). Both signals leave the
controller over plastic optical fiber (HFBR transmitters), so the controller has no
galvanic connection to the bridge.

Everything here is hand-made-friendly: a single-sided PCB for toner transfer, through-hole
resistors, DIP drivers, wide tracks and holes.

```
firmware/
  generator/QCW_GENERATOR_ESP32/QCW_GENERATOR_ESP32.ino   ramp generator + interrupter (ESP32 DevKit V1, 30-pin)
  display/cyd_ramp_view/cyd_ramp_view.ino                 ramp viewer for the CYD display (I2C slave)
  display/cyd_ramp_view/User_Setup_CYD.h                  TFT_eSPI configuration for the CYD board
hardware/control_board/
  kicad/                                                  KiCad 10 project (schematic + PCB + own libraries)
  output/schematic.pdf, schematic.png                     schematic, rev. E
  output/bottom_copper_1to1.pdf (+ _mirrored)             copper artwork 1:1 for toner transfer
  output/top_assembly.pdf, bottom_assembly.pdf            part placement, jumper wires (red), labels
  output/top_silkscreen_1to1.pdf, drill_map.pdf           optional silkscreen print, drill map
  output/gerbers.zip                                      Gerber + Excellon in case you order the board
  output/BOM.csv, jumper_wires.json                       parts list, jumper wire list
  output/3d_top.png, 3d_bottom.png, bottom_copper.png     pictures
```

## 1. How the system is put together

```
 knobs (7 pots) ----\
 FIRE / AUTO -------+--> ESP32 DevKit --> TC4420 U2 --> fiber TX "RAMP"  ~~~~> driver board (buck modulator)
 E-STOP (2 groups) -/        |     \---> TC4420 U1 --> fiber TX "INT"   ~~~~> driver board (bridge enable)
                             |  I2C
                             v
                    CYD display (ramp preview, status, supply voltmeter)
```

* **Ramp** — PWM on GPIO19 (MCPWM, 25 kHz carrier by default, `PWM_TOP`). The duty cycle follows the
  envelope: a "wick" step at the start, a rise to the peak, a knee, then a fall. The buck modulator on the
  driver board turns this PWM into the bus voltage envelope.
* **Interrupter** — GPIO4, enables the bridge for the duration of the pulse.
* **One mode**: single QCW ramp pulses, fired by FIRE or repeated by AUTO at the BPS rate (0.5 … 15 pulses/s).
* **E-STOP** — one mushroom switch with two contact groups. Group A (normally open) pulls GPIO23 to ground;
  that pin is also the MCPWM hardware fault input, so the ramp stops in hardware before software reacts.
  Group B (normally closed) is wired in series with the interrupter driver input and cuts the bridge enable
  signal physically.
* **Display link** — I2C, 250 kHz (`LINK_FREQ_HZ`, the same value in both sketches; lower it if your cable is
  long), the generator is the master, the display is a slave at address 0x2A. The link is one-way: the display
  only draws what it receives. Without a STATUS frame for 2 s the display shows OFFLINE / NO LINK.
* **Supply voltmeter** — the display shows the controller 5 V rail (divider on the control board), so a sagging
  battery/buck output is visible before the ESP32 browns out.

## 2. Control board

Single-sided, 85 x 72 mm, all copper on the bottom. Tracks 0.8 mm, clearance 0.6 mm, ground pour on the bottom
with thermal reliefs. Holes: pin headers 1.1 mm, DIP/resistors 1.0 mm, screw terminal 1.6 mm, four M4 mounting
holes (4.3 mm) at (5,6), (80,6), (4.5,67.5), (81,63) mm from the top-left corner.

Parts on top: ESP32 DevKit V1 30-pin in two 1x15 socket headers, two TC4420 DIP-8 drivers, 13 through-hole
resistors 0.25 W (7.62 mm pitch), pin headers, a 2-way screw terminal. Five capacitors (C8…C12) are SMD 1206
and are soldered on the copper (bottom) side — the only SMD parts. See `output/BOM.csv`.

Where tracks cannot cross on one layer, insulated **jumper wires** run on the top side: 30 short pieces
through 29 holes (0.8 mm), the longest one 26 mm. Both ends of one wire carry the same label on the board
(`W1 GND`, `W2 SCL`, …, `W11 ESTOP_B2`); `output/jumper_wires.json` lists every segment with coordinates and
`output/top_assembly.pdf` shows them in red. Holes labelled with the same `Wn` are joined by one wire, chain style.

Making it by toner transfer: print `bottom_copper_1to1.pdf` at 100 % (it is drawn as seen from the top, so
laying the print face-down on the bottom copper mirrors it correctly; use the `_mirrored` file if your
process needs the opposite). Etch, drill, solder the SMD capacitors first, then the jumper wires, then
resistors, sockets, headers and the terminal. Before inserting the ESP32 check that +5V–GND and 3V3–GND are not
shorted.

### Connectors (all 2.54 mm pin headers except J9)

| Conn. | Function | Pins | Location |
|---|---|---|---|
| J1 | knobs (1x9) | 1 = 3V3 rail, 2 = AMPL, 3 = WICK, 4 = FALL, 5 = U, 6 = PW, 7 = BPS, 8 = T, 9 = GND rail | left edge |
| J3 | AUTO button | 1 = signal (GPIO17), 2 = GND | top edge |
| J2 | FIRE button | 1 = signal (GPIO16), 2 = GND | top edge |
| J4 | E-STOP group A (NO) | 1 = GPIO23 (ESTOP_A), 2 = GND | top edge |
| J5 | display (1x5) | 1 = GND, 2 = SCL, 3 = SDA, 4 = VM (voltmeter), 5 = +5V | top edge |
| J8 | E-STOP group B (NC) | 1, 2 = in series with U1 input (no polarity) | bottom edge |
| J7 | fiber TX "INT" (bridge enable) | A = anode, K = cathode | bottom edge |
| J6 | fiber TX "RAMP" (buck modulator) | A = anode, K = cathode | bottom edge |
| J9 | +5 V input, screw terminal | 1 = +5V, 2 = GND | bottom-right corner |

Every pin function is printed on the board next to the pin.

## 3. Wiring guide (what goes where)

**Knobs.** Seven 10 k linear pots, no extra resistors. The 3V3 rail (J1.1) goes to the first pot and is
daisy-chained from pot to pot; the GND rail (J1.9) the same. Each wiper goes with one wire straight to its
J1 pin. Knob → pin → ESP32 GPIO:

| Knob | J1 pin | GPIO | Meaning |
|---|---|---|---|
| AMPL | 2 | 34 | peak power of the ramp |
| WICK | 3 | 35 | starting step ("wick") at the beginning of the ramp |
| FALL | 4 | 32 | fall of the ramp after the peak |
| U | 5 | 33 | knee position on the power axis |
| PW | 6 | 25 | pulse width |
| BPS | 7 | 26 | pulse repetition rate in AUTO |
| T | 8 | 27 | knee position on the time axis |

**Buttons.** FIRE to J2 and AUTO to J3, one contact to the signal pin, the other to GND; pull-ups are inside
the ESP32. E-STOP: contact group A (normally open) to J4, group B (normally closed) to J8, two separate 2-wire
cables.

**Display (CYD).** Five wires from J5:

| J5 | → CYD | Note |
|---|---|---|
| 1 GND | CN1.GND | common ground |
| 2 SCL | CN1.IO22 | 4.7 k pull-ups to 3.3 V are on the control board |
| 3 SDA | CN1.IO27 | |
| 4 VM | P3.IO35 | voltmeter input, divider 10 k / 10 k on the control board |
| 5 +5V | CYD 5 V (USB connector or VIN pad) | |

CN1.3V3 is not used.

**Fiber transmitters (HFBR-1412 / HFBR-1414).** Two wires per transmitter to J6 (RAMP) and J7 (INT):
A = anode = transmitter pins 2, 6 and 7 tied together; K = cathode = transmitter pin 3. Pins 1, 4, 5, 8 are
unused. The transmitter is just an LED: ~60 mA comes from the TC4420 through two 100 ohm resistors in parallel,
no separate supply. Keep these wires short and twisted. On the driver board the HFBR-2412 receivers are powered
from that board (pin 2 = Vcc 5 V, pins 3 and 7 = common, pin 6 = output); light = bridge enabled, a broken
fiber = dark = inhibit.

**Power.** Any clean 5 V source feeds the screw terminal J9 (1 = +5V, 2 = GND); the author uses a buck
 converter from a 12 V battery. On the board
5 V goes to the ESP32 VIN, to both TC4420 and to the display (J5.5). 3.3 V for the knobs and the I2C pull-ups
comes from the ESP32 module regulator.

**Grounds.** All grounds meet on the board's ground pour: buck minus (J9.2), display (J5.1 only), knobs
(J1.9 rail), buttons, transmitter cathodes. Do not run extra ground wires between external parts. The
controller must never share ground with the bridge or the buck modulator — that is what the fiber is for. If
the enclosure is metal, connect it to J9.2 with a single wire.

## 4. Building and flashing the firmware

Both sketches are for the **Arduino IDE / arduino-cli with the ESP32 Arduino core 3.x** (the author builds with
3.3.x; a few comments in the code describe behaviour observed with that version).

### Generator (`QCW_GENERATOR_ESP32.ino`)

* Board: "ESP32 Dev Module". CPU frequency 240 MHz, partition scheme "Huge APP", flash frequency 80 MHz.
  The firmware checks the CPU frequency at start and refuses to run at the wrong one.
* No external libraries; the MCPWM driver and FreeRTOS come with the core.
* Pin map is in the `PIN DEFINITIONS` section of the sketch. Where to change things: carrier frequency —
  `PWM_TOP` (16 MHz / PWM_TOP, 640 = 25 kHz). A higher carrier makes the ramp smoother (less choke ripple,
  finer envelope steps in time) but loads the gate driver, the gate DC-DC and the IGBT brick harder — switching
  losses grow linearly with frequency. The frequency table in the sketch was computed for the author's buck
  (FF200R12KE4 brick, Rg 15 ohm, 650 V, 800 uH choke, 2 W gate DC-DC) — recompute it for your parts; pulse-rate limits — `BPS_MIN_X10` / `BPS_MAX_X10`; bus
  voltage and wick in volts — `BUS_VOLTAGE`, `WICK_VOLTAGE` (the wick duty is computed from them). These two,
  the alpha (bus sag) compensation and the pot end-stop constants are build-specific: set them for your coil.
* The serial monitor (115200) prints the computed configuration at start and every state change
  (ARMED, AUTO, ESTOP, faults).

### Display (`cyd_ramp_view.ino`)

* Board: "ESP32 Dev Module", upload speed **230400**. On the author's CYD the CH340 USB-serial chip fails at
  460800 and above; other batches may accept more, but 230400 always works.
* Library: TFT_eSPI 2.5.x. Copy `User_Setup_CYD.h` over `Arduino/libraries/TFT_eSPI/User_Setup.h` (rename it
  to `User_Setup.h`), the sketch cannot configure the library by itself. The file has `TFT_INVERSION_ON`
  because this panel variant is inverted (white background otherwise); comment it out if your panel shows
  inverted colours.
* The display draws: mode (IDLE / ARMED / AUTO / E-STOP / OFFLINE), LINK OK / NO LINK with the shot and CRC
  error counters, the ramp curve on a fixed 0…25 ms time axis, and under it peak and wick in volts, fall, BPS,
  pulse width, PWM steps. The supply voltage is shown in small font in the top-left block.

### Link protocol (for anyone who wants another display)

Frame: `[type][len][payload…][crc8]`, CRC-8 over type+len+payload, polynomial 0x07, init 0x00, MSB first.

* `0x00 STATUS` (6 bytes): fw_major, fw_minor, flags (0x01 ARMED, 0x02 AUTO, 0x04 ESTOP, 0x08 PREVIEW_READY),
  pulses_lo, pulses_hi, 0. Sent every 100 ms — this is the heartbeat; without it the display shows NO LINK.
* `0x10|gen META` (22 bytes, uint16 little-endian): pulse_width_ms, bps_x10, peak_duty, wick_offset, fall_x100,
  tscale_x100, uscale_x100, pwm_top, bus_voltage, rise_end_x10000, then two bytes: points (80), rise_points (64).
  `gen` is a 3-bit generation counter of the preview.
* `0x40|gen<<3|n CHUNK`: curve points in three chunks of 30, 30 and 20 (n = 0, 1, 2 → points 0–29, 30–59, 60–79),
  each point is the duty 0…255 relative to pwm_top. The time grid is not uniform: the first rise_points (64) points
  cover the rise, the remaining 16 cover the fall.

## 5. Supply voltmeter

R18 (10 k) from +5 V to the VM node and R19 (10 k) from VM to GND on the control board; VM goes through J5.4
to the CYD input IO35 (ADC1, 11 dB attenuation, input only). In `cyd_ramp_view.ino`: `VMETER_DIV = 2.0`
(= (R_top + R_bottom) / R_bottom), green above `VMETER_OK_V` (4.8 V), yellow between, red below
`VMETER_LOW_V` (4.5 V). If the input is below 1 V (no divider connected) nothing is shown. To watch a 12 V
battery instead, use 100 k / 22 k and set `VMETER_DIV` to 5.55.

## 6. Safety

A Tesla coil is lethal. Do not run this without understanding the schematic and the driver-side protections.
The firmware has a task watchdog, forces all outputs low at start-up, checks the hardware fault path of the
MCPWM at boot and refuses to arm if anything is inconsistent — but none of that replaces a proper E-STOP in
the mains path and an isolated, fiber-coupled driver.

## 7. Build configuration

All numbers that depend on your coil are collected in one `BUILD CONFIGURATION` block at the top of
`QCW_GENERATOR_ESP32.ino`: bus voltage, carrier frequency (`PWM_TOP`), ramp ceiling, alpha compensation, wick
voltage and margins, pulse-width and BPS ranges, buck stage timings, knob end stops and the link speed.
Everything else is derived from them by the preprocessor and checked by `static_assert`s — change a value,
rebuild, and the whole firmware follows. The display sketch has the same kind of block for the voltmeter
divider, thresholds, auto refresh and link speed/timeout.

## 8. License

Copyright (c) 2026 Mikola_Tesla. Released under CC BY 4.0: you may share and modify everything here, including
commercially, as long as you credit the author and state your changes. No warranty.
