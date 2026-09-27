/*
 * ============================================================================
 *   QCW DRSSTC GENERATOR v5.9  -  controller / interrupter firmware
 *   Author: Mikola_Tesla  -  License: CC BY 4.0 (share and adapt freely, credit the author)
 *   + built-in ramp-shape preview on an external CYD display (ESP32-2432S028R)
 * ============================================================================
 *
 *   WHAT THIS IS
 *   ------------
 *   Firmware for an ESP32 DevKit V1 (30-pin module) that drives a Tesla coil
 *   in QCW DRSSTC mode. It does three things:
 *     1) generates the ENVELOPE (ramp) for the buck modulator - it raises and
 *        lowers the power smoothly within one pulse, so that a long "sword"
 *        arc grows instead of a short discharge;
 *     2) generates the INTERRUPTER signal that enables the bridge;
 *     3) reads 7 potentiometers and sets the shape / repetition rate /
 *        length of the pulses in real time.
 *
 *   ONE OPERATING MODE: QCW - single ramp pulses. The INTERRUPTED and
 *   CW_QCW modes were removed in v5.9 together with the MODE button.
 *
 *   HOW IT WORKS (simplified)
 *   -------------------------
 *   The ramp is output as PWM on GPIO19 (MCPWM, 16 MHz timer, period
 *   PWM_TOP+1 ticks). Varying the PWM duty over time sets the buck output
 *   voltage - that is the envelope shape. GPIO4 (interrupter) enables the
 *   bridge. E-STOP on GPIO23 is also wired to the MCPWM hardware fault input,
 *   so it drops both outputs without a single CPU instruction.
 *   The ramp shape (slope, knee point, wick start) is set by the seven knobs.
 *   A preview of the ramp is sent to the CYD display over hardware I2C
 *   (GPIO21 SDA / GPIO22 SCL, slave address 0x2A) - see EXTERNAL DISPLAY.
 *
 *   WHERE TO CHANGE THE FREQUENCIES  <<<  (see the CONSTANTS section below)
 *   -------------------------------
 *     * Ramp PWM carrier frequency  -> set by PWM_TOP (default 25 kHz).
 *       Higher carrier = smoother ramp, but more load on the gate driver,
 *       the gate DC-DC and the IGBT brick (switching losses grow with f).
 *     * Pulse repetition rate (BPS) -> BPS knob, limits BPS_MIN/MAX_X10.
 *   Both are marked with large comment boxes in the code - search for
 *   the word "FREQUENCY".
 *
 *   WIRING (control board rev. E)  - pins: see the PIN DEFINITIONS section
 *   -----------------------------
 *     J1 knobs 1x9: pin 1 = 3V3 rail (daisy-chained to all pots),
 *        2 = AMPL (GPIO34), 3 = WICK (GPIO35), 4 = FALL (GPIO32),
 *        5 = U (GPIO33, u-scale), 6 = PW (GPIO25, pulse width),
 *        7 = BPS (GPIO26), 8 = T (GPIO27, t-scale), 9 = GND rail.
 *        Each pot: ends to the 3V3/GND rails, wiper straight to its pin,
 *        no extra resistors.
 *     J2 FIRE button (GPIO16 to GND), J3 AUTO button (GPIO17 to GND),
 *     J4 E-STOP contact group A (GPIO23 to GND, normally open, 4.7k pull-up
 *        + 100n on board), J8 E-STOP contact group B (normally closed, in
 *        series with the interrupter driver input).
 *     J5 display cable 1x5: 1 GND, 2 SCL (GPIO22 -> CYD CN1.IO22),
 *        3 SDA (GPIO21 -> CYD CN1.IO27), 4 VM (5 V supply voltmeter divider
 *        10k/10k -> CYD P3.IO35), 5 +5V.
 *     J6 TX RAMP and J7 TX INT: fiber transmitters HFBR-14xx, pin A = anode
 *        (transmitter pins 2, 6, 7 tied), pin K = cathode (pin 3); driven by
 *        a TC4420 through 2x100 ohm in parallel (~65 mA).
 *        GPIO19 ramp PWM -> U2 -> J6, GPIO4 interrupter -> U1 -> J7.
 *     J9 screw terminal: +5 V in from a buck converter (pin 1 = +5V,
 *        pin 2 = GND); feeds ESP32 VIN, both TC4420 and the display.
 *
 *   SAFETY: watchdog timer (WATCHDOG_TIMEOUT_MS, 2 s budget); E-STOP button;
 *   all outputs LOW at start-up. A Tesla coil is dangerous - do not run it
 *   without understanding the circuit and its protection.
 *
 * ============================================================================
 *   CHANGELOG (for the developer)
 * ============================================================================
 *
 * Changes in v5.9:
 * - 2026-09-12: s_last_ready is cleared together with preview_ready (the
 *   ready edge was lost with PREVIEW_BATCH 80 - after turning a knob the
 *   frame set only went out with the heartbeat); ADC end stops are accepted
 *   without hysteresis (25 ms / 50 % / 90 were unreachable); fall protection
 *   on the first preview with E-STOP held down; force self-test under
 *   FORCE_LOCK.
 * - 2026-09-12: preview_invalidate no longer discards a frame set that has
 *   not been fully sent (s_dirty); the recalculation is postponed until it
 *   goes out; heartbeat 1 s. Reason: the knob poll ran before the send and,
 *   with the window closed in AUTO, wiped the set every 50 ms - the display
 *   did not update while a knob was being turned.
 * - 2026-09-12: a 3-bit preview generation was added to the META (0x10|gen)
 *   and chunk (0x40|gen<<3|idx) frame types: the display no longer mixes
 *   chunks of a new shape with an old caption if META was dropped by CRC.
 *   The cyd_ramp_view display firmware from 2026-09-12 also understands the
 *   old types (generation 0).
 * - 2026-09-12: closed the window in which a fault could be lost before the
 *   ramp starts - busy_wait_us checks the fault after a timeout exit too,
 *   fire_pulse re-checks the level and the flags before and after arming
 *   the ramp, on_tez drives the ramp only while the bridge is forced HIGH;
 *   otherwise the ramp ran with the bridge off and Cb charged with no load.
 *   PWM_FREQ_HZ is computed from the PWM_TOP+1 period.
 *   analogSetPinAttenuation replaced by analogSetAttenuation (observed with
 *   ESP32 core 3.3.x: the per-pin call did nothing before the first read).
 * - 2026-09-12: display link sped up. I2C 250 kHz (frame ~1.5 ms instead of
 *   ~3.7, measured on the reference build), the preview is computed in one
 *   pass (PREVIEW_BATCH 80),
 *   TX_GUARD_MS 4, STATUS every 100 ms, pots=/prev= added to the POTS line
 *   (knob poll and preview computation times). Reason: at high BPS the set
 *   of four frames (~15 ms) did not fit into the window between pulses, and
 *   the next knob poll discarded the unsent set - the display hardly updated.
 * - 2026-09-11: pot_hyst - the first poll after power-up is now always
 *   accepted (lr = 0xFFFF). Previously a knob at the lower end stop (ADC = 0)
 *   kept its default value until the first movement: AMPL 320/528 (50 % duty),
 *   PW 10 ms instead of 5, BPS 2.0 instead of 0.5. Dangerous when starting
 *   with the amplitude at zero.
 * - REMOVED the INTERRUPTED and CW_QCW modes and the MODE button. Pure QCW
 *   remains. Removed with them: calculate_cw_duty/start_cw_mode/stop_cw_mode/
 *   handle_cw_mode, the cw_* state, CW_MIN_PEAK_DUTY, current_mode, btn_mode.
 * - IMPORTANT: the E-STOP reset condition in handle_buttons() no longer reads
 *   PIN_BTN_MODE. This is the only removal that touches the protection: a
 *   leftover read of the removed pin would have broken the fault reset (at
 *   that point a 3 s hold; the reset is now release + a second stable press,
 *   see handle_buttons).
 * - The shape math was NOT touched: calculate_duty and calc_curve_deviation
 *   are byte-for-byte the same as in v5.8. The OperationMode enum is kept in
 *   full so that the switch inside calculate_duty need not be edited; only
 *   MODE_QCW is reachable.
 * - Removed the display stub (DummyTFT) and the TFT_CS/TFT_DC/TFT_RST pins.
 * - ADDED ramp-shape output to an external CYD display over I2C (this board
 *   is the bus master). See the EXTERNAL DISPLAY section below.
 *
 * Fixes in v5.8:
 * - BUCK_DISCHARGE_US 1 ms -> 2 ms: the bridge stays enabled longer after the
 *   ramp, so the coil drains the output capacitor Cb more completely. A Cb
 *   still charged at the next pulse = bridge switched onto a charged bus =
 *   dead IGBTs.
 * - MAX_DUTY raised to PWM_TOP (100 % duty), AMPL_POT_MAX 320 -> 330, so that
 *   with alpha compensation the ramp peak lands exactly at 100 % instead of
 *   being clipped. Safe ONLY with an ISOLATED gate-driver supply (not
 *   bootstrap!).
 * - WICK_MIN_DUTY is now COMPUTED by a formula from the bus voltage
 *   (BUS_VOLTAGE) and the desired wick in volts (WICK_VOLTAGE).
 * - outputs_off() and emergency_stop() SPLIT into two functions.
 * - busy_wait_us() latches the fault itself.
 * - The warm-up before a pulse heats the buck on a CLAMPED wick:
 *   min(wick, peak).
 * - Dead code removed from calculate_duty.
 *
 * Safety fixes in v5.7:
 * - E-STOP LATCHES from any point of detection.
 * - The emergency path drops the interrupter output immediately, without the
 *   regular discharge pause.
 * - QCW calculate_duty: added a clamp of wick to peak.
 *
 * From v5.3 (kept):
 * - The ramp pin is never taken over with digitalWrite() while running; the
 *   ramp is stopped through the compare register while the PWM unit stays
 *   attached to the pin, and the output is re-attached before every start.
 *   (Originally a Timer1 COM1A1 fix on the AVR build; the MCPWM port keeps
 *   the same rule - see ramp_off_ll() / ramp_release_ll().)
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "hal/mcpwm_ll.h"
#include "soc/mcpwm_struct.h"
#include "soc/gpio_reg.h"
#include "esp_task_wdt.h"

#include "rtc_wdt.h"
#include "esp_cpu.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ############################################################################
// ##  BUILD CONFIGURATION - the numbers to adapt for YOUR coil               ##
// ##  Everything else in this file is derived from these by the             ##
// ##  preprocessor (duty limits, wick, frequencies, tables, static_asserts). ##
// ##  Change a value here, rebuild, and the whole firmware follows.          ##
// ##  Detailed explanations stay next to the derived macros further down.   ##
// ############################################################################

// --- Buck / bus -------------------------------------------------------------
#define BUS_VOLTAGE         325  // bus voltage, V (reference build: 325 in stage 1, 650 in stage 2)
#define PWM_TOP             640  // buck carrier: 16 MHz / (PWM_TOP+1); 640 = 25 kHz. Higher carrier =
                                 // smoother ramp but more load on the gate driver, gate DC-DC and IGBT brick.
                                 // Also = number of ramp steps. See the FREQUENCY box below.
#define MAX_DUTY        PWM_TOP  // ramp ceiling: 100 % duty is safe ONLY with an isolated gate DC-DC.
                                 // With a bootstrap gate supply use ~60 % of PWM_TOP (e.g. 380 for 640).
#define ALPHA_X1000_DEFAULT 210  // bus-sag compensation x1000 (tuned on the reference build's capacitor bank)

// --- Wick (starting step of the ramp) --------------------------------------
#define WICK_VOLTAGE        40   // desired REAL wick step at the buck output, V
#define WICK_LOSS_COMP_PCT  12   // margin for buck losses, % (raise if the scope shows a lower step)
#define WICK_MAX_PCT_OF_BUS 33   // wick knob ceiling, % of the bus

// --- Pulse timing and repetition -------------------------------------------
#define PULSE_WIDTH_MIN_MS  5    // PW knob range, ms. The display time axis is fixed to the maximum
#define PULSE_WIDTH_MAX_MS  25   // (AXIS_MS_MIN in cyd_ramp_view.ino) - change both if you change this.
#define BPS_MIN_X10         5    // AUTO repetition rate, x10: 5 = 0.5 pulses/s
#define BPS_MAX_X10         150  // 150 = 15 pulses/s. Higher - risk of overheating the coil and the brick.

// --- Buck stage timing (microseconds) ---------------------------------------
#define ENABLE_BUCK_TIMING    1     // 1 = ramp starts on wick BEFORE the interrupter enables the bridge
#define BUCK_WARMUP_US        2000  // ramp on wick, interrupter LOW: the LC filter settles
#define BUCK_ENABLE_DELAY_US  1000  // after interrupter HIGH, before the rise starts
#define BUCK_DISCHARGE_US     2000  // interrupter HIGH after the ramp: the coil drains the output cap Cb.
                                    // CHECK WITH A SCOPE that Cb is empty before the next pulse.

// --- Knobs and display link -------------------------------------------------
#define POT_RAW_LO      20       // ADC counts at the knob end stops - MEASURE ON YOUR BOARD (POTS line in
#define POT_RAW_HI    4075       // the serial log), see the pot box further down
#define LINK_FREQ_HZ    250000UL // I2C to the display; lower it for a long cable. The same value must be
                                 // set in cyd_ramp_view.ino - change both together.

// ============================================================================
// PIN DEFINITIONS - pin map (ESP32 DevKit V1, 30-pin)
// ============================================================================

// ┌────────────────────────────────────────────────────────────────────┐
// │  ★ PINS WERE CHOSEN BY THEIR RESET BEHAVIOUR, NOT FOR CONVENIENCE ★  │
// │  The ESP32 leaves its pins undefined for ~300 ms from reset until    │
// │  setup(). During all that time gpio_config() has NOT run yet. So     │
// │  only pins that do not output a logic 1 in that window are used for  │
// │  the power signals, and external 10 k pull-downs at the TC4420       │
// │  inputs are MANDATORY - they, not the code, keep the bridge disabled │
// │  at power-up and during a reboot.                                    │
// │  DO NOT USE: 0, 2, 5, 12, 15 (strapping), 6-11 (module flash),       │
// │  1, 3 (ROM boot log), 14 (glitch at boot).                           │
// └────────────────────────────────────────────────────────────────────┘

// --- Control buttons (all with internal pull-up, switch to GND) ---
#define PIN_BTN_FIRE 16       // GPIO16: FIRE - single pulse / arm
#define PIN_BTN_AUTO 17       // GPIO17: AUTO - auto-repeat (starts a series)
#define PIN_INTERRUPTER 4     // GPIO4:  interrupter OUTPUT (bridge enable)
#define PIN_BTN_ESTOP 23      // GPIO23: E-STOP - also the MCPWM hardware fault input

// GPIO4 was chosen for the interrupter on purpose: at reset it has an internal
// pull-DOWN, so a failure goes towards bridge disable. GPIO15 and GPIO5 have a
// pull-UP - the bridge would be enabled for all 300 ms of boot.
// GPIO23 is read with a single instruction from GPIO_IN_REG (bank 0..31), see ESTOP_DOWN().

// --- Bus to the external CYD display (hardware I2C, ESP32 = master) ---
// Both sides are 3.3 V: NO level shifter needed, 4.7 k pull-ups to 3.3 V (on
// the control board). The hardware Wire peripheral is used; the earlier AVR
// version of this board needed a bit-banged master only because the ATmega328
// TWI pins were taken by a knob.
#define PIN_LINK_SDA 21
#define PIN_LINK_SCL 22

#define PIN_PWM_RAMP 19       // GPIO19: ramp PWM OUTPUT to the buck (MCPWM0A)

// --- Shape and timing potentiometers ---
// ATTENTION: the knob rail is 3.3 V, not 5 V. ESP32 pins do not tolerate
// five volts, and the ADC reference equals the supply - 5 V would burn the
// inputs and wreck the scale.
// GPIO34/35 are input-only; a firmware bug physically cannot turn them into outputs.
//
// PW and BPS are deliberately NOT on the VP/VN pins (GPIO36/39), even though
// those are on the first ADC. The reason is practical: on the board silkscreen
// they are labelled with letters and no numbers, and during assembly two wires
// went astray - the input was left floating, and the pulse length sits on it.
// A floating input picks up noise, and the ramp length starts living a life of
// its own. GPIO25/26 are labelled with numbers and sit next to 27, which
// already carries TSCALE (printed as CURV in the pin map at start-up).
//
// The price of the move: three knobs instead of one sit on the second ADC, so
// the radio ban became even more essential. In return we got failure towards
// the safe side: on a read failure analogRead returns zero, and zero for PW and
// BPS means minimum length and minimum rate, i.e. minimum power. On VP/VN
// there was no such protection at all.
//
// GPIO25 and GPIO26 are the DAC pins. NEVER call dacWrite on them: it would
// drive a voltage straight into the potentiometer wiper.
#define PIN_POT_PW 25         // ADC2_CH8: pulse length (ramp width)
#define PIN_POT_BPS 26        // ADC2_CH9: BPS - pulse repetition rate
#define PIN_POT_AMPL 34       // ADC1_CH6: amplitude (ramp peak power)
#define PIN_POT_WICK 35       // ADC1_CH7: wick - starting step of the ramp
#define PIN_POT_AMPSIN 32     // ADC1_CH4: ramp fall
#define PIN_POT_USCALE 33     // ADC1_CH5: knee-point position in power
#define PIN_POT_TSCALE 27     // ADC2_CH7: knee-point position in time

// ┌────────────────────────────────────────────────────────────────────┐
// │  ★ WHY ONE KNOB IS ON ADC2 AND WHAT FOLLOWS FROM IT ★                │
// │  The WROOM-32 brings out SIX ADC1 channels, and there are seven      │
// │  knobs, so at least one has to go to ADC2. Here there are THREE:     │
// │  PW and BPS were added to the unavoidable TSCALE - deliberately, for │
// │  the sake of readable labelling.                                     │
// │  TSCALE was chosen because its failure is safe by construction:      │
// │  in calc_curve_deviation the value bp_y is clamped to                │
// │  [0, ramp_range], so duty stays within [wick, peak] for ANY          │
// │  tscale_x100 - a corrupted channel can change the BEND of the curve  │
// │  but can neither raise the peak nor make the ramp non-monotonic.     │
// │  THE PRICE: ADC2 belongs to the radio. WiFi and Bluetooth are        │
// │  BANNED FOREVER in this firmware - this is a condition for the knob  │
// │  to work at all, not a wish.                                         │
// └────────────────────────────────────────────────────────────────────┘

// ============================================================================
// CONSTANTS
// ============================================================================

// ╔══════════════════════════════════════════════════════════════════════╗
// ║              ★ RAMP SIGNAL FREQUENCY (PWM carrier) ★                   ║
// ║                                                                        ║
// ║  This is the MAIN frequency variable. The ramp is output as PWM on     ║
// ║  GPIO19, and its carrier frequency = 16 MHz / PWM_TOP (nominal).       ║
// ║                                                                        ║
// ║  With the 16 MHz MCPWM timer (RAMP_RES_HZ, see the MCPWM section):     ║
// ║        frequency = 16 000 000 / 640 = 25 000 Hz = 25 kHz               ║
// ║  (the exact period is PWM_TOP+1 ticks -> 24961 Hz, see PWM_FREQ_HZ)    ║
// ║                                                                        ║
// ║  TO CHANGE THE FREQUENCY - change ONLY PWM_TOP. All the other duty     ║
// ║  values below are recomputed AUTOMATICALLY (they are defined as        ║
// ║  fractions of PWM_TOP).                                                ║
// ║        PWM_TOP = 16 000 000 / desired_frequency_Hz                     ║
// ║                                                                        ║
// ║  TABLE - REFERENCE BUILD ONLY: FF200R12KE4, Rg=15Ω, 650V, L=800µH,     ║
// ║  2 W gate DC-DC (+15/-9 V), Qg≈1.8µC. Recompute EVERY column for your  ║
// ║  own brick, gate supply and choke before trusting the verdicts.        ║
// ║  ┌────────┬─────────┬────────────┬──────────┬────────┬──────────────┐  ║
// ║  │  f     │ PWM_TOP │ sw. peak   │ avg@10BPS│ DC-DC  │  verdict     │  ║
// ║  ├────────┼─────────┼────────────┼──────────┼────────┼──────────────┤  ║
// ║  │ 20 kHz │   800   │   650 W    │   73 W   │  43 %  │ audible whine│  ║
// ║  │ 25 kHz │   640   │   813 W    │   89 W   │  54 %  │ ★ OPTIMUM ★  │  ║
// ║  │ 30 kHz │   533   │   976 W    │  106 W   │  65 %  │ ok           │  ║
// ║  │ 40 kHz │   400   │  1301 W    │  138 W   │  86 %  │ hot + tight  │  ║
// ║  │ 50 kHz │   320   │  1626 W    │  171 W   │ 108 %  │ DC-DC can't  │  ║
// ║  └────────┴─────────┴────────────┴──────────┴────────┴──────────────┘  ║
// ║                                                                        ║
// ║  Window limits:                                                        ║
// ║   ABOVE - the gate DC-DC module (2 W): P = Qg × ΔVge × f.              ║
// ║           With Qg≈1.8 µC and ΔVge≈24 V (reference build) the ceiling   ║
// ║           is ~39 kHz with margin - recompute for your driver supply.   ║
// ║   ABOVE - buck heat: IGBT switching losses grow linearly with f.       ║
// ║   BELOW - audibility: below 20 kHz the choke sings (magnetostriction;  ║
// ║           heard on the reference build's 800 µH choke).                ║
// ║   BELOW - choke ripple (the reference 800 µH covers even 15 kHz).      ║
// ║                                                                        ║
// ║  RAISING THE FREQUENCY (smaller PWM_TOP) makes the ramp SMOOTHER: less ║
// ║  ripple through the choke and a finer time grid of the envelope. The   ║
// ║  price is paid by the whole switching path: the gate driver (more      ║
// ║  average gate current), the gate DC-DC (P = Qg x dVge x f) and the     ║
// ║  IGBT brick (switching losses grow linearly with f) all run hotter.    ║
// ║  Stay inside the table window above.                                   ║
// ║                                                                        ║
// ║  BONUS of a low frequency: PWM_TOP = number of ramp steps.             ║
// ║  25 kHz gives 640 steps versus 400 at 40 kHz - a smoother ramp.        ║
// ╚══════════════════════════════════════════════════════════════════════╝
// PWM_TOP and ALPHA_X1000_DEFAULT are set in BUILD CONFIGURATION at the top of the file.

// ┌────────────────────────────────────────────────────────────────────┐
// │  ★ RAMP CEILING - 100 % DUTY ★                                      │
// │  MAX_DUTY = PWM_TOP means the ramp reaches FULL duty: the buck's    │
// │  high-side switch is on continuously, buck output = bus voltage.    │
// │  This way the bank's energy is used to the very end.                │
// │                                                                     │
// │  SAFE ONLY because the high-side driver is powered from an          │
// │  ISOLATED DC-DC (R12P21509D), NOT from a bootstrap. With a          │
// │  bootstrap, 100 % duty would kill the high-side rail. If you move   │
// │  to a bootstrap - lower MAX_DUTY again (about 60 % of PWM_TOP,      │
// │  e.g. 380 for PWM_TOP 640) so the bootstrap cap gets recharged.     │
// └────────────────────────────────────────────────────────────────────┘
// MAX_DUTY is set in BUILD CONFIGURATION at the top of the file.
#define MIN_DUTY 0         // min. duty = bridge off

// Upper limit of the amplitude knob (AMPL). Chosen so that, WITH the alpha
// compensation, the ramp peak lands exactly at MAX_DUTY instead of being clipped:
//    peak_total = peak × (1 + alpha) = MAX_DUTY
//    => AMPL_POT_MAX = PWM_TOP / (1 + alpha)
#define AMPL_POT_MAX  ((int32_t)PWM_TOP * 1000L / (1000L + ALPHA_X1000_DEFAULT))

// --- Pulse length (ramp width): PULSE_WIDTH_MIN/MAX_MS - see BUILD CONFIGURATION ---
// ┌────────────────────────────────────────────────────────────────────┐
// │  ★ PULSE REPETITION FREQUENCY (BPS = bursts per second) ★           │
// │  How many ramp pulses per second. Set by the BPS knob.              │
// │  Values are stored ×10 (to get a 0.1 step): 50 = 5.0 BPS.           │
// └────────────────────────────────────────────────────────────────────┘
// BPS_MIN_X10 / BPS_MAX_X10 - see BUILD CONFIGURATION at the top of the file.

// --- Ramp fall (how fast the power drops at the end of the pulse) ---
// FALL knob at ZERO = abrupt cut-off at the peak. That is the "sawtooth", classic QCW.
#define FALL_MIN_X100 0          // 0 = abrupt cut-off at the end
#define FALL_MAX_X100 50         // 50 = gentle fall (fraction of the ramp length, ×100)

// --- Wick: the starting step at the beginning of the ramp ---
// ┌────────────────────────────────────────────────────────────────────┐
// │  ★ WICK KNOB CEILING - HARD LIMIT ★                                 │
// │  Wick is the pilot step, not the start of the ramp. The working     │
// │  value on the reference build is ~40 V (WICK_VOLTAGE below).        │
// │  WHY A LIMIT: an excessive wick SPLITS THE ARC at the breakout      │
// │  point (observed on the reference coil). And at ~75 % of the bus    │
// │  the ramp disappears altogether - that is not QCW.                  │
// └────────────────────────────────────────────────────────────────────┘
// WICK_MAX_PCT_OF_BUS - see BUILD CONFIGURATION at the top of the file.
#define WICK_MAX_DUTY  ((int32_t)PWM_TOP * (int32_t)WICK_MAX_PCT_OF_BUS / 100L)

// ╔══════════════════════════════════════════════════════════════════════╗
// ║        ★ WICK IN VOLTS - automatic calculation for any bus ★           ║
// ║  Wick is physically set as a DUTY, but it is easier to think in volts. ║
// ║  V_out = V_bus × duty / PWM_TOP  =>  duty = V × PWM_TOP / V_bus.       ║
// ║  SETUP: enter the three numbers below for your installation.           ║
// ║  CHECK WITH A SCOPE at the buck output (after the choke): if the step  ║
// ║  is below target - raise WICK_LOSS_COMP_PCT.                           ║
// ╚══════════════════════════════════════════════════════════════════════╝
// BUS_VOLTAGE, WICK_VOLTAGE, WICK_LOSS_COMP_PCT - see BUILD CONFIGURATION at the top.
#define WICK_MIN_DUTY  ( ( (long)WICK_VOLTAGE * (100 + WICK_LOSS_COMP_PCT) * PWM_TOP \
                           + (100L * BUS_VOLTAGE) / 2 ) \
                         / (100L * BUS_VOLTAGE) )

// ── DERIVED QUANTITIES (computed from the macros, printed at start-up) ──
// Do not duplicate these numbers in comments - they will go stale. See the CONFIG banner.
#define PWM_FREQ_HZ     (16000000L / ((long)PWM_TOP + 1))   // timer period = PWM_TOP+1 ticks (RAMP_PERIOD_TICKS)
#define WICK_MIN_VOLTS  ((long)WICK_MIN_DUTY * (long)BUS_VOLTAGE / (long)PWM_TOP)
#define WICK_MAX_VOLTS  ((long)WICK_MAX_DUTY * (long)BUS_VOLTAGE / (long)PWM_TOP)

// FOOL-PROOFING: the floor must not exceed the ceiling. Caught at COMPILE time.
static_assert(WICK_MIN_DUTY < WICK_MAX_DUTY,
  "WICK_VOLTAGE is too high: the wick floor exceeds the WICK_MAX_PCT_OF_BUS ceiling. "
  "Lower WICK_VOLTAGE or raise WICK_MAX_PCT_OF_BUS.");
static_assert(WICK_MAX_DUTY <= PWM_TOP,
  "WICK_MAX_PCT_OF_BUS > 100 - the wick ceiling is above full duty.");

// --- Ramp curve shape (controlled by the T and U knobs) ---
// tscale (T knob) - WHERE IN TIME the knee point of the curve is.
// uscale (U knob) - at what HEIGHT (in power) that knee point is.
#define TSCALE_MIN_X100 0        // T: knee at the very beginning
#define TSCALE_MAX_X100 200      // T: knee closer to the end
#define USCALE_MIN_X100 10       // U: low knee point
#define USCALE_MAX_X100 90       // U: high knee point
#define USCALE_LOW_THRESH_X100 40

#define DEBOUNCE_MS 50

// ┌────────────────────────────────────────────────────────────────────┐
// │  ★ ADC SCALE LIMITS ★                                               │
// │  On the AVR it was map(raw, 5, 1018, ...) with 10 bits. Here it is  │
// │  12 bits, the same fractions of travel: 5×4095/1023 = 20,           │
// │  1018×4095/1023 = 4075.                                             │
// │  map stays the SAME linear function of the rotation fraction, and   │
// │  all knob output ranges are preserved to within one rounding step.  │
// │                                                                     │
// │  THESE TWO NUMBERS MUST BE MEASURED ON THE ACTUAL BOARD.            │
// │  Without eFuse calibration the ADC gain spread reaches ±3 %, that   │
// │  is ±122 counts, i.e. ±3 % of knob travel. Procedure: print the raw │
// │  counts, turn every knob to both end stops, take                    │
// │  LO = max(of all minima) + 8, HI = min(of all maxima) - 8.          │
// │                                                                     │
// │  WHAT NOT TO DO:                                                    │
// │   * raw >>= 2 while keeping 5/1018 - throws away the only gain      │
// │     of the port, the resolution;                                    │
// │   * convert to millivolts and divide by 3300 - with 11 dB           │
// │     attenuation full scale is about 3100 mV, the top of every knob  │
// │     becomes unreachable (amplitude stalls at 498 instead of 528).   │
// └────────────────────────────────────────────────────────────────────┘
// Maximum amplitude rise per one poll pass - one tenth of full scale. A single
// spike raises the power by no more than this step and is removed by the next
// pass, because there is no limit in the downward direction.
//
// ABOUT THE PERIOD, precisely: in manual mode a pass runs every 50 ms, but in
// AUTO above roughly ten BPS (with a 25 ms ramp) the quiet-zone gate reduces
// it to one pass per firing cycle - 66...99 ms. So a full knob sweep from bottom to top takes
// 0.55 s in manual and up to 1.1 s in AUTO. The same delay applies to a
// DECREASE: a knob with which the operator cuts the power without touching
// E-STOP responds with a lag of up to 99 ms plus one already started shot.
#define PEAK_RISE_MAX_PER_PASS ((uint16_t)(AMPL_POT_MAX / 10))
// POT_RAW_LO / POT_RAW_HI - see BUILD CONFIGURATION at the top of the file.

#define WATCHDOG_TIMEOUT_MS 2000

// ============================================================================
// RAMP SHIFT / BUCK STAGE TIMING
// ============================================================================
// = 1 : ramp shift ACTIVE.
//   The ramp (GPIO19) starts on wick BUCK_WARMUP_US BEFORE the interrupter
//   (GPIO4) goes HIGH (LC filter settles), and the interrupter is held HIGH
//   for another BUCK_DISCHARGE_US AFTER the ramp end.
// = 0 : shift off (Phase 1 without buck).
// ENABLE_BUCK_TIMING, BUCK_WARMUP_US, BUCK_ENABLE_DELAY_US - see BUILD CONFIGURATION.
// ┌────────────────────────────────────────────────────────────────────┐
// │  ★ Cb DISCHARGE WINDOW - interrupter held HIGH after the ramp end ★ │
// │  While the bridge is on, the coil ITSELF drains the output          │
// │  capacitor. If Cb stays charged, the next pulse switches the bridge │
// │  onto a charged bus - certain death for the IGBTs.                  │
// │  CHECK WITH A SCOPE the voltage on Cb before the next pulse!        │
// └────────────────────────────────────────────────────────────────────┘
// BUCK_DISCHARGE_US - see BUILD CONFIGURATION at the top of the file.

// ============================================================================
// OPERATION MODES
// ============================================================================
// The enum is kept IN FULL on purpose: calculate_duty has a case for all three
// values, and removing the constants would force editing the shape function.
// Only MODE_QCW is reachable; the other branches are unreachable.

enum OperationMode {
 MODE_QCW = 0,
 MODE_INTERRUPTED = 1,
 MODE_CW_QCW = 2,
 MODE_COUNT = 3
};

// ============================================================================
// STATE
// ============================================================================

volatile uint16_t pulse_width_ms = 10;
volatile uint16_t bps_x10 = 20;
volatile uint16_t peak_duty = MAX_DUTY / 2;
// Unfiltered amplitude candidate - only for checks and diagnostics.
// It does NOT go into the ramp path: there peak_duty is used, rate-limited
// on the rise (see update_pots). Needed because the plausibility gate must
// look at the RAW reading: by construction the filter does not let the first
// sample after power-up reach the end stop, and a gate on peak_duty was dead.
volatile uint16_t peak_duty_raw = MAX_DUTY / 2;
volatile uint16_t wick_offset = 0;
volatile uint16_t fall_x100 = 12;
volatile uint16_t tscale_x100 = 100;
volatile uint16_t uscale_x100 = 100;
volatile uint16_t alpha_x1000 = ALPHA_X1000_DEFAULT;

// ============================================================================
// PRECOMPUTED RECIPROCALS (optimisation of the hot ramp loop)
// ============================================================================
// A 32-bit division by a VARIABLE divisor is expensive (~200 cycles on the AVR
// this was ported from; still a hardware divide of a few dozen cycles on the
// LX6, inside an interrupt at 25 kHz). The divisor only changes when the knobs
// are turned - so the reciprocal is computed ONCE in update_pots, and the hot
// path multiplies and shifts:
//      was:  y = (x * 10000) / D
//      now:  y = (x * inv_D) >> 16,   where inv_D = (10000 << 16) / D
// (On the AVR build this took the ramp loop from ~100 to ~40 us.) Overflow is
// impossible: the maximum product = 655 360 000 < 2^32.
// NOT volatile on purpose. They ARE read from the carrier interrupt (on_tez ->
// calculate_duty), but only while a pulse runs, and update_pots - the only
// writer - never runs during a pulse (fire_pulse is blocking, see the invariant
// note there). volatile would only eat the gain.
uint32_t inv_pulse_q16 = 1;   // (10000<<16) / pulse_us
uint32_t inv_rise_q16  = 1;   // (10000<<16) / RISE_END_X10000
uint32_t inv_fall_q16  = 1;   // (10000<<16) / fall_total
int32_t  bp_x_cached   = 5000;
uint32_t inv_bpx_q30   = 1;   // (1<<30) / bp_x
uint32_t inv_seg2_q30  = 1;   // (1<<30) / (10000 - bp_x)

volatile bool fire_armed = false;
volatile bool auto_mode = false;
volatile bool manual_pulse_pending = false;
volatile bool estop_triggered = false;
// The E-STOP button was released after it tripped. While false, pressing it
// again will not clear the fault - otherwise one long press would set it and
// immediately clear it.
volatile bool estop_released  = false;
// The button is ready to set a NEW fault. Becomes true only on release.
// Without this, the press that cleared the fault would set it again on the
// very next pass: the set path reads the raw level, and the button is still
// pressed at that moment.
volatile bool estop_ready     = true;
// Since when the button has been continuously released. Needed so that
// re-arming happens on a STABLE high level: after a press the mechanical
// contact bounces for several milliseconds, and without this the very first
// bounce up would arm the button and the next bounce down would latch the
// fault again immediately.
uint32_t estop_high_since = 0;
// Mirror accumulator: since when the button has been continuously PRESSED.
// Clearing the fault requires a stable press, otherwise a single glitch on the
// E-STOP input would clear the fault by itself.
uint32_t estop_low_since  = 0;
// A blocked reset is reported once, not on every loop pass.
bool     estop_blocked_reported = false;

struct ButtonState {
 bool last_state;
 bool stable_state;
 uint32_t last_change_ms;
};

ButtonState btn_fire = {HIGH, HIGH, 0};
ButtonState btn_auto = {HIGH, HIGH, 0};
ButtonState btn_estop = {HIGH, HIGH, 0};

uint32_t last_pulse_ms = 0;
// A separate timestamp of the pulse START, only for choosing the knob-poll
// moment. last_pulse_ms must not be touched: the repetition rate hangs on it,
// and it is deliberately taken in the same place as on the AVR - at the start
// of the loop pass. Because of that it lags the real edge by the length of a
// pass (up to ~12 ms: display tick plus knob poll), and the quiet zone computed
// from it would be off by exactly that amount.
volatile uint32_t last_pulse_start_ms = 0;
uint32_t last_pot_update_ms = 0;
uint32_t armed_since_ms = 0;
uint32_t pulse_counter = 0;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void hw_init_mcpwm();
bool mcpwm_index_selftest();
void estop_sync_from_isr();
void estop_recover_hw();
void print_config_banner();
void outputs_off();
void emergency_stop();
void update_pots();
void handle_buttons();
bool check_button_press(uint8_t pin, ButtonState& state);
int32_t calculate_duty(uint32_t t_elapsed_us, uint32_t pulse_us, uint8_t mode);
void fire_pulse();
uint16_t read_pot_oversampled(uint8_t pin);
int32_t calc_curve_deviation(int32_t rise_t, int32_t ramp_range);
void print_mode_name();
inline bool busy_wait_us(uint32_t us);

// ============================================================================
// MCPWM: RAMP (GPIO19) AND INTERRUPTER (GPIO4)
// ============================================================================
//
// On the AVR the ramp ran on Timer1 in Fast PWM mode 14, TOP = ICR1 = PWM_TOP,
// prescaler 1. Carrier = 16 MHz / (PWM_TOP + 1) = 24961 Hz. Here this is
// REPRODUCED BIT FOR BIT: timer resolution 16 MHz, period PWM_TOP + 1 ticks.
// The same number of hertz, the same duty scale 0..640, not a single shape
// constant recomputed. The caption on the display (it divides 16 000 000 by
// pwm_top+1, see cyd_ramp_view.ino) remains correct.
//
// WHY MCPWM AND NOT LEDC - three reasons, each decisive:
//  1) the LEDC period is always a power of two, 641 ticks cannot be expressed.
//     The duty would have to be scaled - i.e. the envelope shape would be
//     SILENTLY changed;
//  2) LEDC has no hardware fault input at all. MCPWM has one: E-STOP drops the
//     output without a single CPU instruction;
//  3) MCPWM is clocked from PLL160M and does not depend on the APB bus frequency.
//
// WHAT CHANGED IN SUBSTANCE: the duty is no longer recomputed in a blocking
// loop, but updated from a callback on EVERY carrier period. That is ~625
// updates per 25 ms pulse (624 at the exact 24 961 Hz), and it is the ceiling:
// the comparator shadow register latches once per period, writing more often
// is pointless.
//
// The E-STOP poll and the reverse watchdog hang on the callback as well: they
// run on every carrier period, not once per loop iteration of unknown length.

#define RAMP_RES_HZ    16000000UL          // equal to the F_CPU of the earlier AVR build - the duty scale did not shift
#define RAMP_PERIOD_TICKS  ((uint32_t)PWM_TOP + 1)

// The core frequency is a CONSTANT, not a variable. The LX6 has hardware
// division (quos/quou), but it costs about thirty cycles, while division by a
// constant is unrolled by the compiler into a multiply and shift - a few
// cycles. In an interrupt at 25 kHz that is a difference worth taking for free.
// The firmware CHECKS the real frequency at start-up and refuses to run on a
// mismatch: otherwise all three buck phases and the whole ramp length would
// shift by a factor of 1.5, because busy_wait_us counts in cycles.
#define CPU_MHZ 240

// The pulse watchdog is built on pdMS_TO_TICKS with a 3 ms margin. With a
// 100 Hz tick (the plain ESP-IDF default; the ESP32 Arduino core 3.3.x builds
// with 1000 Hz) that would give 2 ticks = 20 ms for a 25 ms setting - a false
// fault on EVERY shot, and 0 ticks for a short pulse.
// Caught at compile time, not on the coil.
static_assert(configTICK_RATE_HZ >= 1000,
  "The pulse watchdog requires a scheduler tick of at least 1 kHz (CONFIG_FREERTOS_HZ).");

// Safe comparator value for the ramp switched off. ONE, not zero: with zero
// the match lands on the same cycle as the counter reset, and the driver does
// not promise how the conflict is resolved. One gives a 62.5 ns pulse, which
// does not pass to the output anyway while the level force is on.
#define CMP_SAFE 1

static mcpwm_timer_handle_t h_tmr  = NULL;
static mcpwm_oper_handle_t  h_oper = NULL;
static mcpwm_cmpr_handle_t  h_cmp  = NULL;
static mcpwm_gen_handle_t   h_genA = NULL;   // ramp
static mcpwm_gen_handle_t   h_genB = NULL;   // interrupter
static mcpwm_fault_handle_t h_flt  = NULL;

#define MCPWM_HW  MCPWM_LL_GET_HW(0)
#define OPER_ID   0
#define GEN_RAMP  0
#define GEN_BRDG  1
#define CMP_ID    0

// Direct register access instead of driver calls - out of necessity.
// In the prebuilt ESP32 Arduino core (observed with 3.3.x)
// CONFIG_MCPWM_CTRL_FUNC_IN_IRAM is off, i.e.
// mcpwm_comparator_set_compare_value lives in FLASH and takes a spinlock.
// That cannot be called from an interrupt at 25 kHz. The functions below are
// static inline from hal/mcpwm_ll.h; they inline straight into our IRAM code.
// Force level: -1 release, 0 - hold LOW, 1 - hold HIGH.
static inline void IRAM_ATTR cmp_write_ll(uint32_t v) {
 mcpwm_ll_operator_set_compare_value(MCPWM_HW, OPER_ID, CMP_ID, v);
}

// ★ LOCK ON THE FORCE REGISTER ★
// All four helpers below edit bit fields of ONE 32-bit register gen_force, and
// each does a read-modify-write. Both the task and the interrupt write to it.
// There is exactly one dangerous interleaving and it is fatal: the task runs
// bridge_force_ll(1) - has read the word, has not yet written it - and between
// the read and the write an interrupt with ramp_kill_ll on a fault slips in.
// The task's write brings back the OLD contents of both fields: the E-STOP
// shutdown is undone, the ramp is released, the bridge is enabled. The window
// is narrow (a dozen instructions), but it occurs once per pulse, and the
// price is an enabled bridge with E-STOP pressed.
// The SAFE versions work both in the task and in the interrupt. The lock is
// recursive PER CORE (the spinlock tracks owner and depth), so nested takes
// are legal.
// About the frequency, honestly: there are several dangerous windows per
// pulse, each a dozen instructions long, and they are dangerous ONLY when they
// coincide with a fault event. So it is not "once per pulse" but "once per
// fault that arrives within a fraction of a percent of the pulse length". The
// fix is needed not because of the frequency but because of the price of the
// coincidence.
static portMUX_TYPE g_force_mux = portMUX_INITIALIZER_UNLOCKED;
#define FORCE_LOCK()   portENTER_CRITICAL_SAFE(&g_force_mux)
#define FORCE_UNLOCK() portEXIT_CRITICAL_SAFE(&g_force_mux)
// Ramp shutdown WITH THE COMPARATOR ZEROED. Use THIS, not the bare force.
//
// WHY: the comparator is shadowed, the written value latches only on the
// counter reset. If the value from the peak of the previous pulse is left in
// it (and with the fall knob at the lower stop - the "classic sawtooth" -
// almost full duty stays there), then at the start of the next pulse there is
// a window: the new wick value has not latched yet, but the force is already
// released. For up to 40 us (one carrier period) the buck outputs the full bus
// instead of the wick step - with the defaults in this file (325 V bus, 40 V
// wick, WICK_MIN_DUTY 88) that step is 5.5 us wide, i.e. seven times the
// volt-seconds. The current goes into the choke
// and recharges the storage bank ABOVE wick while the bridge is still
// disabled, 2 ms before it is enabled. A charged bank at the moment the
// bridge switches on is death for the IGBTs, as written in this same file at
// BUCK_DISCHARGE_US.
//
// On the AVR the state is unreachable: there OCR1A is zero between pulses.
static inline void IRAM_ATTR ramp_off_ll(void) {
 FORCE_LOCK();
 mcpwm_ll_gen_set_continue_force_level(MCPWM_HW, OPER_ID, GEN_RAMP, 0);
 cmp_write_ll(CMP_SAFE);          // under the same lock: the pair is indivisible
 FORCE_UNLOCK();
}
static inline void IRAM_ATTR ramp_release_ll(void) {
 FORCE_LOCK();
 mcpwm_ll_gen_set_continue_force_level(MCPWM_HW, OPER_ID, GEN_RAMP, -1);
 FORCE_UNLOCK();
}
static inline void IRAM_ATTR bridge_force_ll(int on) {
 FORCE_LOCK();
 mcpwm_ll_gen_set_continue_force_level(MCPWM_HW, OPER_ID, GEN_BRDG, on ? 1 : 0);
 FORCE_UNLOCK();
}
// The pair {ramp, bridge} is shut down in ONE critical section, not two in a
// row. The ESP-IDF lock is recursive per core, so the nested takes inside are
// legal. Without the outer lock there was a gap between the ramp shutdown and
// the bridge shutdown with interrupts enabled - harmless today, but the next
// editor would assume this pair is indivisible, and it would fall apart silently.
static inline void IRAM_ATTR ramp_kill_ll(void) {
 FORCE_LOCK();
 ramp_off_ll();
 bridge_force_ll(0);
 FORCE_UNLOCK();
}

// Raw fault read - one instruction, no Arduino wrapper. GPIO23 is in bank
// 0..31, so GPIO_IN_REG is enough.
#define ESTOP_DOWN() (((REG_READ(GPIO_IN_REG) >> PIN_BTN_ESTOP) & 1UL) == 0UL)
#define ESTOP_UP()   (((REG_READ(GPIO_IN_REG) >> PIN_BTN_ESTOP) & 1UL) != 0UL)

// --- ramp state, shared between the task and the interrupt ---
static volatile uint32_t rs_tick          = 0;      // period counter, diagnostics only
static volatile uint32_t g_pulse_us       = 0;
static volatile uint32_t g_ramp_t0_cyc    = 0;      // core cycle count at ramp start
static volatile bool     ramp_active      = false;
static volatile bool     g_estop_req      = false;  // the interrupt asks to latch the fault
static volatile bool     g_brake_ost      = false;  // the hardware brake has tripped
// Whether the button level was low AT THE MOMENT OF THE BRAKE. A glitch cannot
// be told from a finger by reading the level LATER, at synchronisation: up to
// a whole loop pass elapses between the event and its handling, and a short
// deliberate press has time to be released - the firmware would declare it a
// glitch.
static volatile bool     g_brake_btn      = false;
static volatile bool     g_stall_kill     = false;  // the task stopped responding
static bool              g_brake_ok       = false;  // the hardware brake was really created
// MCPWM registers may only be touched after the driver has enabled the group
// clock. hw_init_mcpwm may exit at its very first step, and then an access to
// MCPWM_HW from outputs_off or from the safety net in loop() would go to an
// unpowered peripheral - a failure in the shutdown chain itself.
static volatile bool     g_mc_regs_ok     = false;
// Flag that the carrier callback has run AT LEAST ONCE, plus the core number
// for diagnostics. The value -1 means "the interrupt has never run" - and in
// that state firing is forbidden: otherwise the bridge would be enabled, the
// buck would run at a constant wick duty, and only the watchdog timeout would
// catch it, after everything had already switched on.
// The question of CCOUNT being out of sync between cores NO LONGER ARISES here:
// the ramp reference point is taken in the interrupt itself (see on_tez), so
// the cycle counter is always read by the same core.
static volatile int8_t   g_isr_core       = -1;
static volatile uint32_t g_isr_since_task = 0;      // reverse watchdog, see below
static volatile uint16_t g_cmp_min = 0xFFFF, g_cmp_max = 0;
static TaskHandle_t      h_qcw     = NULL;
static uint32_t          g_cpu_mhz = 240;
static bool              g_need_recover = false;
static bool              g_mcpwm_ok = false;

// How many carrier periods the task is allowed to stay silent. Legitimate
// idle time (measured on the reference build, pots=/prev= in the POTS line):
// ramp up to 25 ms + knob poll ~4 ms + frame set ~6 ms + diagnostics to the
// port ~6 ms ≈ 50 ms. A threshold of 160 ms (4000 periods of 40 us) gives a
// threefold margin.
// WHY THIS AT ALL: on the AVR the ramp lived INSIDE the loop - loop stalls,
// ramp stalls. Here the ramp is driven by an interrupt, and it will keep
// happily producing pulses over a dead task. This is a new failure class that
// did not exist on the AVR.
#define ISR_TASK_STALL_TICKS 4000

// Time inside the ramp is ABSOLUTE, from the core cycle counter.
//
// Counting interrupt calls would be cheaper, but it changes the failure
// property. On the AVR the length was measured by an absolute clock, so any
// hiccup made the system jump forward and the pulse still ended on time - the
// error was SELF-CORRECTING. A call counter behaves the opposite way: losing
// every second call gives exactly a twice-as-long pulse, twice the energy, and
// NOT ONE diagnostic message - the pulse counter grows as for a normal shot.
//
// RSR CCOUNT is one instruction and takes no spinlock (unlike micros() and
// esp_timer_get_time()). Division by a CONSTANT is unrolled by the compiler
// into a multiply and shift.
static inline uint32_t IRAM_ATTR ramp_elapsed_us(void) {
 return (esp_cpu_get_cycle_count() - g_ramp_t0_cyc) / CPU_MHZ;
}

int32_t calculate_duty(uint32_t t_elapsed_us, uint32_t pulse_us, uint8_t mode);

// Callback on counter reset (TEZ) - one call per carrier period.
// EVERYTHING it calls must be in IRAM and free of float: FPU registers are not
// saved in an interrupt, that is silent data corruption.
static bool IRAM_ATTR on_tez(mcpwm_timer_handle_t timer,
                             const mcpwm_timer_event_data_t *ed, void *ctx) {
 (void)timer; (void)ed; (void)ctx;
 BaseType_t hp = pdFALSE;

 if (g_isr_core < 0) g_isr_core = (int8_t)xPortGetCoreID();

 const uint32_t alive = g_isr_since_task + 1;
 g_isr_since_task = alive;
 if (alive > ISR_TASK_STALL_TICKS) {                // the task is not responding
 ramp_kill_ll();
 ramp_active  = false;
 g_stall_kill = true;                              // NOT gated: nothing to do with the button
 return false;
 }

 if (ESTOP_DOWN()) {
 ramp_kill_ll();                                   // the outputs are ALWAYS dropped
 ramp_active = false;
 // But the REQUEST to latch the fault is gated by estop_ready - exactly as
 // POTS_ESTOP_GUARD does. Without the gate the fault could NEVER be cleared:
 // clearing requires HOLDING the button for 50 ms, and all that time the
 // callback sees a low level and asks for the latch again - about 1250 times
 // per gesture. The next loop pass would latch the fault back, deterministically.
 // This is exactly the bug that on the AVR sounded like "sometimes you have
 // to press it 10 times".
 if (estop_ready) {
 g_estop_req = true;
 if (h_qcw) vTaskNotifyGiveFromISR(h_qcw, &hp);
 }
 return hp == pdTRUE;
 }

 if (!ramp_active) return false;

 // Safety net inside the interrupt itself: during the ramp the
 // bridge generator MUST be in force "high" (cntuforce_mode == 2); fire_pulse
 // sets it before arming. If it is not, the bridge was shut down by an
 // interrupt (estop_isr/on_ost) in a window the task did not see. The ramp
 // must not run with the bridge off: the buck bank would charge with no load,
 // and the next pulse would switch the bridge onto a charged bus. Shut down
 // and ask for the latch.
 if (MCPWM_HW->operators[OPER_ID].gen_force.gen_b_cntuforce_mode != 2) {
 ramp_kill_ll();
 ramp_active = false;
 if (estop_ready) {
 g_estop_req = true;
 if (h_qcw) vTaskNotifyGiveFromISR(h_qcw, &hp);
 }
 return hp == pdTRUE;
 }

 // The reference point is taken RIGHT HERE, in the interrupt, on the first
 // period after arming. Each core has its own CCOUNT, not synchronised with
 // the other; if the task took the reference and the interrupt computed the
 // difference, correctness would rest on their happening to be on the same
 // core - a coincidence fixed nowhere. With CCOUNT always read by the same
 // core, cross-core skew does not exist as a failure class.
 // The price: the ramp start is aligned to a carrier period boundary, i.e. a
 // delay of up to 40 us. It lengthens the pause before the rise, and a longer
 // pause is safe here: the buck LC filter settles further with the bridge
 // already enabled.
 if (rs_tick == 0) g_ramp_t0_cyc = esp_cpu_get_cycle_count();

 uint32_t el = ramp_elapsed_us();
 if (el >= g_pulse_us) {                            // the ramp has ended
 ramp_off_ll();                                    // by level + comparator to safe
 ramp_active = false;
 if (h_qcw) vTaskNotifyGiveFromISR(h_qcw, &hp);
 return hp == pdTRUE;
 }

 int32_t d = calculate_duty(el, g_pulse_us, MODE_QCW);
 if (d < 0) d = 0;
 if (d > MAX_DUTY) d = MAX_DUTY;

 // RE-CHECK THE ARMED STATE BEFORE WRITING. While the point was being computed,
 // another handler (button or brake) may have latched the fault - it drops the
 // outputs and clears ramp_active. Without this line we would write the point
 // and release the force, i.e. UNDO the shutdown: the buck would charge the
 // bank with the bridge disabled.
 if (!ramp_active) return false;

 if (d == 0) {
 ramp_off_ll();
 } else {
 cmp_write_ll((uint32_t)d);
 ramp_release_ll();
 }
 if ((uint16_t)d < g_cmp_min) g_cmp_min = (uint16_t)d;
 if ((uint16_t)d > g_cmp_max) g_cmp_max = (uint16_t)d;
 rs_tick = rs_tick + 1;
 return false;
}

// The hardware brake has latched. Mirroring it into software is MANDATORY: a
// nanosecond-long glitch drops the outputs in hardware, and the firmware would
// not know - the loop would keep "firing", the pulse counter would grow, a
// green STATUS would go to the display, yet the outputs are dead. The operator
// would turn the amplitude up, and the first pulse after the latch is cleared
// would go out at the increased power.
static bool IRAM_ATTR on_ost(mcpwm_oper_handle_t oper,
                             const mcpwm_brake_event_data_t *ed, void *ctx) {
 (void)oper; (void)ed; (void)ctx;
 // Drop the outputs OURSELVES rather than rely on the silicon having done it.
 // The brake wiring is assembled from five calls, and the generator actions
 // are two of them; on their partial failure the latch in hardware exists but
 // the shutdown does not. This was the only interrupt handler that did not
 // touch the outputs, i.e. all the safety rested on the setup succeeding.
 ramp_kill_ll();
 g_brake_ost = true;
 g_brake_btn = ESTOP_DOWN();          // sample the level HERE, not later
 ramp_active = false;
 // Waking the task is MANDATORY. A glitch on the button cable gives a
 // nanosecond dip - this chip has no hardware filter on the fault input at
 // all - the brake latches, and the edge interrupt reads the level already
 // recovered and stays silent. Without the wake-up the task would sleep out
 // the full timeout and print "ramp ISR did not respond", i.e. a FALSE
 // diagnosis: the interrupt was working fine, and the real cause - EMI on the
 // cable - was lost.
 BaseType_t hp = pdFALSE;
 if (estop_ready) g_estop_req = true;
 if (h_qcw) vTaskNotifyGiveFromISR(h_qcw, &hp);
 return hp == pdTRUE;
}

// A dedicated interrupt on the button. Needed NOT for speed - by this time the
// hardware brake has already dropped the outputs - but to remove the force from
// the interrupter: force and brake are different mechanisms, and the priority
// between them is not promised by the silicon.
static void IRAM_ATTR estop_isr(void) {
 if (ESTOP_DOWN()) {
 ramp_kill_ll();                       // the outputs are ALWAYS dropped, unconditionally
 ramp_active = false;
 if (estop_ready) {                    // the latch request - see the gate in on_tez
 g_estop_req = true;
 BaseType_t hp = pdFALSE;
 if (h_qcw) vTaskNotifyGiveFromISR(h_qcw, &hp);
 if (hp == pdTRUE) portYIELD_FROM_ISR();
 }
 }
}

static inline void wdt_kick(void) {
 esp_task_wdt_reset();
 rtc_wdt_feed();
}

// Every initialisation step is checked and NAMES ITSELF on failure, so that
// the banner line "FATAL: operation forbidden - reason above" always has a
// reason printed above it - a silent early exit would leave nothing there.
#define MC_STEP(call, what) do { \
 esp_err_t e_ = (call); \
 if (e_ != ESP_OK) { Serial.print(F("FATAL MCPWM: ")); Serial.print(F(what)); \
 Serial.print(F(" -> ")); Serial.println(esp_err_to_name(e_)); return; } \
} while (0)

void hw_init_mcpwm() {
 mcpwm_timer_config_t tc = {};
 tc.group_id      = 0;
 tc.clk_src       = MCPWM_TIMER_CLK_SRC_PLL160M;
 tc.resolution_hz = RAMP_RES_HZ;
 tc.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
 tc.period_ticks  = RAMP_PERIOD_TICKS;
 tc.intr_priority = 0;
 MC_STEP(mcpwm_new_timer(&tc, &h_tmr), "timer creation");

 mcpwm_operator_config_t oc = {};
 oc.group_id = 0;
 MC_STEP(mcpwm_new_operator(&oc, &h_oper), "operator creation");
 MC_STEP(mcpwm_operator_connect_timer(h_oper, h_tmr), "connecting operator to timer");

 mcpwm_comparator_config_t cc = {};
 cc.flags.update_cmp_on_tez = true;     // shadow register: no glitches
 MC_STEP(mcpwm_new_comparator(h_oper, &cc, &h_cmp), "comparator creation");
 MC_STEP(mcpwm_comparator_set_compare_value(h_cmp, CMP_SAFE), "comparator initialisation");

 mcpwm_generator_config_t gA = {}; gA.gen_gpio_num = PIN_PWM_RAMP;
 MC_STEP(mcpwm_new_generator(h_oper, &gA, &h_genA), "ramp generator");
 mcpwm_generator_config_t gB = {}; gB.gen_gpio_num = PIN_INTERRUPTER;
 MC_STEP(mcpwm_new_generator(h_oper, &gB, &h_genB), "interrupter generator");

 // FORCE TO ZERO RIGHT HERE, not at the end of the function.
 // mcpwm_new_generator first of all resets the generator, and that reset
 // REMOVES the continuous force. From this moment the pins are bound to the
 // MCPWM matrix, and the digitalWrite(LOW) from setup no longer acts on them -
 // while any of the early exits below would leave both power pins with
 // no force at all for tens of milliseconds, until outputs_off() at the end of
 // setup. All that time safety would rest on the peripheral's reset state,
 // i.e. on nothing the firmware does.
 ramp_off_ll();
 bridge_force_ll(0);
 g_mc_regs_ok = true;             // the group clock is on, the registers are alive

 // Ramp: HIGH on counter reset, LOW on comparator match - exactly what
 // Fast PWM 14 did with the non-inverting OC1A output.
 // THESE TWO CALLS DEFINE THE PWM ITSELF, and their failure is worse than all
 // the others. If the first passes and the second fails, GPIO19 will rise on
 // every counter reset and NEVER fall: one hundred percent duty on the buck
 // continuously, from the moment the level force is removed. The index
 // self-test does not see this - it only checks the force.
 MC_STEP(mcpwm_generator_set_actions_on_timer_event(h_genA,
   MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
       MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
   MCPWM_GEN_TIMER_EVENT_ACTION_END()), "ramp action on counter reset");
 MC_STEP(mcpwm_generator_set_actions_on_compare_event(h_genA,
   MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, h_cmp,
       MCPWM_GEN_ACTION_LOW), MCPWM_GEN_COMPARE_EVENT_ACTION_END()),
   "ramp action on compare match - WITHOUT IT 100% DUTY");

 // Both outputs fall to ZERO on a fault by one event, with no code involved.
 // OST mode is "the operator does not recover by itself, even if the fault
 // signal has gone". That is literally the same semantics as the
 // estop_triggered flag, only in silicon. Recovery is
 // mcpwm_operator_recover_from_fault(), and it returns an error while the
 // finger is on the button: the hardware implements estop_ready by itself.
 // The hardware brake is assembled from FIVE calls, and it is declared working
 // only if ALL of them passed. If the flag were set on the success of the first
 // one alone and the callback registration failed, the brake in silicon would
 // work, on_ost would never be called, the firmware would never learn about
 // the latch - and the banner would still report three layers of protection.
 mcpwm_gpio_fault_config_t fc = {};
 fc.group_id = 0;
 fc.gpio_num = PIN_BTN_ESTOP;
 fc.flags.active_level = 0;             // the button switches to GND
 fc.flags.pull_up = 1;
 esp_err_t be = mcpwm_new_gpio_fault(&fc, &h_flt);
 if (be == ESP_OK) {
 mcpwm_brake_config_t bc = {};
 bc.fault = h_flt;
 bc.brake_mode = MCPWM_OPER_BRAKE_MODE_OST;
 esp_err_t e1 = mcpwm_operator_set_brake_on_fault(h_oper, &bc);
 esp_err_t e2 = mcpwm_generator_set_actions_on_brake_event(h_genA,
   MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
       MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW),
   MCPWM_GEN_BRAKE_EVENT_ACTION_END());
 esp_err_t e3 = mcpwm_generator_set_actions_on_brake_event(h_genB,
   MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
       MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW),
   MCPWM_GEN_BRAKE_EVENT_ACTION_END());
 mcpwm_operator_event_callbacks_t ocb = {};
 ocb.on_brake_ost = on_ost;
 esp_err_t e4 = mcpwm_operator_register_event_callbacks(h_oper, &ocb, NULL);
 g_brake_ok = (e1 == ESP_OK && e2 == ESP_OK && e3 == ESP_OK && e4 == ESP_OK);
 if (!g_brake_ok) {
 // A FAILURE, not a note in the banner. The fault input was created, so the
 // latch in silicon is armed; but if the callback did not register, the
 // firmware will NEVER learn about it. The result is "outputs dead, firmware
 // thinks it is firing": the pulse counter grows, a green STATUS goes to the
 // display, there is no arc, the operator turns the amplitude up. The COMPLETE
 // absence of the brake (input not created) is a different matter: there is
 // simply one protection layer fewer, and that state is consistent, it can run.
 Serial.println(F("FATAL: fault input exists but the brake wiring is incomplete - inconsistent state"));
 // EXIT, not an assignment. A plain g_mcpwm_ok = false here would be a dead
 // write: the last line of the function sets it to true unconditionally, and
 // the protection would not work at all. We exit before starting the
 // carrier: then g_mcpwm_ok stays false, and the callback never runs, so the
 // firing gate is closed in two independent ways. There is no point starting
 // the carrier with inconsistent wiring anyway.
 return;
 }
 } else {
 Serial.print(F("MCPWM: fault input not created -> ")); Serial.println(esp_err_to_name(be));
 }

 // The force was already applied above, right after the generators were created.

 mcpwm_timer_event_callbacks_t tcb = {};
 tcb.on_empty = on_tez;                 // register BEFORE mcpwm_timer_enable
 MC_STEP(mcpwm_timer_register_event_callbacks(h_tmr, &tcb, NULL), "carrier callback registration");
 MC_STEP(mcpwm_timer_enable(h_tmr), "timer enable");
 MC_STEP(mcpwm_timer_start_stop(h_tmr, MCPWM_TIMER_START_NO_STOP), "timer start");

 g_mcpwm_ok = true;
}

// A check of the assumption, not faith in it. We address the registers
// directly by the indices OPER_ID/GEN_RAMP, assuming the driver gave us
// operator 0 and generators 0/1. That is true only on a clean boot, when
// nobody else has taken MCPWM. Here it is VERIFIED: set the force through
// the public call and read the field our macros address. No match - no run.
bool mcpwm_index_selftest() {
 if (!g_mcpwm_ok) return false;
 // Under the force lock: the carrier and the button interrupt are already
 // alive, and this is the only writer of gen_force outside FORCE_LOCK.
 FORCE_LOCK();
 const esp_err_t ea = mcpwm_generator_set_force_level(h_genA, 0, true);
 const bool a = (MCPWM_HW->operators[OPER_ID].gen_force.gen_a_cntuforce_mode == 1);
 const esp_err_t eb = mcpwm_generator_set_force_level(h_genB, 0, true);
 const bool b = (MCPWM_HW->operators[OPER_ID].gen_force.gen_b_cntuforce_mode == 1);
 FORCE_UNLOCK();
 if (ea != ESP_OK || eb != ESP_OK) return false;
 return a && b;
}

inline void set_buck_duty(uint16_t duty) {
 if (duty > MAX_DUTY) duty = MAX_DUTY;
 // Zero is output BY LEVEL, not by a zero compare. On the AVR, with
 // OCR1A = BOTTOM a narrow one-cycle pulse remained every period - the
 // "switched-off" generator twitched the TC4420 input 24961 times a second.
 // On MCPWM with cmp = 0 the compare event coincides with the counter reset
 // and the driver does not promise how the conflict is resolved at all. A
 // level force removes both questions at once.
 if (duty == 0) { ramp_off_ll(); return; }
 cmp_write_ll(duty);
 ramp_release_ll();
}

// Clamped wick: the warm-up baseline can NOT be higher than the effective peak.
inline uint16_t clamped_wick(int32_t effective_peak) {
 int32_t w = (int32_t)wick_offset;
 if (w > effective_peak) w = effective_peak;
 if (w < 0) w = 0;
 return (uint16_t)w;
}

// Busy-wait with watchdog service and an IMMEDIATE emergency stop.
// Returns false if E-STOP was pressed during the wait.
//
// Counting uses the core cycle counter: ±1 cycle versus ±10 us for micros()
// on the AVR (4 us granularity there plus an overrun of one iteration).
// Pre-emption by the scheduler can only LENGTHEN the window, and all three
// extensions are safe: a longer warm-up - the LC settles further with the
// bridge disabled; a later bridge enable - start is counted AFTER the wait,
// the shape is not distorted; a longer discharge - exactly what it was raised
// from 1 ms to 2 ms for in v5.8.
//
// The buck phases DELIBERATELY stayed here and did not move to carrier ticks:
// 2000 us / 40.06 us = 49.92, there is no whole number of periods, and binding
// to TEZ would shift the phase boundary by 0...40 us. Plus the bridge-enable
// edge would stop being asynchronous to the carrier and would land EXACTLY at
// the moment the buck high-side switch turns on - the worst phase for
// switching noise.
inline bool busy_wait_us(uint32_t us) {
 const uint32_t t0 = esp_cpu_get_cycle_count();
 const uint32_t n  = us * CPU_MHZ;
 const uint32_t kick_step = CPU_MHZ * 1000UL;        // feed the watchdog once per 1 ms
 uint32_t next_kick = kick_step;
 for (;;) {
 uint32_t el = esp_cpu_get_cycle_count() - t0;
 if (el >= n) break;
 if (el >= next_kick) { wdt_kick(); next_kick += kick_step; }
 g_isr_since_task = 0;
 // Check BOTH THE RAW LEVEL AND THE FLAGS FROM THE INTERRUPTS.
 // The level alone is not enough: a nanosecond dip on the cable drops the
 // outputs through the hardware brake or the interrupt, and by the time of
 // the poll the level has already recovered - the wait loop notices nothing
 // and returns true. Then fire_pulse calmly proceeds to enabling the bridge
 // and UNDOES the shutdown made on the fault. The window here is not a dozen
 // instructions but the whole two milliseconds of buck warm-up.
 if (ESTOP_DOWN() || g_estop_req || g_brake_ost || g_stall_kill) {
 emergency_stop();   // <- drops the outputs and latches estop immediately
 return false;
 }
 }
 // And once more AFTER the timeout exit: the break comes before the check, and
 // without this a fault arriving in the last iteration would return true.
 if (ESTOP_DOWN() || g_estop_req || g_brake_ost || g_stall_kill) {
 emergency_stop();
 return false;
 }
 return true;
}

// Carries a fault seen by an interrupt or by the hardware brake into software.
void estop_sync_from_isr() {
 const bool stall = g_stall_kill; if (stall) g_stall_kill = false;
 const bool brake = g_brake_ost;  if (brake) g_brake_ost  = false;
 const bool bbtn  = g_brake_btn;  if (brake) g_brake_btn   = false;
 const bool req   = g_estop_req;  if (req)   g_estop_req  = false;
 const bool emc   = brake && !bbtn;      // brake without a pressed button = glitch

 // The hardware brake ALWAYS requires clearing the latch in hardware, whatever
 // caused it. Set this before all other decisions and before any early return.
 if (brake) g_need_recover = true;

 // The brake source is the same E-STOP button, so the brake itself does not
 // yet indicate a glitch. We distinguish by the level: the operator's finger
 // is still on the button at this moment, while a nanosecond dip from
 // switching is already gone. Without this distinction the message would be
 // printed on every normal press and would stop being an EMI indicator - which
 // is what it was introduced for.
 if (emc)
 Serial.println(F(">> MCPWM hardware brake without a pressed button - glitch on the cable <<"));
 if (stall)
 Serial.println(F(">> SAFE: the task did not respond for more than 160 ms <<"));

 if (!(stall || brake || req)) return;
 if (estop_triggered) return;

 // THE estop_ready GATE IS HERE, AT THE DECISION, not at setting the flags.
 // The brake is set by the same button, and after the fault is cleared the
 // very first contact bounce would give a new brake event and latch the fault
 // back - the same "press it several times", only through hardware.
 //
 // ONLY the button-related paths obey the gate. A task stall and a glitch on
 // the cable have nothing to do with the clearing gesture and always latch.
 // Without this exception a glitch would go into an endless loop: brake,
 // message, recovery, brake again - the port floods, and the latch never
 // happens.
 if (!stall && !emc && !estop_ready) return;

 emergency_stop();
 if (!stall && !emc) Serial.println(F(">> ESTOP <<"));
}

// Clearing the hardware latch. The order is MANDATORY: if E-STOP was pressed
// in the middle of a ramp, the comparator still holds a value from the middle
// of the rise, and a bare recovery would turn the buck on at that duty
// CONTINUOUSLY - with no pulse and no time limit. So first force to zero, then
// zero the comparator under the force, and only then recover.
void estop_recover_hw() {
 if (!g_need_recover || !g_mcpwm_ok) return;
 if (ESTOP_DOWN()) return;                // the hardware would refuse anyway
 ramp_off_ll();                           // and the comparator to safe, not to zero
 bridge_force_ll(0);
 if (!h_flt || mcpwm_operator_recover_from_fault(h_oper, h_flt) == ESP_OK) {
 g_need_recover = false;
 // DO NOT clear g_brake_ost HERE. A brake arriving between the flag sync and
 // this line would be swallowed silently: no message, no latch, and the
 // hardware brake would stay tripped. The machine would look healthy, the
 // outputs would be dead, and fire_pulse would happily count pulses that do
 // not exist. Only estop_sync_from_isr clears the flag.
 }
}

// ############################################################################
// ############################################################################
// ##                                                                        ##
// ##   EXTERNAL DISPLAY - ramp-shape output to the CYD (ESP32-2432S028R)    ##
// ##                                                                        ##
// ##   Everything below, down to the END OF DISPLAY SECTION mark, has       ##
// ##   nothing to do with ramp generation and can be cut out entirely       ##
// ##   together with three calls: qcw_display_begin() in setup(),           ##
// ##   preview_invalidate() at the end of update_pots() and                 ##
// ##   qcw_display_tick() in loop().                                        ##
// ##                                                                        ##
// ############################################################################
// ############################################################################
//
//  WHAT IT DOES: runs the very same calculate_duty() over 80 points, packs
//  the result into frames and sends it to the display. There is no second
//  copy of the shape math anywhere - change calculate_duty and the display
//  follows by itself.
//
//  ROLES: the generator is the MASTER of the hardware I2C on GPIO21 (SDA) and
//         GPIO22 (SCL), the ESP32/CYD is the slave at 0x2A. The exchange is
//         ONE-WAY: the display only receives and draws, it cannot affect the
//         pulse shape.
//
//  BUS vs. RAMP: the link is driven only from loop(), and fire_pulse is
//  blocking - so no bus activity ever overlaps a shot, and tx_window_open()
//  additionally keeps the bus quiet for TX_GUARD_MS before the next pulse in
//  AUTO. The ESP32 hardware Wire master on GPIO21/22 is used (see PIN
//  DEFINITIONS); the bit-banged master of the earlier AVR build is gone.
//
//  WIRING:
//      GPIO21 --- SDA ---------------------------- CN1.IO27 CYD
//      GPIO22 --- SCL ---------------------------- CN1.IO22 CYD
//      GND --------------------------------------- CN1.GND  CYD
//      two 4.7k pull-ups from SDA and SCL to 3.3 V - on the control board
//      (R16/R17 to the ESP32 3V3 rail); CN1.3V3 of the display is not used
//  The pull-ups to 3.3 V replace a level shifter: the bus physically cannot
//  rise above 3.3 V, and there is nowhere for 5 V to reach the ESP32 inputs
//  from. Both sides are 3.3 V. Wire.begin() enables the weak internal
//  pull-ups (~45 k) by itself (observed with ESP32 core 3.3.x), in parallel
//  with the external 4.7 k - that is harmless.
//
// ============================================================================

#define LINK_SLAVE_ADDR 0x2A
// LINK_FREQ_HZ - see BUILD CONFIGURATION at the top of the file.
// I2C transaction timeout. FIFTY, and this is a measured value, not a chosen
// one (reference build, ESP32 core 3.3.x).
//
// The tempting argument "the bus timeout must be less than the transmit window
// margin TX_GUARD_MS, so that a stuck bus does not creep into a pulse" is right
// in substance, but a value like 6 ms does not work: on the live bus EVERY
// frame ran into exactly that timeout and never went through, including the
// six-byte STATUS.
// Measured: with 50 ms the same transaction completes in ~1000 us with code 0.
// At 6 ms the link did not degrade - it was absent completely.
//
// What we pay: a stuck bus can delay a loop pass by up to 50 ms, and in AUTO
// the shot moves to the next iteration. That is a longer pause, i.e. the safe
// side - the bank discharges more, not less.
#define LINK_WIRE_TIMEOUT_MS 50
#define LINK_FW_MAJOR   5
#define LINK_FW_MINOR   9

#define PREVIEW_POINTS 80     // points in the curve (three I2C chunks)
#define PREVIEW_RISE_N 64     // of them on the RISE; the remaining 16 - on the fall
#define PREVIEW_BATCH  80     // ALL points in one preview_tick() call: tens of microseconds on
                              // the ESP32 (prev= in the POTS line); spreading them over several
                              // passes would push the send out of the window between pulses

// IMPORTANT: the time grid is NON-UNIFORM. A uniform one does not work: with a
// small fall_x100 the whole fall takes a few percent of the pulse length and
// gets 2-3 samples for the full swing - an abrupt cut-off would turn into a
// gentle line, exactly in the place QCW is tuned for.
// The first PREVIEW_RISE_N points cover the rise [0, t_rise_end) uniformly,
// the rest cover the fall. The point with index PREVIEW_RISE_N lands EXACTLY
// on the knee. The fall in calculate_duty is strictly linear, 16 points cover
// it without error.
// Verified against calculate_duty on 320 samples with the constants in this
// file: worst error 0.67 % of PWM_TOP, which is already almost pure 8-bit
// quantisation (1 LSB = 0.39 %). Re-run the comparison if you change the grid.

static uint8_t  preview_buf[PREVIEW_POINTS];
static bool     preview_ready = false;
static uint16_t preview_rise_end_x10000 = 10000;

// Snapshot of the parameters the current buffer was computed from. The display
// captions the curve with EXACTLY these numbers, not with the live knob values:
// otherwise the caption and the curve drift apart while a recalculation runs.
// 0=pulse_width_ms 1=peak_duty 2=wick_offset 3=fall_x100 4=tscale 5=uscale
#define PREVIEW_SNAP_N 6
static uint16_t preview_snap[PREVIEW_SNAP_N] = {0, 0, 0, 0, 0, 0};

static uint16_t link_pwm_top     = PWM_TOP;
static uint16_t link_bus_voltage = BUS_VOLTAGE;
static uint8_t  s_next_idx       = 0;
static uint32_t s_pulse_us       = 10000UL;
static uint32_t s_rise_end_us    = 10000UL;
static bool     s_snap_valid     = false;
static uint16_t s_bps_snap       = 0;      // BPS is tracked separately, see below
static bool     s_bps_valid      = false;
static uint32_t s_last_full      = 0;      // when the full set was last sent
static uint8_t  s_dirty          = 0;    // bits 0..3: META, P0, P1, P2
// Preview generation (3 bits) in the frame type of META and the chunks. The
// display accepts only chunks of the generation announced by the last META: a
// META lost to CRC does not let chunks of a new shape mix with the old caption.
// Changes on every shape recalculation; the heartbeat and a lone META for BPS
// do not change the generation.
// For the old display (types 0x10/0x40..) this is generation 0 - compatible.
static uint8_t  s_gen            = 0;
static bool     s_last_ready     = false;
static uint32_t s_last_status    = 0;
static uint32_t s_retry_at       = 0;    // pause after a failed transmission

#define FRM_STATUS   0x00
#define FRM_META     0x10   // | gen (3 bits): preview generation, see s_gen
#define FRM_PREVIEW0 0x40   // | gen<<3 | idx: chunk idx of generation gen
#define STATUS_LEN 6
#define META_LEN   22
#define CHUNK0_LEN 30
#define CHUNK1_LEN 30
#define CHUNK2_LEN (PREVIEW_POINTS - CHUNK0_LEN - CHUNK1_LEN)   // 20
#define STATUS_PERIOD_MS 100   // STATUS costs ~0.4 ms at 250 kHz, the counter is livelier

static uint8_t s_meta[META_LEN];

// --- Bus ---------------------------------------------------------------------
// Hardware I2C. The byte order in a frame [type][len][payload][crc] is kept
// BYTE FOR BYTE: the parser on the CYD reads type from b[0], len from b[1] and
// checks crc8 over (len+2) bytes. Losing or swapping len would drop ALL frames,
// and it would look like a link loss by LINK_TIMEOUT_MS, not like an error.
//
// WHAT WAS GIVEN UP DELIBERATELY: bit-level abort of a frame on E-STOP right in
// the middle of a transmission. Wire.endTransmission is atomic. The
// compensation is not in the bus but in the architecture: a fault drops
// the outputs through the MCPWM hardware brake and the interrupt, and neither
// depends on what the task is busy with.

// CRC-8, polynomial 0x07, init 0x00, MSB-first.
static uint8_t link_crc8(const uint8_t* p, uint8_t n) {
 uint8_t crc = 0;
 while (n--) {
 crc ^= *p++;
 for (uint8_t b = 0; b < 8; b++)
 crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
 }
 return crc;
}

static inline void put16(uint8_t* p, uint16_t v) {
 p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8);
}



// Frame: [type][len][payload...][crc8 over type+len+payload].
static bool send_frame(uint8_t type, const uint8_t* payload, uint8_t len) {
 uint8_t hdr[2] = {type, len};
 uint8_t crc = link_crc8(hdr, 2);
 for (uint8_t i = 0; i < len; i++) {
 crc ^= payload[i];
 for (uint8_t b = 0; b < 8; b++)
 crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
 }

 Wire.beginTransmission((uint8_t)LINK_SLAVE_ADDR);
 Wire.write(type);
 Wire.write(len);
 if (len) Wire.write(payload, len);
 Wire.write(crc);
 bool ok = (Wire.endTransmission(true) == 0);
 wdt_kick();
 return ok;
}

// Has the knob moved by more than the tolerance. Returns true = real movement.
static inline bool pot_moved(uint16_t a, uint16_t b, uint16_t tol) {
 uint16_t d = (a > b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
 return d > tol;
}

// Checks whether the shape parameters changed; if so - restarts the
// recalculation. Cheap: six comparisons. Called from update_pots().
void preview_invalidate() {
 // A set is still on its way (the window between pulses is closed or the bus
 // is busy): do NOT touch it. If every knob poll (50 ms) reset s_dirty, then in
 // AUTO with a short window the set would never arrive while a knob is being
 // turned - the display would freeze. The parameter snapshot is not updated,
 // so a knob movement will be noticed by the next poll, once the set is gone.
 if (s_dirty) return;
 uint16_t pw = pulse_width_ms, peak = peak_duty, wick = wick_offset;
 uint16_t fall = fall_x100, tscale = tscale_x100, uscale = uscale_x100;

 // BPS (the BPS knob) does not affect the ramp SHAPE, so there is no need to
 // recompute the preview because of it - resending META is enough. Checking it
 // separately is MANDATORY: it is not in the list of shape parameters, so
 // without this branch it would reach the display only incidentally, when
 // ADC jitter happened to trigger a shape recalculation - and with the jitter
 // suppressed by the pot_moved tolerances BPS on the display would freeze.
 if (!s_bps_valid || pot_moved(bps_x10, s_bps_snap, 1)) {
 s_bps_snap  = bps_x10;
 s_bps_valid = true;
 build_meta();
 s_dirty |= 0x01;
 }

 // Tolerance for ADC jitter. Without it ±1 count on any knob recomputed the
 // preview, pushed a new META plus three curve chunks to the display, and the
 // display erased and redrew the graph area round and round - constant
 // flicker at rest.
 // IMPORTANT: the tolerance applies ONLY to the snapshot for the display. In
 // the ramp path peak_duty, wick_offset and the rest are used exact, unrounded.
 if (s_snap_valid &&
 !pot_moved(pw,     preview_snap[0], 0) &&
 !pot_moved(peak,   preview_snap[1], 3) &&
 !pot_moved(wick,   preview_snap[2], 3) &&
 !pot_moved(fall,   preview_snap[3], 1) &&
 !pot_moved(tscale, preview_snap[4], 2) &&
 !pot_moved(uscale, preview_snap[5], 2)) {
 return;   // the knobs have not moved - nothing to recompute
 }

 preview_snap[0] = pw;   preview_snap[1] = peak;   preview_snap[2] = wick;
 preview_snap[3] = fall; preview_snap[4] = tscale; preview_snap[5] = uscale;
 s_snap_valid = true;
 s_gen = (uint8_t)((s_gen + 1) & 7);      // new shape - new generation

 s_pulse_us = (uint32_t)pw * 1000UL;
 if (s_pulse_us < 2) s_pulse_us = 2;

 // The same boundary as RISE_END_X10000 inside calculate_duty.
 uint32_t rise_end_x10000 = 10000UL - (uint32_t)fall * 100UL;
 if (rise_end_x10000 > 10000UL) rise_end_x10000 = 10000UL;
 preview_rise_end_x10000 = (uint16_t)rise_end_x10000;
 s_rise_end_us = (s_pulse_us * rise_end_x10000) / 10000UL;
 if (s_rise_end_us > s_pulse_us - 1) s_rise_end_us = s_pulse_us - 1;

 s_next_idx    = 0;
 preview_ready = false;
 s_last_ready  = false;   // otherwise the "became ready" edge is lost: with PREVIEW_BATCH 80
                          // preview_tick finishes and sets true again BEFORE the check

 // Clear ALL bits, including META. preview_buf is about to be overwritten in
 // place right now, and sending a chunk from it would give the display a
 // splice of the old and the new curve.
 //
 // Do NOT preserve the META bit either. It is set not only in the BPS branch
 // (where build_meta has indeed just run), but also on the preview_ready edge
 // via s_dirty = 0x0F. If that set has not gone out yet, a preserved bit would
 // send a LONE META of the new generation on top of the curve of the old one:
 // on the display the right edge of the ramp would jump while the shape stayed
 // the same. Nothing is lost - once the preview is ready the full 0x0F is set
 // anyway, together with a fresh build_meta().
 s_dirty = 0;
}

// Computes the next PREVIEW_BATCH points - all 80 in one call (the time it
// takes is printed in the POTS line as prev=). Not called during a shot:
// fire_pulse is blocking, and loop() simply does not run then.
static uint32_t g_prev_us = 0;      // diagnostics: time of the last full computation
static void preview_tick() {
 if (preview_ready) return;
 const uint32_t t_prev0 = esp_cpu_get_cycle_count();

 uint8_t end = s_next_idx + PREVIEW_BATCH;
 if (end > PREVIEW_POINTS) end = PREVIEW_POINTS;

 for (uint8_t i = s_next_idx; i < end; i++) {
 // The last point is taken at (pulse_us - 1), not at pulse_us:
 // calculate_duty returns 0 for t >= pulse_us, and the graph would get a
 // false drop to zero at the very edge regardless of the shape.
 uint32_t t;
 if (i < PREVIEW_RISE_N) {
 t = ((uint32_t)i * s_rise_end_us) / PREVIEW_RISE_N;
 } else {
 uint32_t span = ((s_pulse_us - 1) > s_rise_end_us) ? (s_pulse_us - 1) - s_rise_end_us : 0;
 t = s_rise_end_us +
 ((uint32_t)(i - PREVIEW_RISE_N) * span) / (PREVIEW_POINTS - 1 - PREVIEW_RISE_N);
 }
 int32_t duty = calculate_duty(t, s_pulse_us, MODE_QCW);
 if (duty < 0) duty = 0;
 if (duty > (int32_t)link_pwm_top) duty = (int32_t)link_pwm_top;
 preview_buf[i] = (uint8_t)(((uint32_t)duty * 255UL) / link_pwm_top);
 }

 s_next_idx = end;
 if (s_next_idx >= PREVIEW_POINTS) {
 preview_ready = true;
 g_prev_us = (esp_cpu_get_cycle_count() - t_prev0) / CPU_MHZ;
 }
}

static void build_meta() {
 put16(&s_meta[0],  preview_snap[0]);      // pulse_width_ms
 put16(&s_meta[2],  bps_x10);
 put16(&s_meta[4],  preview_snap[1]);      // peak_duty
 put16(&s_meta[6],  preview_snap[2]);      // wick_offset
 put16(&s_meta[8],  preview_snap[3]);      // fall_x100
 put16(&s_meta[10], preview_snap[4]);      // tscale_x100
 put16(&s_meta[12], preview_snap[5]);      // uscale_x100
 put16(&s_meta[14], link_pwm_top);
 put16(&s_meta[16], link_bus_voltage);
 put16(&s_meta[18], preview_rise_end_x10000);
 s_meta[20] = PREVIEW_POINTS;
 s_meta[21] = PREVIEW_RISE_N;
}

// The window in which the bus may be used. Outside AUTO - always. In AUTO -
// during the pause between shots except its last TX_GUARD_MS: a frame (~1.5 ms
// at 250 kHz) must not creep up to the ramp start. See tx_window_open().
//
// SECOND CONDITION - a pause after a failure. If the display is not connected
// or unpowered, the pull-ups are dead, SCL does not rise, and every attempt
// costs a failed Wire transaction (up to LINK_WIRE_TIMEOUT_MS). Without the
// pause this would repeat on EVERY loop() pass and keep adding to the E-STOP
// reaction delay.
#define LINK_RETRY_MS 250

// Period of the unconditional resend of the whole set, ms. Cures a frame lost
// to a bad CRC: there is no back channel, and without this the mismatch would
// persist until the next knob movement.
#define LINK_REFRESH_MS 1000   // the set costs ~6 ms at 250 kHz - once a second is fine

// Margin before the start of the next pulse, ms. A frame at 250 kHz costs
// ~1.5 ms (measured on the reference build); we take the margin with overlap:
// while there is more than this until the pulse, the bus may be used.
#define TX_GUARD_MS 4

static bool tx_window_open() {
 if ((int32_t)(millis() - s_retry_at) < 0) return false;   // waiting after a failure
 if (estop_triggered) return true;          // outputs are off, no hurry
 if (!fire_armed || !auto_mode) return true;

 uint16_t b = bps_x10 ? bps_x10 : 1;
 uint32_t cycle_ms = 10000UL / b;
 uint32_t elapsed  = millis() - last_pulse_ms;   // from the START of the previous pulse

 // The window is counted from the END of the pause, not from its start.
 //
 // Why not simply (elapsed < cycle_ms / 2), i.e. the first half of the period
 // counted from the pulse start: the pulse itself is blocking and eats almost
 // all of that half - warm-up 2 ms + bridge enable 1 ms + ramp length +
 // discharge 2 ms. With a 25 ms ramp at 15 BPS the period is 66 ms, the half
 // is 33 ms, the pulse 30 ms - 3 ms would remain for the transmission, less
 // than one frame (3.7 ms at 100 kHz, ~1.5 ms at 250 kHz). NOTHING would go
 // out, STATUS included, and after two seconds the display would drop the
 // link as soon as AUTO is switched on.
 //
 // So the whole pause is allowed, except the tail before the next pulse.
 // At the same 25 ms and 15 BPS the free window is ~30 ms instead of 3 - the
 // full set of ~6 ms fits with margin.
 if (elapsed >= cycle_ms) return false;          // the pulse is about to start
 return (cycle_ms - elapsed) > TX_GUARD_MS;
}

// Send with back-off: a failure arms the retry timer.
static bool link_try(uint8_t type, const uint8_t* payload, uint8_t len) {
 if (send_frame(type, payload, len)) return true;
 s_retry_at = millis() + LINK_RETRY_MS;
 return false;
}

// Display initialisation. One call from setup().
void qcw_display_begin(uint16_t pwm_top, uint16_t bus_voltage) {
 link_pwm_top     = (pwm_top == 0) ? 1 : pwm_top;
 link_bus_voltage = bus_voltage;

 // The bus is brought up once in setup(); here only the protocol state.

 s_next_idx    = 0;
 preview_ready = false;
 s_snap_valid  = false;
 for (uint8_t i = 0; i < PREVIEW_POINTS; i++) preview_buf[i] = 0;

 build_meta();
 s_dirty       = 0x0F;
 s_last_status = millis();
}

// Finishes the preview and sends the pending frames: STATUS when due, then up
// to four curve frames per call, the window re-checked before each. One call
// from loop().
void qcw_display_tick() {
 preview_tick();

 if (preview_ready && !s_last_ready) {
 build_meta();
 s_dirty = 0x0F;               // META + three curve chunks
 s_last_full = millis();
 }
 s_last_ready = preview_ready;

 // Heartbeat: once a second the whole set is resent, even if nothing changed.
 // There is no back channel - the display silently drops a frame with a bad
 // CRC, while the generator clears the s_dirty bit on the bus ACK and
 // considers the frame delivered. Without the heartbeat one lost frame would
 // freeze a third of the curve or the caption FOREVER, until a knob is touched
 // (ADC jitter cannot be relied on to trigger a resend: the pot_moved
 // tolerances suppress it). One second is half the link timeout on the
 // display (2 s, LINK_TIMEOUT_MS in cyd_ramp_view.ino).
 if (preview_ready && (millis() - s_last_full) >= LINK_REFRESH_MS) {
 build_meta();
 s_dirty = 0x0F;
 s_last_full = millis();
 }

 if (!tx_window_open()) return;

 // STATUS goes FIRST, before the curve. If it stood after the curve loop,
 // behind a check like "if (s_dirty) return;", one consistently failing
 // frame would be enough to lock the heartbeat forever: the chunk does not go
 // -> the bit does not clear -> STATUS is never reached -> after LINK_TIMEOUT_MS
 // the display drops the link, although the generator is alive. The curve is
 // decoration, STATUS is the sign of life, and it has the priority.
 uint32_t now = millis();
 if (now - s_last_status >= STATUS_PERIOD_MS) {
 s_last_status = now;
 uint8_t st[STATUS_LEN];
 uint8_t flags = 0;
 if (fire_armed)      flags |= 0x01;
 if (auto_mode)       flags |= 0x02;
 if (estop_triggered) flags |= 0x04;
 if (preview_ready)   flags |= 0x08;
 st[0] = LINK_FW_MAJOR;
 st[1] = LINK_FW_MINOR;
 st[2] = flags;
 put16(&st[3], (uint16_t)(pulse_counter & 0xFFFF));
 st[5] = 0;
 link_try(FRM_STATUS, st, STATUS_LEN);
 if (!tx_window_open()) return;   // the frame ate the window - the curve can wait
 }

 // Up to four frames go out per call, but the window is RE-CHECKED before
 // each: it still cannot creep into a pulse. With EXACTLY ONE frame per call
 // AUTO would starve the link: with a long pulse at high BPS the chunks P1 and
 // P2 would never be reached - the right part of the curve on the display
 // would freeze forever.
 uint8_t budget = 4;
 while (s_dirty && budget--) {
 if (!tx_window_open()) return;
 const uint8_t tm = (uint8_t)(FRM_META | s_gen);
 const uint8_t tp = (uint8_t)(FRM_PREVIEW0 | (s_gen << 3));
 if      (s_dirty & 0x01) { if (!link_try(tm, s_meta, META_LEN)) return; s_dirty &= (uint8_t)~0x01; }
 else if (s_dirty & 0x02) { if (!link_try((uint8_t)(tp + 0), &preview_buf[0], CHUNK0_LEN)) return; s_dirty &= (uint8_t)~0x02; }
 else if (s_dirty & 0x04) { if (!link_try((uint8_t)(tp + 1), &preview_buf[CHUNK0_LEN], CHUNK1_LEN)) return; s_dirty &= (uint8_t)~0x04; }
 else if (s_dirty & 0x08) { if (!link_try((uint8_t)(tp + 2), &preview_buf[CHUNK0_LEN + CHUNK1_LEN], CHUNK2_LEN)) return; s_dirty &= (uint8_t)~0x08; }
 }
}

// ############################################################################
// ##                        END OF DISPLAY SECTION                          ##
// ############################################################################

// ============================================================================
// SETUP
// ============================================================================

// Prints the REAL configuration computed from the macros. Comments in the code
// go stale when PWM_TOP/BUS_VOLTAGE change - this does not. This is the source
// of truth.
void print_config_banner() {
 Serial.println(F("---------------- CONFIG (computed from macros) ------------------"));
 Serial.print(F("  PWM_TOP     = ")); Serial.print(PWM_TOP);
 Serial.print(F("   -> carrier = ")); Serial.print(PWM_FREQ_HZ); Serial.println(F(" Hz"));
 Serial.print(F("  MAX_DUTY    = ")); Serial.print((int32_t)MAX_DUTY);
 Serial.println(F("   (100 % duty)"));
 Serial.print(F("  AMPL_POT_MAX= ")); Serial.print((int32_t)AMPL_POT_MAX);
 Serial.print(F("   (x")); Serial.print(1000 + ALPHA_X1000_DEFAULT);
 Serial.println(F("/1000 alpha -> peak = MAX_DUTY)"));
 Serial.print(F("  Bus         = ")); Serial.print(BUS_VOLTAGE); Serial.println(F(" V"));
 Serial.print(F("  WICK floor  = duty ")); Serial.print((int32_t)WICK_MIN_DUTY);
 Serial.print(F("  -> ~")); Serial.print(WICK_MIN_VOLTS);
 Serial.print(F(" V ideal (target ")); Serial.print(WICK_VOLTAGE);
 Serial.println(F(" V real)"));
 Serial.print(F("  WICK ceiling= duty ")); Serial.print((int32_t)WICK_MAX_DUTY);
 Serial.print(F("  -> ~")); Serial.print(WICK_MAX_VOLTS);
 Serial.print(F(" V  (")); Serial.print(WICK_MAX_PCT_OF_BUS);
 Serial.println(F(" % of bus)"));
 Serial.print(F("  Disch.window= ")); Serial.print(BUCK_DISCHARGE_US);
 Serial.println(F(" us (GPIO4 HIGH after the ramp)"));
 Serial.println(F("-----------------------------------------------------------------"));
}

void setup() {
 // FIRST OF ALL - remove a possible hardware hold from the interrupter pin.
 // GPIO4 is in the RTC domain, and its hold SURVIVES a system reset. If it
 // ever caught the pin high, the bridge would stay enabled through the whole
 // reboot, and an external resistor cannot overpower an actively driven output.
 gpio_hold_dis((gpio_num_t)PIN_INTERRUPTER);
 gpio_hold_dis((gpio_num_t)PIN_PWM_RAMP);

 // Outputs to zero BEFORE everything else. The real protection in the boot
 // window is the external 10 k pull-downs at the TC4420 inputs: the code here
 // runs ~300 ms after reset, i.e. already AFTER the whole dangerous window.
 pinMode(PIN_INTERRUPTER, OUTPUT); digitalWrite(PIN_INTERRUPTER, LOW);
 pinMode(PIN_PWM_RAMP,    OUTPUT); digitalWrite(PIN_PWM_RAMP,    LOW);

 // The core frequency is fixed here and READ BACK, never left to the board
 // menu / build profile (whose default differs between board entries and core
 // versions - 160 MHz has been seen): busy_wait_us counts in cycles, and a
 // hard-coded constant with a 160 MHz core would give a 1.5x error in all
 // three buck phases.
 setCpuFrequencyMhz(CPU_MHZ);
 g_cpu_mhz = getCpuFrequencyMhz();

 Serial.begin(115200);
 delay(200);

 pinMode(PIN_BTN_FIRE,  INPUT_PULLUP);
 pinMode(PIN_BTN_AUTO,  INPUT_PULLUP);
 pinMode(PIN_BTN_ESTOP, INPUT_PULLUP);

 analogReadResolution(12);
 // Observed with ESP32 core 3.3.x: analogSetPinAttenuation before the first
 // analogRead on that pin is a no-op with an error in the log - the pin is not
 // yet bound to the ADC. So the attenuation is set as the GLOBAL default, and
 // the pins pick it up on the first read. 11 dB - full scale ~3.1 V for the
 // 3.3 V knob rail.
 analogSetAttenuation(ADC_11db);

 h_qcw = xTaskGetCurrentTaskHandle();

 // MCPWM is created BEFORE attachInterrupt: mcpwm_new_gpio_fault (ESP-IDF 5.x
 // driver as shipped with core 3.3.x) internally calls gpio_config with
 // intr_type = DISABLE and would remove an interrupt already installed.
 hw_init_mcpwm();
 attachInterrupt(digitalPinToInterrupt(PIN_BTN_ESTOP), estop_isr, CHANGE);

 Wire.begin(PIN_LINK_SDA, PIN_LINK_SCL, LINK_FREQ_HZ);
 Wire.setTimeOut(LINK_WIRE_TIMEOUT_MS);

 // Two watchdogs, and they do DIFFERENT things.
 // RTC_WDT is the true analogue of the AVR one: a hardware chip reset
 // bypassing the panic handler, the pins immediately go to high-impedance
 // input and the external resistors hold zero.
 // TWDT is only a logger, it fires earlier and WITHOUT panic: the panic
 // handler prints a backtrace, and that is tens of milliseconds during which
 // the outputs do not change.
 // idle_core_mask = 0 is mandatory: the task legitimately eats the core for
 // the whole pulse length and would starve that core's idle task.
 rtc_wdt_protect_off();
 rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
 rtc_wdt_set_time(RTC_WDT_STAGE0, WATCHDOG_TIMEOUT_MS);
 rtc_wdt_enable();
 rtc_wdt_protect_on();

 esp_task_wdt_config_t wc = {};
 wc.timeout_ms    = WATCHDOG_TIMEOUT_MS - 500;
 wc.idle_core_mask = 0;
 wc.trigger_panic = false;
 if (esp_task_wdt_reconfigure(&wc) != ESP_OK) esp_task_wdt_init(&wc);
 esp_task_wdt_add(NULL);
 wdt_kick();

 // Priority above background: otherwise equal-priority tasks would pre-empt
 // us once a millisecond right in the middle of a buck phase.
 vTaskPrioritySet(NULL, 10);

 Serial.println(F("=========================================="));
 Serial.println(F("QCW Generator v5.9-ESP32 (QCW only + CYD link)"));
 Serial.print(F("Ramp shift: "));
 Serial.println(ENABLE_BUCK_TIMING ? F("ON") : F("OFF"));
 Serial.println(F("=========================================="));
 Serial.println(F("Pins (ESP32 WROOM-32):"));
 Serial.println(F(" 16=FIRE 17=AUTO 23=ESTOP"));
 Serial.print(F(" 4=INTERRUPTER 19=PWM_RAMP ("));
 Serial.print(PWM_FREQ_HZ); Serial.println(F(" Hz)"));
 Serial.println(F(" 21=SDA 22=SCL (display link, 0x2A)"));
 Serial.println(F(" 25=PW 26=BPS 34=AMPL 35=WICK 32=FALL 27=CURV 33=CPOS"));
 Serial.print(F("CPU: ")); Serial.print(g_cpu_mhz); Serial.println(F(" MHz"));
 // A frequency mismatch is a failure, not a warning. All three buck phases and
 // the whole ramp length are computed by multiplying with the CONSTANT
 // CPU_MHZ; at 160 MHz every window would come out 1.5 times longer than set.
 if (g_cpu_mhz != CPU_MHZ) {
 g_mcpwm_ok = false;
 Serial.print(F("FATAL: core frequency ")); Serial.print(g_cpu_mhz);
 Serial.print(F(" MHz instead of ")); Serial.println(CPU_MHZ);
 }

 if (g_mcpwm_ok && !mcpwm_index_selftest()) {
 g_mcpwm_ok = false;
 Serial.println(F("FATAL: MCPWM indices do not match the direct register addressing"));
 } else if (!g_mcpwm_ok) {
 Serial.println(F("FATAL: operation forbidden - reason above"));
 } else if (!g_brake_ok) {
 // The hardware brake is the only layer that works without software. The
 // banner must not print "OST brake" unless it was really created: a false
 // "ok" in the fault chain is what the operator decides to start the machine
 // on.
 Serial.println(F("MCPWM: up, but NO HARDWARE BRAKE - two protection layers, not three"));
 } else {
 Serial.println(F("MCPWM: ok (16 MHz timer, OST hardware brake active)"));
 }
 outputs_off();
 // Not just print, but FORBID operation. Without this latch the firmware would
 // come up with uncontrolled outputs and allow arming.
 if (!g_mcpwm_ok) { estop_triggered = true; estop_ready = false; }

 print_config_banner();
 Serial.println();

 delay(50);
 btn_fire.last_state    = digitalRead(PIN_BTN_FIRE);
 btn_fire.stable_state  = btn_fire.last_state;
 btn_auto.last_state    = digitalRead(PIN_BTN_AUTO);
 btn_auto.stable_state  = btn_auto.last_state;
 btn_estop.last_state   = ESTOP_UP() ? HIGH : LOW;
 btn_estop.stable_state = btn_estop.last_state;

 Serial.print(F("Boot: FIRE=")); Serial.print(btn_fire.last_state);
 Serial.print(F(" AUTO=")); Serial.print(btn_auto.last_state);
 Serial.print(F(" ESTOP=")); Serial.println(btn_estop.last_state);
 Serial.println(F("(all should = 1 when not pressed)"));
 Serial.println();

 update_pots();

 // Plausibility gate: an operator does not set all seven knobs to the end stop
 // at the same time - that is what a BROKEN PANEL GROUND looks like, and it
 // gives the worst possible combination (amplitude at the ceiling, wick at the
 // ceiling, 25 ms, 15 BPS).
 // Pull-ups on the inputs are powerless against this by construction: they are
 // four orders of magnitude weaker than the failure path.
 if (peak_duty_raw >= (uint16_t)AMPL_POT_MAX && wick_offset >= (uint16_t)WICK_MAX_DUTY
     && pulse_width_ms >= PULSE_WIDTH_MAX_MS && bps_x10 >= BPS_MAX_X10) {
 Serial.println(F("FATAL: all knobs at the end stop - looks like a broken panel ground"));
 emergency_stop();
 }

 qcw_display_begin(PWM_TOP, BUS_VOLTAGE);

 Serial.print(F("Initial peak_duty=")); Serial.print(peak_duty);
 Serial.print(F("/")); Serial.println(AMPL_POT_MAX);
 if (peak_duty < 50) {
 Serial.println(F("WARN: peak_duty very low - check AMPL pot connection"));
 }
 Serial.println();
 Serial.print(F("Current mode: "));
 print_mode_name();
 Serial.println();

 // AS THE LAST ACTION, reset the reverse watchdog. The carrier starts in the
 // middle of initialisation, and the "interrupt alive, task dead" counter ticks
 // from that moment. Printing the banner over blocking Serial at 115200 costs
 // about 120 ms (measured on the reference build), delay(50) another fifty,
 // the first knob poll up to twelve: the 160 ms threshold is reached right at
 // boot, and without this reset the fault would latch on EVERY power-up and on
 // every DTR reset when the serial monitor is opened.
 // The price of such a false fault is not the fault itself, but that the
 // operator learns to clear it without reading - and will clear a real FATAL
 // with the same gesture.
 g_isr_since_task = 0;
 // AND THE FLAGS TOO. Resetting one counter is not enough: the threshold is
 // reached BEFORE this line, the callback manages to set g_stall_kill, and the
 // very first loop pass prints "SAFE: the task did not respond" and latches
 // the fault. The margin there is tens of milliseconds, so some boots went
 // through cleanly - i.e. the failure would be intermittent, which is worse
 // than permanent.
 g_stall_kill = false;
 g_estop_req  = false;
 // DO NOT clear g_brake_ost HERE - unlike the other two, it is not
 // self-restoring: the interrupt re-sets g_estop_req from the level every
 // 40 us, while the brake is set ONCE by an event. Erasing it means losing the
 // fact of the latch forever: there will be no recovery, the firmware comes up
 // with estop_triggered = false and outputs dead in silicon, fire_pulse runs
 // through completely, the pulse counter grows, a green STATUS goes to the
 // display - and there is no arc. The operator turns the amplitude up, and the
 // outputs come alive on the next real fault: the first real pulse goes out
 // at a blindly raised power. A level dip on the cable in the boot window is
 // enough for this.
}

void print_mode_name() {
 Serial.print(F("QCW (GPIO19 burst PWM, GPIO4 pulsed)"));   // there is only one mode
}

// ============================================================================
// POT READING
// ============================================================================

uint16_t read_pot_oversampled(uint8_t pin) {
 // DUMMY READ AFTER A CHANNEL SWITCH - kept deliberately.
 // The reason it appeared on the AVR (one sample capacitor for all inputs,
 // the charge from the previous channel pulls the first measurement towards
 // the neighbour) is different in mechanism on the ESP32, but the result is
 // the same: the first sample after a channel change is systematically
 // offset. The cost is about 55 us per knob (50 us pause + one conversion).
 analogRead(pin);
 delayMicroseconds(50);

 // TRIMMED MEAN OF SIXTEEN, not a median of five. The median of five was
 // chosen for the AVR noise of ±1-2 counts at 10 bits and a ratiometric
 // reference. On the ESP32 the raw noise is ±15-25 counts (measured on the
 // reference board), and one duty step of the amplitude costs 4055/528 = 7.7
 // counts. Without this filter the peak jitter from pulse to pulse would be
 // ±2.6 steps (±1.3 V on a 325 V bus) - WORSE than on the AVR, and the whole
 // gain from 12 bits would go into noise.
 //
 // We discard four samples from each end (the median of five kept two) and
 // average the remaining eight: white noise is suppressed by the square root
 // of eight, i.e. 2.8 times; on the reference board the spread falls to
 // ±4 counts = ±0.5 duty step.
 //
 // This also keeps the pot_moved tolerances (peak 3, bps 1) valid without
 // editing. Without the filter the display would start redrawing from knobs
 // nobody touched - exactly the flicker the median used to remove.
 //
 // WHAT THIS FILTER CANNOT DO: suppress a COMMON-MODE shift. Sixteen samples
 // are taken in a row within ~900 us, and the source impedance of the knob
 // together with the wiring / ADC input capacitance (or any RC you add at the
 // input) stretches the noise hit from the coil into a tail that moves ALL
 // sixteen in one direction. The square root of eight is about white noise,
 // not about EMI. The second layer against this is the amplitude rise-rate
 // limiter in update_pots.
 uint16_t s[16];
 for (uint8_t i = 0; i < 16; i++) s[i] = (uint16_t)analogRead(pin);

 for (uint8_t i = 1; i < 16; i++) {         // insertion sort
 uint16_t v = s[i];
 int8_t j = (int8_t)i - 1;
 while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
 s[j + 1] = v;
 }
 uint32_t acc = 0;
 for (uint8_t i = 4; i < 12; i++) acc += s[i];
 return (uint16_t)((acc + 4) >> 3);
}

// Reads the 7 potentiometers, averages them and converts them into working
// parameters. The T/U knobs are coupled: with an early knee point tscale is
// limited from below so that the ramp does not go into a plateau.
// The E-STOP button is polled BETWEEN the ADC measurements. Seven knobs,
// sixteen samples plus a dummy read each - about 6 ms in a row during which
// E-STOP would otherwise not be read at all, and a press shorter than that in
// the pause between pulses would be lost completely. With the guard between
// knobs the blind window is under a millisecond.
// The estop_ready gate is MANDATORY here, and this is why. Right after the
// fault is cleared the finger is still on the button, and estop_ready = false
// until it is released. Without the gate the same loop() pass would reach
// update_pots, see the low level and latch the fault again - the same hole as
// anywhere else the raw level is read without the gate. Arming with the button held is still
// impossible: handle_buttons has "if (estop_dn) return;", and fire_pulse
// checks the input.
#define POTS_ESTOP_GUARD() do { \
 if (estop_ready && ESTOP_DOWN()) { emergency_stop(); return; } \
} while (0)

// Hysteresis on the raw counts. The value is recomputed only when the knob has
// really moved by more than the threshold; noise within the threshold does not
// get through at all.
//
// WHY. The scale of the shape knobs is coarse: the fall has only 50 divisions
// over 4055 counts, i.e. one step is 81 counts - and at the same time a WHOLE
// PERCENT of the pulse length. A knob standing on the boundary between two
// values jumps between them from ADC noise, and the ramp knee point wanders by
// 250 us on a 25 ms pulse. Every shot comes out with its own shape.
//
// This did not happen on the AVR: there the noise was ±1-2 counts at ten bits
// with a reference equal to the supply. On the ESP32 the noise is several
// times larger and the reference is internal.
//
// The threshold is taken at about half the output step: then the jitter is
// suppressed completely, and the full knob travel loses not a single value.
// The thresholds in update_pots are sized for the ranges in this file; the
// wick range depends on BUS_VOLTAGE / WICK_VOLTAGE - re-check them if you
// change the ranges (threshold must stay below one output step in counts).
static inline uint16_t pot_hyst(uint16_t raw, uint16_t &last_raw, uint16_t &out,
                                int32_t omin, int32_t omax, uint16_t hyst) {
 uint16_t d = (raw > last_raw) ? (uint16_t)(raw - last_raw) : (uint16_t)(last_raw - raw);
 // End stops are accepted without hysteresis: between POT_RAW_HI and the ADC
 // ceiling (4095) there are only 20 counts, and with a threshold of 100 the
 // extreme value was unreachable.
 if (d > hyst || raw >= POT_RAW_HI || raw <= POT_RAW_LO) {
 last_raw = raw;
 out = (uint16_t)constrain(map(raw, POT_RAW_LO, POT_RAW_HI, omin, omax), omin, omax);
 }
 return out;
}

void update_pots() {
 uint16_t pw_raw = read_pot_oversampled(PIN_POT_PW);         POTS_ESTOP_GUARD();
 uint16_t bps_raw = read_pot_oversampled(PIN_POT_BPS);       POTS_ESTOP_GUARD();
 uint16_t ampl_raw = read_pot_oversampled(PIN_POT_AMPL);     POTS_ESTOP_GUARD();
 uint16_t wick_raw = read_pot_oversampled(PIN_POT_WICK);     POTS_ESTOP_GUARD();
 uint16_t fall_raw = read_pot_oversampled(PIN_POT_AMPSIN);   POTS_ESTOP_GUARD();
 uint16_t tscale_raw = read_pot_oversampled(PIN_POT_TSCALE); POTS_ESTOP_GUARD();
 uint16_t uscale_raw = read_pot_oversampled(PIN_POT_USCALE); POTS_ESTOP_GUARD();

 // lr = 0xFFFF, not 0: the first poll must be computed from the knob at any
 // count. With zero, a knob at the lower end stop (the ADC gives 0, the ESP32
 // has a dead zone at the bottom) did not pass the hysteresis threshold, and
 // until the first knob movement the default value lived on: AMPL 320/528,
 // PW 10 ms, BPS 2.0.
 { static uint16_t lr = 0xFFFF, ov = 10;
 pulse_width_ms = pot_hyst(pw_raw, lr, ov, PULSE_WIDTH_MIN_MS, PULSE_WIDTH_MAX_MS, 100); }
 { static uint16_t lr = 0xFFFF, ov = 20;
 bps_x10 = pot_hyst(bps_raw, lr, ov, BPS_MIN_X10, BPS_MAX_X10, 15); }
 // ★ AMPLITUDE RISE-RATE LIMITER ★
 //
 // The trimmed mean inside read_pot_oversampled suppresses white noise, but
 // not a common-mode shift of all sixteen samples at once - and pickup from
 // the coil, stretched by the input RC, looks exactly like that.
 //
 // WHY A RATE LIMIT AND NOT A CONFIRMATION. The scheme "apply on the second
 // consecutive big jump" breaks in two ways at once:
 //  * TWO DIFFERENT spikes of the same sign in a row it passes as truth, and
 //    the amplitude can go to the ceiling from a third of the scale - i.e.
 //    threefold;
 //  * it is symmetric, while the threat is one-sided. Holding back a DECREASE
 //    is not allowed at all: it is the only knob with which the operator cuts
 //    the power without touching E-STOP, and a shot at the old amplitude goes
 //    entirely into risk.
 //
 // Here: downwards - immediately and unconditionally. Upwards - no faster than
 // a tenth of the scale per pass (50 ms). The properties that follow:
 //  * a spike ONE PASS LONG raises the power by no more than one step:
 //    27 V on a 325 V bus, and at the ramp top with alpha compensation ×1.21 -
 //    up to 32 V; it is removed on the very next pass, because down is
 //    immediate.
 //    AN HONEST CAVEAT: against a spike N passes long the limiter gives
 //    min(N × step, spike size), i.e. for N >= 11 it gives nothing. The whole
 //    protection rests on the measurements being independent of each other -
 //    and that is true only as long as the poll is not captured by the firing
 //    phase, see the poll-period dithering in loop();
 //  * it cannot get stuck: there is no "waiting for confirmation" state;
 //  * there is no behaviour discontinuity at the threshold - the function is
 //    continuous;
 //  * the full knob travel from bottom to top is 11 passes: 0.55 s in manual
 //    mode and about 0.73 s in AUTO at 15 BPS, where a pass is stretched to
 //    the quiet zone.
 {
 static uint16_t lr = 0xFFFF, ov = MAX_DUTY / 2;
 uint16_t want = pot_hyst(ampl_raw, lr, ov, 0, AMPL_POT_MAX, 4);
 peak_duty_raw = want;
 if (want <= peak_duty) {
 peak_duty = want;                                    // down - immediately
 } else {
 uint16_t step = (uint16_t)(want - peak_duty);
 if (step > PEAK_RISE_MAX_PER_PASS) step = PEAK_RISE_MAX_PER_PASS;
 peak_duty = (uint16_t)(peak_duty + step);
 }
 }
 { static uint16_t lr = 0xFFFF, ov = WICK_MIN_DUTY;
 wick_offset = pot_hyst(wick_raw, lr, ov, WICK_MIN_DUTY, WICK_MAX_DUTY, 18); }
 { static uint16_t lr = 0xFFFF, ov = 12;
 fall_x100 = pot_hyst(fall_raw, lr, ov, FALL_MIN_X100, FALL_MAX_X100, 45); }
 { static uint16_t lr = 0xFFFF, ov = 100;
 tscale_x100 = pot_hyst(tscale_raw, lr, ov, TSCALE_MIN_X100, TSCALE_MAX_X100, 12); }
 { static uint16_t lr = 0xFFFF, ov = 50;
 uscale_x100 = pot_hyst(uscale_raw, lr, ov, USCALE_MIN_X100, USCALE_MAX_X100, 28); }

 // Coupled limit: when the U knob (uscale) puts the knee in the first ~40% of
 // the ramp, the T knob (tscale) is limited from below so that the start of
 // the ramp does not go into a plateau.
 if (uscale_x100 < USCALE_LOW_THRESH_X100) {
 int32_t tscale_floor = 100 - 2 * (int32_t)uscale_x100;
 if ((int32_t)tscale_x100 < tscale_floor) tscale_x100 = (uint16_t)tscale_floor;
 }

 // --- PRECOMPUTED RECIPROCALS (see the comment at the declaration) ---
 // Computed HERE because the divisors depend only on the knobs. During a pulse
 // update_pots is not called -> the values are frozen and consistent.
 uint32_t pu = (uint32_t)pulse_width_ms * 1000UL;
 if (pu < 1) pu = 1;
 inv_pulse_q16 = (655360000UL + pu / 2) / pu;

 int32_t rise_end = 10000L - (int32_t)fall_x100 * 100L;
 if (rise_end < 1) rise_end = 1;
 inv_rise_q16 = (655360000UL + (uint32_t)rise_end / 2) / (uint32_t)rise_end;

 int32_t fall_tot = 10000L - rise_end;
 if (fall_tot < 1) fall_tot = 1;
 inv_fall_q16 = (655360000UL + (uint32_t)fall_tot / 2) / (uint32_t)fall_tot;

 int32_t bpx = ((int32_t)uscale_x100 * 10000L) / 100L;
 if (bpx < 100) bpx = 100;
 if (bpx > 9900) bpx = 9900;
 bp_x_cached = bpx;
 inv_bpx_q30 = (1UL << 30) / (uint32_t)bpx;
 int32_t s2t = 10000L - bpx;
 if (s2t < 1) s2t = 1;
 inv_seg2_q30 = (1UL << 30) / (uint32_t)s2t;

 preview_invalidate();   // <- display hook: restart the preview on a shape change
}

// ============================================================================
// BUTTONS
// ============================================================================

bool check_button_press(uint8_t pin, ButtonState& state) {
 bool current_state = digitalRead(pin);
 uint32_t now = millis();

 if (current_state != state.last_state) {
 state.last_change_ms = now;
 state.last_state = current_state;
 }

 bool press_detected = false;
 if ((now - state.last_change_ms) >= DEBOUNCE_MS) {
 if (current_state != state.stable_state) {
 bool was_high = state.stable_state;
 state.stable_state = current_state;
 if (was_high && current_state == LOW) {
 press_detected = true;
 }
 }
 }

 return press_detected;
}

// Handles the button presses with debouncing: FIRE (pulse/arm),
// AUTO (start a series), E-STOP (emergency stop).
void handle_buttons() {
 const uint32_t now       = millis();
 const bool     estop_dn  = ESTOP_DOWN();

 // ── Level-stability accumulators on the E-STOP input ─────────────────────
 // ONE common block for both states, and it runs ALWAYS. If the release count
 // lived only inside the "no fault" branch, then a fault latched by something
 // other than handle_buttons (a pulse, busy_wait_us, a frame transmission)
 // would leave the timestamp a minute old; the 50 ms would "expire" instantly,
 // and contact bounce would set the fault again.
 if (estop_dn) {
 estop_high_since = 0;
 if (estop_low_since == 0) estop_low_since = now;
 } else {
 estop_low_since = 0;
 if (estop_high_since == 0) estop_high_since = now;
 }
 const bool held_low  = estop_low_since  && (now - estop_low_since)  >= DEBOUNCE_MS;
 const bool held_high = estop_high_since && (now - estop_high_since) >= DEBOUNCE_MS;

 // ── FAULT SET - IMMEDIATE ─────────────────────────────────────────────────
 // On the very first low level, without debouncing. A debounced set (through
 // check_button_press with DEBOUNCE_MS = 50) would lose a press shorter than
 // 50 ms completely: inside a pulse the fault would work (raw level read
 // there), but in the pause between pulses it would not. Debouncing is only on
 // the RE-ARM.
 if (!estop_triggered) {
 if (held_high) estop_ready = true;
 if (estop_dn && estop_ready) {
 emergency_stop();      // it also clears all flags and both accumulators
 Serial.println(F(">> ESTOP <<"));
 return;
 }
 }

 if (estop_triggered) {
 // The FIRE and AUTO buttons are still POLLED during a fault, the result is
 // discarded. Otherwise their state freezes at the pre-fault value, and right
 // after clearing check_button_press reports a false press - the system left
 // the fault already armed, and with two buttons held it managed to fire.
 (void)check_button_press(PIN_BTN_FIRE, btn_fire);
 (void)check_button_press(PIN_BTN_AUTO, btn_auto);

 if (held_high) estop_released = true;

 // CLEARING - on a STABLE press, not on a single sample. With a single raw
 // read here a level dip of a couple of microseconds from coil noise would
 // clear the fault without any press. A false fault is safe, a false clear is
 // not, hence exactly this asymmetry.
 if (estop_released && held_low) {
 if (btn_fire.stable_state == HIGH && btn_auto.stable_state == HIGH) {
 estop_triggered = false;
 estop_released  = false;
 estop_ready     = false;   // the button is still pressed: a new set only after release
 estop_blocked_reported = false;
 // Arming is deliberately NOT restored: leaving a fault always ends in SAFE.
 fire_armed = false;
 auto_mode  = false;
 manual_pulse_pending = false;
 last_pulse_ms = now;       // so that AUTO does not fire immediately after re-arming
 Serial.println(F(">> ESTOP cleared (second press) - SAFE, arming removed <<"));
 } else if (!estop_blocked_reported) {
 // Safety over availability: with the start button held we do not clear the
 // fault. But we must not stay silent - otherwise a stuck button looks like
 // a dead board.
 estop_blocked_reported = true;
 Serial.println(F(">> ESTOP: reset blocked, release S1/S2 <<"));
 }
 }

 // While the fault is latched, FIRE and AUTO are not processed further.
 return;
 }

 // The buttons are polled BEFORE any prohibitions, the result is held back.
 //
 // If the poll were placed AFTER the exit on estop_dn, then while the finger
 // rests on the E-STOP button after clearing, the btn_fire/btn_auto structures
 // freeze: their last_change_ms stays pre-fault. On the very first pass after
 // release the debouncer decides the 50 ms expired long ago and reports a
 // press IMMEDIATELY - the board arms, and with S2 held it also fires, without
 // a deliberate press.
 const bool fire_pressed = check_button_press(PIN_BTN_FIRE, btn_fire);
 const bool auto_pressed = check_button_press(PIN_BTN_AUTO, btn_auto);

 // Arming is forbidden both with the E-STOP button held and with the button
 // NOT RE-ARMED. The second condition matters: if the E-STOP contact bounces
 // or is broken, estop_ready never rises, both non-pulse latch paths are dead
 // - and in that state firing must not be allowed. A faulty button must block
 // operation, not the protection.
 if (estop_dn || !estop_ready) return;

 if (fire_pressed) {
 if (!fire_armed) {
 fire_armed = true;
 armed_since_ms = millis();
 Serial.println(F(">> ARMED (press S1 for single pulse, S2 for AUTO) <<"));
 } else if (auto_mode) {
 Serial.println(F(">> S1 ignored in AUTO (press S2 to stop AUTO) <<"));
 } else {
 manual_pulse_pending = true;
 Serial.println(F(">> FIRE (single pulse: ramp + meander) <<"));
 }
 }

 if (auto_pressed) {
 if (fire_armed) {
 auto_mode = !auto_mode;
 if (!auto_mode) {
 outputs_off();          // NOT a fault: just drop the outputs, arming stays
 Serial.println(F(">> AUTO=OFF (still ARMED) <<"));
 } else {
 Serial.println(F(">> AUTO=ON (firing) <<"));
 }
 } else {
 Serial.println(F(">> AUTO ignored (not ARMED). Press FIRE first <<"));
 }
 }
}

// Drops the outputs WITHOUT latching a fault: ramp (GPIO19) -> 0 %,
// interrupter (GPIO4) -> LOW.
// Does NOT touch the flags (fire_armed / auto_mode / estop_triggered).
void outputs_off() {
 if (!g_mc_regs_ok) return;      // the peripheral is not up - it must not be touched
 // The order matters: FIRST remove the interrupt's permission to drive the
 // envelope, and only then shut down. In the reverse order an interrupt
 // arriving between the two lines would write the next ramp point and undo
 // the shutdown - and there would be nobody left to restore the force,
 // because the flag is already cleared by then.
 ramp_active = false;
 ramp_kill_ll();          // ramp by level + comparator to safe, bridge to zero
}

// Emergency stop (LATCH). Drops the outputs through outputs_off() AND LATCHES
// the fault. The only way out is a deliberate reset: release the button and
// press it a SECOND time (see handle_buttons). There is no timed hold.
// The SOLE owner of the transition into the fault. All flags and both level
// accumulators are cleared here, so it does not matter where the latch came
// from: the button, a pulse, busy_wait_us or a frame transmission - the state
// is always consistent. No other function edits these flags on the way in:
// otherwise a latch from a pulse would leave stale timers behind.
void emergency_stop() {
 outputs_off();
 estop_ready            = false;
 estop_released         = false;
 estop_high_since       = 0;
 estop_low_since        = 0;
 estop_blocked_reported = false;
 estop_triggered      = true;        // <- LATCH: mark the fault
 g_need_recover       = true;        // the hardware brake is cleared separately
 fire_armed           = false;       // remove arming
 auto_mode            = false;       // switch auto mode off
 manual_pulse_pending = false;       // cancel a pending pulse
}

// ============================================================================
// CURVE TRANSFORMATION
// ============================================================================

// Computes the bend of the ramp curve - how far the power deviates from a
// straight line. This is what bends the ramp according to the T/U knobs. The
// result is clamped so that the ramp stays monotonic.
int32_t IRAM_ATTR calc_curve_deviation(int32_t rise_t, int32_t ramp_range) {
 if (tscale_x100 == 100) return 0;
 // bp_x and its reciprocals are precomputed in update_pots (no division here)
 int32_t bp_x = bp_x_cached;
 int32_t curve_factor = (int32_t)tscale_x100 - 100;
 int32_t bp_y_shift = (ramp_range * curve_factor) / 200L;
 int32_t chord_at_bp = (bp_x * ramp_range) / 10000L;
 int32_t bp_y = chord_at_bp + bp_y_shift;
 // Clamp the knee point to [0, ramp_range] => both segments have a
 // non-negative slope => the ramp is monotonic for ANY T/U knob position.
 if (bp_y < 0) bp_y = 0;
 if (bp_y > ramp_range) bp_y = ramp_range;
 int32_t actual_y;
 if (rise_t < bp_x) {
 // was: (rise_t * bp_y) / bp_x - a division by a variable divisor.
 uint32_t frac = ((uint32_t)rise_t * inv_bpx_q30) >> 14;   // 0..65536
 actual_y = ((int32_t)frac * bp_y) >> 16;
 } else {
 int32_t seg2_t = rise_t - bp_x;
 int32_t seg2_dy = ramp_range - bp_y;
 uint32_t frac2 = ((uint32_t)seg2_t * inv_seg2_q30) >> 14; // 0..65536
 actual_y = bp_y + (((int32_t)frac2 * seg2_dy) >> 16);
 }
 int32_t chord_y = (rise_t * ramp_range) / 10000L;
 return actual_y - chord_y;
}

// ============================================================================
// DUTY CALCULATION
// ============================================================================

// The MAIN ramp shape function. From the elapsed time inside the pulse it
// returns the required PWM duty (0..PWM_TOP) - the instantaneous power.
// Accounts for the wick start, the rise, the curve bend and the fall.
// v5.9: NOT CHANGED. The mode parameter and its switch are kept on purpose -
// only MODE_QCW is reachable, the other branches are unreachable.
int32_t IRAM_ATTR calculate_duty(uint32_t t_elapsed_us, uint32_t pulse_us, uint8_t mode) {
 if (t_elapsed_us >= pulse_us) return 0;
 // was: (t_elapsed_us * 10000UL) / pulse_us - a division by a variable
 // divisor (~200 cycles on the AVR this was ported from).
 uint32_t t_norm = (t_elapsed_us * inv_pulse_q16) >> 16;
 if (t_norm > 10000UL) t_norm = 10000UL;   // guard against rounding up
 int32_t duty = 0;

 switch (mode) {
 case MODE_QCW: {
 const int32_t RISE_END_X10000 = 10000 - ((int32_t)fall_x100 * 100);
 int32_t effective_peak = (int32_t)peak_duty;
 // Clamp wick to peak. Without this, with wick > peak the rise sits flat on
 // wick while the fall starts from peak - a step down in the middle of the pulse.
 int32_t effective_wick = (int32_t)clamped_wick(effective_peak);
 int32_t ramp_range = effective_peak - effective_wick;   // always >= 0 after the clamp

 if ((int32_t)t_norm < RISE_END_X10000) {
 int32_t rise_t = (int32_t)(((uint32_t)t_norm * inv_rise_q16) >> 16);
 if (rise_t > 10000L) rise_t = 10000L;
 int32_t chord = effective_wick + (rise_t * ramp_range) / 10000;
 int32_t deviation = calc_curve_deviation(rise_t, ramp_range);
 int32_t duty_target = chord + deviation;
 int32_t comp_step1 = (duty_target * rise_t) / 10000L;
 int32_t duty_comp = (comp_step1 * (int32_t)alpha_x1000) / 1000L;
 duty = duty_target + duty_comp;
 } else {
 int32_t fall_position = (int32_t)t_norm - RISE_END_X10000;
 int32_t fall_t = (int32_t)(((uint32_t)fall_position * inv_fall_q16) >> 16);
 if (fall_t > 10000L) fall_t = 10000L;
 int32_t peak_at_end_rise = effective_peak;
 int32_t comp_at_peak = (peak_at_end_rise * (int32_t)alpha_x1000) / 1000L;
 int32_t peak_total = peak_at_end_rise + comp_at_peak;
 if (peak_total > MAX_DUTY) peak_total = MAX_DUTY;
 duty = peak_total - (peak_total * fall_t) / 10000L;
 }
 break;
 }
 case MODE_INTERRUPTED: duty = 0; break;   // unreachable since v5.9
 case MODE_CW_QCW:      duty = 0; break;   // unreachable since v5.9
 }
 if (duty < 0) duty = 0;
 if (duty > MAX_DUTY) duty = MAX_DUTY;
 return duty;
}

// ============================================================================
// FIRE PULSE
// ============================================================================

// Performs ONE ramp pulse: buck warm-up on wick, interrupter (GPIO4) on, the
// ramp pass by calculate_duty output on GPIO19, then the fall and switch-off.
// The ramp shift (buck timing) is implemented here as well.
void fire_pulse() {
 // Check AT THE ENTRY. Without it a path existed: right after clearing the
 // fault the button is still held, estop_ready = false, arming passes - and
 // the pulse started with a finger on E-STOP. The bridge was not enabled only
 // thanks to ENABLE_BUCK_TIMING, i.e. a setting saved the day, not the logic.
 if (ESTOP_DOWN()) { emergency_stop(); return; }
 // Hardware gate. Without it a partial MCPWM initialisation failure looked
 // like this: the firmware printed FATAL and "forbade operation" with the same
 // flags that the ordinary fault-reset gesture clears - and then the very
 // first FIRE set the bridge enable and went to wait for a notification that
 // would never come, because the carrier callback is not called. The bridge
 // stayed enabled for the whole timeout.
 // g_isr_core < 0 means the carrier callback has NEVER been called. Without
 // this half of the gate that case would pass straight through, and only the
 // watchdog timeout would catch it - by which time the bridge is already
 // enabled and the buck runs at a constant wick duty for the whole pulse
 // length.
 if (!g_mcpwm_ok || g_isr_core < 0) {
 emergency_stop();
 // Staying silent here is not acceptable: the operator would clear the FATAL
 // with the normal gesture, arm, press FIRE - and get the arming removed
 // without a single line on the port.
 static uint32_t last_say = 0;
 if (millis() - last_say > 1000) {
 last_say = millis();
 Serial.println(F(">> FIRING FORBIDDEN: hardware is not in a working state <<"));
 }
 return;
 }

 uint32_t pulse_us = (uint32_t)pulse_width_ms * 1000UL;
 int32_t initial_duty = calculate_duty(0, pulse_us, MODE_QCW);

 g_pulse_us       = pulse_us;
 rs_tick          = 0;
 g_cmp_min        = 0xFFFF;
 g_cmp_max        = 0;
 g_isr_since_task = 0;

#if ENABLE_BUCK_TIMING
 // === RAMP SHIFT ===
 // 1) ramp to the CLAMPED wick baseline (min(wick, peak)), bridge still
 //    disabled - LC filter settles. baseline == calculate_duty(0), the shape
 //    is continuous. On E-STOP busy_wait_us itself drops the outputs and
 //    latches the fault.
 set_buck_duty(clamped_wick((int32_t)peak_duty));
 if (!busy_wait_us(BUCK_WARMUP_US)) return;
 // 2) Bridge enable, then a short warm-up before the rise starts.
 bridge_force_ll(1);
 if (!busy_wait_us(BUCK_ENABLE_DELAY_US)) return;
#else
 set_buck_duty((uint16_t)initial_duty);
 bridge_force_ll(1);
#endif

 // FROM HERE the envelope is driven by the interrupt, one update per carrier period.
 //
 // THE AVR INVARIANT IS PRESERVED STRUCTURALLY, and this is the main property
 // of the whole port. calculate_duty takes no shape parameters - it reads the
 // globals peak_duty, wick_offset, fall_x100, tscale_x100 and the six
 // precomputed reciprocals. On the AVR they were consistent because fire_pulse
 // is blocking and update_pots was not called during a pulse. Here it is
 // exactly the same: the task sits in ulTaskNotifyTake and physically cannot
 // execute update_pots. That is why calculate_duty was ported UNTOUCHED - no
 // parameter snapshot, no mutexes, no parameterisation of the shape.
 // SPLITTING THE KNOB POLL ONTO THE SECOND CORE IS NOT ALLOWED: the result
 // would be an envelope frame computed from half the old and half the new set.
 ulTaskNotifyTake(pdTRUE, 0);        // clear the tail from the previous pulse
 g_isr_since_task = 0;
 // The reset above may have eaten a notification from estop_isr/on_ost that
 // arrived in the last microseconds of the warm-up. The interrupt flags live
 // until estop_sync_from_isr, so checking them here catches any event that
 // happened before this line. Without it the ramp would start with the bridge
 // already shut down by the interrupt, and the buck bank would charge with no
 // load.
 if (ESTOP_DOWN() || g_estop_req || g_brake_ost || g_stall_kill) {
 emergency_stop();
 Serial.println(F(">> ESTOP before ramp start - SAFE <<"));
 return;
 }
 // The ramp is ARMED here, after the warm-up and the bridge enable - exactly
 // where start_us = micros() stood on the AVR; the cycle-counter reference
 // itself (g_ramp_t0_cyc) is taken by on_tez on the first carrier period after
 // arming (see there). The shape is continuous because of this.
 last_pulse_start_ms = millis();     // mark for choosing the knob-poll moment
 ramp_active = true;
 // A second time AFTER arming: the interrupt may have fired between the check
 // above and the write of ramp_active. The window is a couple of instructions,
 // but closing it is cheap.
 if (ESTOP_DOWN() || g_estop_req || g_brake_ost || g_stall_kill) {
 ramp_active = false;
 ramp_kill_ll();
 emergency_stop();
 Serial.println(F(">> ESTOP at ramp arming - SAFE <<"));
 return;
 }

 // The forward watchdog. The reverse one (g_isr_since_task) catches a dead
 // task with a live interrupt; here the MIRROR and more dangerous case is
 // caught - the interrupt stopped being called in the middle of the ramp. Then
 // the comparator HOLDS the last value (near the peak that is almost full
 // duty), the bridge stays enabled, and no other watchdog sees it.
 // Margin +3 ms, not +20. The ramp end moment is deterministic to within one
 // carrier period (40 us); a 20 ms margin is 500 periods where two suffice,
 // and all of it would turn into time during which the bridge is enabled and
 // the comparator holds the peak value - at PW=5 ms four pulse lengths of it.
 if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS((uint32_t)pulse_width_ms + 3)) == 0) {
 ramp_active = false;
 ramp_kill_ll();
 emergency_stop();
 Serial.println(F(">> ramp ISR did not respond - SAFE <<"));
 return;
 }
 if (g_estop_req || g_brake_ost || g_stall_kill) {
 // The bridge is removed RIGHT HERE. Relying on the flag sync to get to it is
 // not acceptable: between this return and the estop_sync_from_isr call in
 // the next loop pass lies a whole pass.
 bridge_force_ll(0);
 // There was no discharge window: if the ramp managed to run with the bridge
 // off, the buck output bank (Cb) may have stayed charged. Discharge it
 // before the next pulse.
 Serial.println(F(">> Fault inside a pulse: buck Cb may have stayed charged - discharge it before the next shot <<"));
 estop_sync_from_isr();
 return;
 }
 // SPURIOUS WAKE-UP. The notification is taken as the sign of the ramp end,
 // but the source of the notification may turn out to be anything else. If
 // the task goes to the discharge while the interrupt keeps driving the ramp
 // upwards, the bridge is removed in the middle of the rise - and the buck
 // keeps charging the bank to the peak with the bridge DISABLED. That is
 // exactly the scenario this file calls certain death for the IGBTs at
 // BUCK_DISCHARGE_US.
 if (ramp_active) {
 ramp_active = false;      // FIRST forbid, then shut down: otherwise the interrupt
 ramp_kill_ll();           // manages to undo the shutdown by writing the next point
 emergency_stop();
 Serial.println(F(">> SAFE: woken up before the ramp end <<"));
 return;
 }
 if (estop_triggered) return;

#if ENABLE_BUCK_TIMING
 // 4) Normal completion: the bridge is kept enabled while the buck cap
 //    discharges through the antiparallel diodes. The return IS CHECKED:
 //    otherwise a fault pressed exactly in these 2 ms would drop the outputs,
 //    but the pulse would still be counted as normal - the counter would grow,
 //    and the display would add a shot that never happened.
 if (!busy_wait_us(BUCK_DISCHARGE_US)) return;
#endif

 bridge_force_ll(0);
 pulse_counter++;

 if ((pulse_counter & 0x07) == 0) {
 Serial.print(F("QCW #")); Serial.print(pulse_counter);
 Serial.print(F(" duty=")); Serial.print(g_cmp_min);
 Serial.print(F("..")); Serial.print(g_cmp_max);
 Serial.print(F(" ticks=")); Serial.print(rs_tick);
 Serial.print(F(" init=")); Serial.print(initial_duty);
 Serial.print(F(" peak=")); Serial.print(peak_duty);
 Serial.println();
 }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

// Main loop: updates the knobs, handles the buttons, feeds the watchdog,
// services the display and, if armed/auto - fires pulses at the required
// rate (BPS).
void loop() {
 wdt_kick();
 g_isr_since_task = 0;          // "task alive" - the reverse watchdog in on_tez
 uint32_t now_ms = millis();

 // A fault seen by an interrupt or by the hardware brake is carried into
 // software BEFORE the buttons are handled: otherwise the state machine in
 // handle_buttons would run without knowing that the outputs are already dead.
 estop_sync_from_isr();

 handle_buttons();

 // Clearing the hardware latch comes AFTER handle_buttons and as a separate
 // step: the fault is cleared on a STABLE PRESS, i.e. at that moment the
 // button is still held down and the hardware will not allow recovery. Here
 // it waits for the release.
 // The hardware recovery waits for estop_ready, i.e. 50 ms of STABLE release.
 // Without this condition the latch would be cleared on the very first pass
 // with a high level - i.e. in the middle of the release bounce - and the next
 // bounce down would give a new brake. A short bounce is indistinguishable
 // from a glitch (the level recovers before the handler is entered), it would
 // pass the gate as a glitch and latch the fault again milliseconds after
 // clearing. The very same "press it several times", back through a new door.
 if (!estop_triggered && estop_ready) estop_recover_hw();

 // ★ WHEN TO POLL THE KNOBS ★
 //
 // A timer of its own at 50 ms is not enough. The blocking shot and this timer
 // form a loop: in AUTO mode the ADC poll locks firmly into the phase right
 // after the bridge enable is removed and stays there for as long as you like.
 // Then a disturbance tied to the shot - the ringing of the buck LC filter,
 // the current through the panel ground - is present on EVERY measurement, not
 // on a random one, and stops being noise: it becomes a systematic offset that
 // no filter removes. This lives in the upper third of the BPS scale, i.e. at
 // maximum power and maximum EMI.
 //
 // So in AUTO the poll is forbidden until the pulse has COMPLETELY ended
 // (warm-up 2 ms + enable 1 ms + ramp + discharge 2 ms) plus another 7 ms for
 // decay. The upper fuse at 500 ms is there so that no combination of BPS and
 // length leaves the knobs unpolled altogether.
 // THE WINDOW EDGE IS DITHERED, NOT THE POLL PERIOD - and this is essential.
 //
 // Dithering the poll PERIOD instead (say 50...71 ms) is a false fix: the
 // average period of 60.5 ms is shorter than the 66 ms firing cycle at 15 BPS,
 // so the measurement moment is set not by the period but by the edge of the
 // quiet zone - the poll runs into it and lands at the same phase (on the
 // reference build: T+40 ms six times out of eight). The phase lock stays, and
 // the amplitude rise-rate limiter, which relies on independent samples, would
 // be resting on a non-existent property.
 //
 // The poll lands on the edge - so the edge dithering carries over to the
 // measurement moment one to one. The steady state at 15 BPS and a 25 ms ramp
 // (measured on the reference build) is eight distinct phases with an 18 ms
 // spread (the nominal 21 ms dithering spread is clipped from below by the
 // "not more often than 50 ms" condition), whereas the free window between
 // the pulse end and the arming of the next one is 36 ms.
 // The display tick runs BEFORE the knob poll: a set that did not make it in
 // the previous cycle goes out as soon as the window opens, not after ~6 ms of
 // ADC polling. Otherwise at high BPS the send would regularly run into the
 // margin before the pulse.
 // Display hook. Placed BEFORE the arming checks on purpose (see above):
 // fire_pulse() is blocking (the task either spins in busy_wait_us or sits in
 // ulTaskNotifyTake), so loop() physically does not run during a pulse, and
 // the preview computation and the frame transmission fall only into the gaps
 // between shots.
 qcw_display_tick();

 static uint8_t pot_seq = 0;
 bool pots_due = (now_ms - last_pot_update_ms) >= 50;
 if (pots_due && auto_mode && fire_armed && !estop_triggered
     && (now_ms - last_pot_update_ms) < 500) {
 // The dithering is taken from the SUM of the poll counter and the shot
 // counter. From pot_seq alone it degenerates: when the number of polls per
 // firing cycle is a multiple of eight (e.g. 2.5 and 0.5 BPS), pot_seq & 7
 // arrives at the first poll after a pulse always the same, and the edge
 // locks firmly. pulse_counter grows by one per cycle and breaks this
 // commensurability.
 const uint32_t quiet_after_ms = (uint32_t)pulse_width_ms + 12
                               + (((uint32_t)pot_seq + pulse_counter) & 7UL) * 3UL;
 if ((now_ms - last_pulse_start_ms) < quiet_after_ms) pots_due = false;
 }

 if (pots_due) {
 pot_seq++;
 static uint32_t pots_us = 0, pots_us_max = 0;     // diagnostics: poll duration
 const uint32_t t_pots0 = esp_cpu_get_cycle_count();
 update_pots();
 pots_us = (esp_cpu_get_cycle_count() - t_pots0) / CPU_MHZ;
 if (pots_us > pots_us_max) pots_us_max = pots_us;
 last_pot_update_ms = now_ms;

 static uint8_t diag_cnt = 0;
 if (++diag_cnt >= 40) {
 diag_cnt = 0;
 Serial.print(F("POTS PW=")); Serial.print(pulse_width_ms);
 Serial.print(F("ms AMPL=")); Serial.print(peak_duty);
 Serial.print(F("/")); Serial.print(AMPL_POT_MAX);
 Serial.print(F(" BPS=")); Serial.print(bps_x10 / 10);
 Serial.print(F(".")); Serial.print(bps_x10 % 10);
 Serial.print(F("Hz WICK=")); Serial.print(wick_offset);
 Serial.print(F(" ARM=")); Serial.print(fire_armed);
 Serial.print(F(" AUTO=")); Serial.print(auto_mode);
 Serial.print(F(" pots=")); Serial.print(pots_us); Serial.print(F("us(max "));
 Serial.print(pots_us_max); Serial.print(F(") prev=")); Serial.print(g_prev_us); Serial.print(F("us"));
 Serial.println();
 }
 }


 // ★ SAFETY NET ON THE ACTUAL HARDWARE STATE ★
 //
 // We read the REGISTER, not the software flag - as the original on the AVR
 // read OCR1A. The whole point is to catch a discrepancy between what the
 // firmware thinks and what the silicon outputs; a software flag by definition
 // cannot see such a discrepancy. (The order "clear the flag, then shut down"
 // is observed in the paths from the task, but NOT in all paths from the
 // interrupts - there the shutdown comes first. It is safe there: a handler
 // does not pre-empt itself. But that is exactly why the safety net cannot be
 // built on the flag.)
 //
 // Force mode 1 = held at zero; any other value = the output is live.
 //
 // The check sits BEFORE the "not armed" return on purpose: otherwise, when
 // armed and idle without AUTO, the loop would reach the return without
 // reading the registers even once, and nobody would catch outputs that came
 // alive in that state.
 if (g_mc_regs_ok && !ramp_active) {
 const bool ramp_held = (MCPWM_HW->operators[OPER_ID].gen_force.gen_a_cntuforce_mode == 1);
 const bool brdg_held = (MCPWM_HW->operators[OPER_ID].gen_force.gen_b_cntuforce_mode == 1);
 if (!ramp_held || !brdg_held) outputs_off();   // protective safety net, NOT a fault
 }

 if (!fire_armed || estop_triggered) return;

 if (!auto_mode) {
 if (manual_pulse_pending) {
 manual_pulse_pending = false;
 fire_pulse();
 // The mark is moved on a manual shot too. On the AVR it was updated only
 // in the AUTO branch, so switching AUTO on right after a manual pulse gave
 // a second shot a few milliseconds later instead of after the BPS period -
 // an average power higher than set by the knob. A deliberate departure
 // from the reference.
 last_pulse_ms = millis();
 }
 return;
 }

 uint32_t cycle_ms = 10000UL / bps_x10;
 if ((now_ms - last_pulse_ms) >= cycle_ms) {
 fire_pulse();
 last_pulse_ms = now_ms;
 }
}
