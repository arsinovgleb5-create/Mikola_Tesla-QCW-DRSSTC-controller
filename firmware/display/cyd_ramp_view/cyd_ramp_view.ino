/*
 * ============================================================================
 *   CYD RAMP VIEW - ramp envelope display for the QCW DRSSTC GENERATOR v5.9
 *   Author: Mikola_Tesla  -  License: CC BY 4.0 (share and adapt freely, credit the author)
 *   Board: ESP32-2432S028R ("Cheap Yellow Display", ILI9341 320x240)
 * ============================================================================
 *
 *   WHAT IT IS: a stand-alone display for the QCW DRSSTC generator. It shows
 *   the ramp envelope the generator has computed (buck output voltage versus
 *   time), the generator mode (AUTO / ARMED / IDLE / E-STOP), the link state,
 *   the shot counter, the parameters the curve was computed from, and a small
 *   supply voltmeter.
 *
 *   ROLE: I2C SLAVE at address 0x2A, bus clock 250 kHz. The generator (another
 *   ESP32) is the master. The link is one-way: the display only receives and
 *   draws; it cannot influence the pulse shape in any way.
 *
 *   WIRING (reference build: control board rev. E; adjust for yours)
 *   Control board J5 (1x5) -> CYD:
 *       J5.1 GND  -> CN1.GND
 *       J5.2 SCL  -> CN1.IO22
 *       J5.3 SDA  -> CN1.IO27
 *       J5.4 VM   -> P3.IO35   (voltmeter input; in the reference build a 10k/10k
 *                               divider on the control board measures the 5 V
 *                               supply, hence VMETER_DIV = 2.0 - see VMETER_*)
 *       J5.5 +5V  -> CYD 5 V   (USB or VIN)
 *   In the reference build the I2C cable is short and the 4.7k pull-ups sit on
 *   the control board (to its 3.3 V rail); CN1.3V3 is not used. Observed with
 *   ESP32 core 3.3.x: the slave driver also enables the weak internal pull-ups
 *   (~45k); in parallel with the external 4.7k this is harmless. Ground is
 *   common with the generator.
 *
 *   Upload (reference build; adjust for yours): ESP32 Arduino core 3.3.x, board
 *   "ESP32 Dev Module", upload speed 230400 - the CH340 USB-serial on the
 *   author's CYD batch fails at 460800 and above.
 *
 *   PROTOCOL. Frame: [type][len][payload...][crc8]; crc8 is computed over
 *   type+len+payload (polynomial 0x07, init 0x00, MSB-first).
 *     0x00 STATUS  (6 bytes): fw_major, fw_minor, flags, pulses_lo, pulses_hi, 0
 *                  flags: 0x01 ARMED, 0x02 AUTO, 0x04 ESTOP, 0x08 PREVIEW_READY
 *                  Sent every 100 ms - this is the generator's sign of life.
 *     0x10|gen META (22 bytes, all uint16 little-endian except the last two;
 *                  gen is the 3-bit preview generation, see below):
 *                  [0]  pulse_width_ms   [2]  bps_x10        [4]  peak_duty
 *                  [6]  wick_offset      [8]  fall_x100      [10] tscale_x100
 *                  [12] uscale_x100      [14] pwm_top        [16] bus_voltage
 *                  [18] rise_end_x10000  [20] points (80)    [21] rise_points (64)
 *     0x40|gen<<3|n CHUNK n of the curve of generation gen: n=0 - points 0..29,
 *                  n=1 - 30..59, n=2 - 60..79.
 *                  Each point is a duty value 0..255 relative to pwm_top.
 *   The time grid is NON-UNIFORM: the first rise_points points cover the rise
 *   [0, rise_end) uniformly, the remaining ones cover the fall [rise_end, pw).
 *   The point with index rise_points sits exactly at the knee.
 *
 *   SUPPLY VOLTMETER: a resistor divider from the supply to IO35 (connector P3);
 *   the value is drawn in small font under the right edge of the left half of
 *   the header: green - normal, yellow - sagging, red - below VMETER_LOW_V.
 *   Thresholds and the divider ratio are the VMETER_* constants (reference
 *   build: 10k/10k on a 5 V supply; adjust them to your divider and supply).
 *
 *   SCREEN LAYOUT: dark blue background, blue axes with a dotted grid,
 *   yellow-green envelope; top-left the mode in large font (AUTO / ARMED /
 *   IDLE / E-STOP), top-right LINK OK and shots/crc; at the bottom peak and
 *   wick in volts, fall, BPS, pulse width, the number of PWM steps and the
 *   voltage increment per step.
 *
 *   REDRAW LOGIC: the curve is redrawn only when a COMPLETE set of chunks has
 *   arrived after META (otherwise a splice of the old and the new curve would
 *   reach the screen). Status, captions and plot are redrawn by signatures:
 *   nothing changed - nothing flickers. No STATUS for longer than
 *   LINK_TIMEOUT_MS - the link is considered lost, the last curve stays.
 *
 *   LIBRARIES: TFT_eSPI (reference build: 2.5.43) with the supplied
 *   User_Setup_CYD.h (copy it over Arduino/libraries/TFT_eSPI/User_Setup.h; it
 *   selects ILI9341_2_DRIVER, the HSPI port and TFT_INVERSION_ON - the panel on
 *   the author's CYD batch is inverted, without it the image is a negative:
 *   white background, black text; adjust for your panel), and Wire from the
 *   ESP32 Arduino core (reference build: 3.3.x).
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>

// ============================================================================
// CYD HARDWARE
// ============================================================================
#define PIN_LINK_SDA   27
#define PIN_LINK_SCL   22
#define PIN_TFT_BL     21      // backlight, active HIGH
#define PIN_LED_R      4       // on-board RGB LED, active LOW
#define PIN_LED_G      16
#define PIN_LED_B      17

// ############################################################################
// ##  BUILD CONFIGURATION - adapt these for YOUR build: voltmeter divider   ##
// ##  and thresholds, auto refresh, link speed and timeout (below).         ##
// ############################################################################
// --- Display supply voltmeter: input IO35 on connector P3 (ADC1_CH7, input only) ---
// Divider from the measured point to IO35: no more than 3.1 V on the pin.
//   5 V (CYD/USB supply):   10 kOhm + 10 kOhm      -> VMETER_DIV 2.0   (reference build)
//   12 V battery:           100 kOhm + 22 kOhm     -> VMETER_DIV 5.55 (14.4 V -> 2.6 V)
// VMETER_DIV = (R_top + R_bottom) / R_bottom. Colour thresholds are in volts at the measured point.
// Adjust VMETER_DIV, VMETER_LOW_V and VMETER_OK_V to your divider and supply.
#define PIN_VMETER        35
#define VMETER_DIV        2.0f
#define VMETER_LOW_V      4.5f     // below this - red: the supply is sagging (value for a 5 V supply)
#define VMETER_OK_V       4.8f     // above this - green; in between - yellow (value for a 5 V supply)
#define VMETER_PERIOD_MS  200
#define VMETER_SAMPLES    8
#define VMETER_MIN_SHOW_V 1.0f     // below this we assume there is no divider (pin floating) and show nothing

// --- Auto refresh: every AUTO_REFRESH_MIN minutes the display is initialised
// again, the whole screen is redrawn and the crc counter is cleared. Coil
// interference on the SPI bus can upset the ILI9341 controller (shifted window,
// inversion) - this way the picture recovers by itself. Generator data (shots,
// curve) is left untouched. Adjust for your build: the refresh redraws the
// whole screen once; make it longer if your picture never gets corrupted.
// Must stay above 0 (0 would refresh on every loop pass).
#define AUTO_REFRESH_MIN  20

// ============================================================================
// PROTOCOL (mirror of the generator - change only together with it)
// ============================================================================
#define LINK_SLAVE_ADDR   0x2A
#define LINK_FREQ_HZ      250000UL   // 250 kHz: the longest frame (33 bytes + address, 9 clocks each) is ~1.2 ms of bus
                                     // time, margin on the edges. Reference build: short cable, 4.7k pull-ups. Lower it
                                     // for a long cable - always together with the generator (its LINK_FREQ_HZ)
#define LINK_TIMEOUT_MS   2000UL   // STATUS comes every 100 ms (generator STATUS_PERIOD_MS); 20 misses = link lost.
                                   // Adjust for your build if you change the STATUS period or slow the link down
#define META_PW_MAX_MS    1000     // META plausibility: the generator gives 5..25 ms

#define FRM_STATUS        0x00
#define FRM_META          0x10     // | gen (3 bits) - preview generation
#define FRM_PREVIEW0      0x40     // | gen<<3 | idx - chunk idx of generation gen
// Generation: the generator changes it on every recomputation of the shape.
// Chunks are accepted only for the generation announced by the last META, so
// a lost META cannot mix generations. A generator that does not use
// generations (types 0x10 and 0x40..0x42) decodes as generation 0 and is
// accepted.
#define STATUS_LEN        6
#define META_LEN          22
#define CHUNK_LEN         30       // points in chunks 0 and 1 (the last one carries the remainder)
#define CURVE_MAX         128      // buffer with headroom: the generator sends 80
#define LINK_RX_MAX       (CURVE_MAX + 3)

#define FLAG_ARMED        0x01
#define FLAG_AUTO         0x02
#define FLAG_ESTOP        0x04
#define FLAG_PREVIEW      0x08

// ============================================================================
// SCREEN LAYOUT 320x240 (rotation 1):
// dark blue background, blue axes and dotted grid, yellow-green envelope
// without fill, mode in large font on the left, LINK OK and shots/crc on the
// right, two lines at the bottom: peak/wick in volts, fall, BPS; pulse width,
// number of PWM steps and voltage increment per step.
// ============================================================================
#define SCR_W       320
#define SCR_H       240
#define HDR_H       30                       // header: mode on the left, link on the right
#define HDR_SPLIT   (SCR_W - 140)            // boundary between the left and right parts of the header: on the right "shots 65535  crc 65535" = 132 px in font 1
#define PLOT_X      44                       // V-axis labels on the left
#define PLOT_Y      44
#define PLOT_W      262
#define PLOT_H      138                      // plot bottom at 181, X labels below it
#define INFO_Y      198                      // parameter lines: 198 (font 2), 218 (font 1)

#define COL_BG      0x0046                   // dark blue
#define COL_GRID    0x014D                   // dotted grid
#define COL_AXIS    0x2BDF                   // axes
#define COL_LBL     0x547F                   // axis labels
#define COL_TEXT    0xCEFF                   // first parameter line
#define COL_TXT2    0x6CBB                   // second line, shots/crc
#define COL_CURVE   TFT_GREEN                // envelope
#define CURVE_W     3.2f                     // line width, px (anti-aliased)
#define PRE_MS      1                        // the first millisecond is zero, then a sharp rise to wick
#define COL_OK      TFT_GREEN
#define COL_BAD     TFT_RED
#define COL_AUTO    TFT_ORANGE
#define COL_ARMED   TFT_YELLOW
#define COL_IDLE    0x7BEF

// ============================================================================
// STATE
// ============================================================================
static TFT_eSPI    tft;
static TFT_eSprite spr(&tft);
static bool        g_spr_ok = false;         // whether there was enough memory for the plot sprite

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// --- receive side: written by on_receive(), read by loop() under g_mux ------
// A curve generation = META + its chunks. Chunks are accepted ONLY after META
// (r_meta_valid) and only until the generation is complete: a META lost to CRC
// cannot let the chunks of a new generation mix with the old caption.
static uint8_t  r_curve[CURVE_MAX];          // curve being assembled from chunks
static uint8_t  r_meta[META_LEN];            // META of the current generation
static bool     r_meta_valid  = false;       // META received, chunks are accepted
static bool     r_curve_valid = false;       // r_curve as a whole belongs to r_meta
static bool     r_gen_changed = false;       // curve bytes changed within this generation
static uint8_t  r_gen         = 0;           // generation from the last META
static bool     r_expect_same = false;       // the last META had the same shape: chunks must
                                             // repeat r_curve byte for byte (heartbeat)
static uint8_t  g_set_done    = 0;           // bit mask of the chunks received for this generation
static bool     g_have_data   = false;       // set complete, loop() will pick it up
static uint8_t  r_flags       = 0;           // from STATUS
static uint16_t r_pulses      = 0;
static uint8_t  r_fw_major    = 0, r_fw_minor = 0;
static volatile uint16_t r_crc_err = 0;      // frames discarded for CRC/length/plausibility
static uint32_t g_last_rx_ms  = 0;           // millis() of the last good frame
static bool     g_link_ok     = false;

// --- copies for drawing: loop() only, taken under g_mux --------------------
static uint8_t  g_curve[CURVE_MAX];
static uint8_t  g_meta[META_LEN] = {0};
static int      g_cv = 0;                    // curve version: ++ on every complete set
static uint8_t  g_flags = 0;
static uint16_t g_pulses = 0;
static uint8_t  g_fw_major = 0, g_fw_minor = 0;
static uint16_t g_crc_err = 0;

// --- signatures: redraw only what has changed -------------------------------
static uint32_t g_status_sig = 0xFFFFFFFF;   // flags, link (fw is received but not shown)
static uint32_t g_pulses_sig = 0xFFFFFFFF;   // link, shots, crc (top right)
static uint32_t g_volts_sig  = 0xFFFFFFFF;   // supply voltmeter (top left)
static float    g_volts      = 0.0f;         // smoothed voltage, V
static bool     g_volts_valid = false;
static uint32_t g_info_sig   = 0xFFFFFFFF;   // axis labels and parameter lines: META
static uint32_t g_plot_sig   = 0xFFFFFFFF;   // plot: curve bytes + META without BPS
static uint32_t g_sig_changed_ms = 0;        // millis() of the last new curve set: drives the LED flash (new_curve_recent)
static bool     g_forced     = true;         // full redraw (start-up, link state change, auto refresh)
static bool     g_rx_active  = false;        // a curve set has just arrived (mirrors the LED flash; not drawn on screen)
static bool     g_link_shown = false;        // link state currently shown on screen
static bool     g_wire_ok    = false;        // slave is up (Wire.begin succeeded)
static uint32_t g_last_refresh_ms = 0;       // last display auto refresh

// ============================================================================
// PROTOCOL: CRC and unpacking
// ============================================================================
static uint8_t link_crc8(const uint8_t* p, uint8_t n) {
  uint8_t crc = 0;
  while (n--) {
    crc ^= *p++;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

static inline uint16_t get16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// META fields by offset, as in the generator's build_meta().
#define M_PW(m)       get16(&(m)[0])
#define M_BPS10(m)    get16(&(m)[2])
#define M_PEAK(m)     get16(&(m)[4])
#define M_WICK(m)     get16(&(m)[6])
#define M_FALL100(m)  get16(&(m)[8])
#define M_TSCALE(m)   get16(&(m)[10])
#define M_USCALE(m)   get16(&(m)[12])
#define M_PWMTOP(m)   get16(&(m)[14])
#define M_BUS(m)      get16(&(m)[16])
#define M_RISEEND(m)  get16(&(m)[18])
#define M_POINTS(m)   ((m)[20])
#define M_RISEN(m)    ((m)[21])

// How many chunks are needed for `points` points (on_receive derives the
// "all received" mask from it).
static inline uint8_t chunks_for(uint8_t points) {
  return (uint8_t)((points + CHUNK_LEN - 1) / CHUNK_LEN);
}

// I2C receive handler. Observed with ESP32 core 3.3.x: it is called from the
// Wire slave driver's task, not from an interrupt - millis() and critical
// sections are allowed here. The whole frame sits in the Wire buffer (the
// longest frame is 33 bytes; the driver's default buffer is 128).
static void on_receive(int n) {
  uint8_t b[LINK_RX_MAX];
  int k = 0;
  while (Wire.available()) {
    int c = Wire.read();
    if (k < (int)sizeof(b)) b[k++] = (uint8_t)c;
  }
  (void)n;

  if (k == 0) return;            // empty transaction (address probe) - not a frame
  // At least type+len+crc. The length must agree with len, otherwise it is garbage.
  if (k < 3 || (int)b[1] + 3 != k) { r_crc_err = r_crc_err + 1; return; }
  const uint8_t type = b[0], len = b[1];
  if (link_crc8(b, (uint8_t)(len + 2)) != b[len + 2]) { r_crc_err = r_crc_err + 1; return; }
  const uint8_t* pl = &b[2];

  bool good = true;
  portENTER_CRITICAL(&g_mux);
  if (type == FRM_STATUS && len == STATUS_LEN) {
    r_fw_major = pl[0];
    r_fw_minor = pl[1];
    r_flags    = pl[2];
    r_pulses   = get16(&pl[3]);
  } else if ((type & 0xF8) == FRM_META && len == META_LEN) {
    const uint8_t gen = (uint8_t)(type & 7);
    // Plausibility: garbage with a valid CRC (1 corrupted frame in 256) must not
    // break the drawing (e.g. a pw of 65535 wraps the axis arithmetic).
    const uint16_t pw = get16(&pl[0]);
    const uint8_t  points = pl[20], riseN = pl[21];
    if (pw > META_PW_MAX_MS || points == 0 || points > CURVE_MAX ||
        riseN >= points || get16(&pl[14]) == 0) {
      good = false;
    } else {
      // Same shape (everything except bps_x10 in bytes 2..3) - either a lone
      // META while the BPS knob is being turned, or a heartbeat. The curve stays
      // valid; it has to be handed over with a new caption only if META really changed.
      const bool same_shape = r_curve_valid && (memcmp(r_meta, pl, 2) == 0) &&
                              (memcmp(&r_meta[4], &pl[4], META_LEN - 4) == 0);
      if (same_shape) {
        const bool changed = (memcmp(r_meta, pl, META_LEN) != 0);
        memcpy(r_meta, pl, META_LEN);
        r_meta_valid  = true;
        r_expect_same = true;
        r_gen         = gen;
        if (changed) g_have_data = true;
      } else {
        // New generation. Chunks of the old one are discarded, and so is a set
        // not yet picked up by loop(), otherwise the old curve would reach the
        // screen with the new caption.
        memcpy(r_meta, pl, META_LEN);
        r_meta_valid  = true;
        r_expect_same = false;
        r_gen         = gen;
        r_curve_valid = false;
        r_gen_changed = false;
        g_set_done    = 0;
        g_have_data   = false;
      }
    }
  } else if ((type & 0xC0) == FRM_PREVIEW0 && (type & 7) < chunks_for(CURVE_MAX)) {
    const uint8_t idx = (uint8_t)(type & 7);
    const uint8_t gen = (uint8_t)((type >> 3) & 7);
    const uint16_t off = (uint16_t)idx * CHUNK_LEN;
    const uint8_t points = r_meta_valid ? M_POINTS(r_meta) : 0;
    if (!r_meta_valid || gen != r_gen || off >= points) {
      // A chunk without META (it was lost to CRC), of a foreign generation, or
      // with an out-of-range index: silently wait for the next set. This is not
      // frame corruption - it is not counted in crc.
    } else {
      const uint8_t expect = (uint8_t)(((points - off) < CHUNK_LEN) ? (points - off) : CHUNK_LEN);
      if (len != expect) {
        good = false;            // chunk length does not agree with the number of points
      } else if ((r_expect_same || (g_set_done & (uint8_t)(1u << idx))) &&
                 memcmp(&r_curve[off], pl, len) != 0) {
        // Within one generation the generator sends only identical bytes:
        // heartbeats and duplicates repeat r_curve byte for byte. Different
        // bytes in a chunk already received, or after a META of the same shape,
        // mean a new shape whose META was lost to CRC. Do not mix: reset the
        // generation and wait for the next complete set (the generator resends
        // the whole set about once a second, its LINK_REFRESH_MS).
        r_meta_valid  = false;
        g_set_done    = 0;
        r_curve_valid = false;   // otherwise a corrupted chunk with a valid CRC would stay until the shape changes
      } else {
        if (memcmp(&r_curve[off], pl, len) != 0) r_gen_changed = true;
        memcpy(&r_curve[off], pl, len);
        g_set_done |= (uint8_t)(1u << idx);
        const uint8_t need = chunks_for(points);
        const uint8_t mask = (uint8_t)((1u << need) - 1);
        if ((g_set_done & mask) == mask) {
          // Set complete. Hand it over if the bytes changed or there was no
          // curve yet; a repeated heartbeat with the same bytes leaves the screen alone.
          if (r_gen_changed || !r_curve_valid) g_have_data = true;
          r_curve_valid = true;
          r_meta_valid  = false;   // further chunks only after a new META
          r_gen_changed = false;
          g_set_done    = 0;
        }
      }
    }
  } else {
    good = false;                // unknown type or wrong length
  }
  if (good) {
    g_last_rx_ms = millis();
    g_link_ok    = true;
  }
  portEXIT_CRITICAL(&g_mux);
  if (!good) r_crc_err = r_crc_err + 1;
}

// ============================================================================
// CURVE GEOMETRY
// ============================================================================

// Time of point i on the non-uniform grid, ms. Mirrors the generator's preview_tick().
static float point_t_ms(uint8_t i) {
  const float pw      = (float)M_PW(g_meta);
  const uint8_t N     = M_POINTS(g_meta);
  const uint8_t riseN = M_RISEN(g_meta);
  const float rise_ms = pw * (float)M_RISEEND(g_meta) / 10000.0f;
  // riseN >= N-1: not a single interval is left for the fall (0/0) - the grid
  // is uniform.
  if (N < 2 || riseN == 0 || riseN >= N - 1) {
    return pw * (float)i / (float)(N > 1 ? N - 1 : 1);
  }
  if (i < riseN) return rise_ms * (float)i / (float)riseN;
  return rise_ms + (pw - rise_ms) * (float)(i - riseN) / (float)(N - 1 - riseN);
}

// Buck output voltage at time t_ms - linear interpolation of the curve on the
// non-uniform grid. Returns volts (0..bus).
static float __attribute__((unused)) curve_v_at_ms(float t_ms) {   // not used by the drawing code; kept as a reference implementation of the interpolation
  const uint8_t N = M_POINTS(g_meta);
  const float bus = (float)M_BUS(g_meta);
  if (N == 0) return 0.0f;
  if (N == 1 || t_ms <= 0.0f) return bus * g_curve[0] / 255.0f;
  const float pw = (float)M_PW(g_meta);
  if (t_ms >= pw) return bus * g_curve[N - 1] / 255.0f;
  // Find the segment: the grid is monotonic and there are few points - a linear scan is cheaper than a binary search.
  float t0 = 0.0f;
  for (uint8_t i = 1; i < N; i++) {
    const float t1 = point_t_ms(i);
    if (t_ms <= t1) {
      const float f = (t1 > t0) ? (t_ms - t0) / (t1 - t0) : 0.0f;
      const float v0 = g_curve[i - 1], v1 = g_curve[i];
      return bus * (v0 + (v1 - v0) * f) / 255.0f;
    }
    t0 = t1;
  }
  return bus * g_curve[N - 1] / 255.0f;
}

// Actual top of the curve (0..255). The PEAK knob sets the target BEFORE the
// generator's alpha compensation (+21 % with its default ALPHA_X1000_DEFAULT
// 210); the real ramp is higher - the caption shows what actually goes to the
// buck, the same as the plot.
static uint8_t curve_max_byte() {
  const uint8_t N = M_POINTS(g_meta);
  uint8_t m = 0;
  for (uint8_t i = 0; i < N && i < CURVE_MAX; i++) if (g_curve[i] > m) m = g_curve[i];
  return m;
}

static inline int v_to_y(float v, float bus, int top, int bottom) {
  if (bus <= 0.0f) return bottom;
  int y = bottom - (int)lroundf(v / bus * (float)(bottom - top));
  if (y < top) y = top;
  if (y > bottom) y = bottom;
  return y;
}

// ============================================================================
// DRAWING
// ============================================================================

static inline uint32_t fnv1a(uint32_t h, const uint8_t* p, size_t n) {
  while (n--) { h ^= *p++; h *= 16777619u; }
  return h;
}

// Top left: mode in large font. Changes rarely.
static uint32_t status_signature() {
  const uint8_t v[2] = {g_flags, (uint8_t)g_link_ok};
  return fnv1a(2166136261u, v, sizeof v);
}
// Top right: LINK OK / NO LINK, shots, crc. Changes on every pulse - its own
// area, so that the mode is not redrawn.
static uint32_t pulses_signature() {
  const uint8_t v[5] = {(uint8_t)g_pulses, (uint8_t)(g_pulses >> 8),
                        (uint8_t)g_crc_err, (uint8_t)(g_crc_err >> 8), (uint8_t)g_link_ok};
  return fnv1a(2166136261u, v, sizeof v);
}
// Axis labels and parameter lines: the whole META.
static uint32_t info_signature() {
  const uint8_t m = curve_max_byte();          // "peak" is derived from the curve
  return fnv1a(fnv1a(2166136261u, g_meta, META_LEN), &m, 1) ^ ((uint32_t)(g_cv > 0) << 31);
}
// Plot: curve bytes AND META without BPS. The generator computes the preview
// in normalised time, so when only the pulse width changes the bytes are often
// the same while the end of the pulse on the axis is different.
static uint32_t plot_signature() {
  const uint8_t N = M_POINTS(g_meta);
  uint32_t h = fnv1a(2166136261u, g_curve, (N <= CURVE_MAX) ? N : CURVE_MAX);
  h = fnv1a(h, &g_meta[0], 2);
  h = fnv1a(h, &g_meta[4], META_LEN - 4);
  return h ^ ((uint32_t)(g_cv > 0) << 31) ^ ((uint32_t)g_link_ok << 30);
}

static void draw_pulses() {
  tft.fillRect(HDR_SPLIT, 0, SCR_W - HDR_SPLIT, HDR_H - 2, COL_BG);
  tft.setTextDatum(TR_DATUM);
  tft.setTextFont(2);
  if (g_link_ok) { tft.setTextColor(COL_OK,  COL_BG); tft.drawString("LINK OK", SCR_W - 6, 2); }
  else           { tft.setTextColor(COL_BAD, COL_BG); tft.drawString("NO LINK", SCR_W - 6, 2); }
  char t[32];
  snprintf(t, sizeof t, "shots %u  crc %u", (unsigned)g_pulses, (unsigned)g_crc_err);
  tft.setTextFont(1);
  tft.setTextColor(COL_TXT2, COL_BG);
  tft.drawString(t, SCR_W - 6, 20);
  tft.setTextDatum(TL_DATUM);
}

// Display supply voltmeter: small (font 1, like shots/crc) in the bottom-right
// corner of the left half of the header, one decimal. Green above VMETER_OK_V,
// yellow in between, red below VMETER_LOW_V.
#define VOLTS_X0 (HDR_SPLIT - 44)
static uint32_t volts_signature() {
  const bool shown = g_volts_valid && g_volts >= VMETER_MIN_SHOW_V;
  return shown ? (((uint32_t)lroundf(g_volts * 10.0f) << 1) | 1u) : 0u;   // noise below the threshold does not trigger a redraw
}
static void draw_volts() {
  tft.fillRect(VOLTS_X0, 18, HDR_SPLIT - VOLTS_X0, 10, COL_BG);
  if (!g_volts_valid || g_volts < VMETER_MIN_SHOW_V) return;   // no divider - nothing is shown
  const unsigned v10 = (unsigned)lroundf(g_volts * 10.0f);
  char v[16];
  snprintf(v, sizeof v, "%u.%uV", v10 / 10, v10 % 10);
  uint32_t c = COL_ARMED;
  if (g_volts >= VMETER_OK_V)  c = COL_OK;
  if (g_volts <  VMETER_LOW_V) c = COL_BAD;
  tft.setTextFont(1);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(c, COL_BG);
  tft.drawString(v, HDR_SPLIT - 6, 20);
  tft.setTextDatum(TL_DATUM);
}
// Read once per VMETER_PERIOD_MS: the mean of VMETER_SAMPLES calibrated mV
// readings, then exponential smoothing so that the tenths do not jitter with ADC noise.
static void vmeter_poll(uint32_t now) {
  static uint32_t last = 0;
  if ((uint32_t)(now - last) < VMETER_PERIOD_MS) return;
  last = now;
  uint32_t mv = 0;
  for (uint8_t i = 0; i < VMETER_SAMPLES; i++) mv += analogReadMilliVolts(PIN_VMETER);
  const float v = (float)mv / VMETER_SAMPLES / 1000.0f * VMETER_DIV;
  g_volts = g_volts_valid ? (g_volts * 0.5f + v * 0.5f) : v;   // ~1 s to settle
  g_volts_valid = true;
}

static void draw_status() {
  tft.fillRect(0, 0, HDR_SPLIT, HDR_H - 2, COL_BG);
  const char* w; uint32_t c;
  if (!g_link_ok)                { w = "OFFLINE"; c = COL_IDLE;  }
  else if (g_flags & FLAG_ESTOP) { w = "E-STOP";  c = COL_BAD;   }
  else if (g_flags & FLAG_AUTO)  { w = "AUTO";    c = COL_AUTO;  }
  else if (g_flags & FLAG_ARMED) { w = "ARMED";   c = COL_ARMED; }
  else                           { w = "IDLE";    c = COL_IDLE;  }
  // Font 4: OFFLINE 102 px, E-STOP 89, ARMED 88 (TFT_eSPI 2.5.43 width tables) -
  // drawn from x=6, none of them reaches the voltmeter (VOLTS_X0 = 136).
  tft.setTextFont(4);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(c, COL_BG);
  tft.drawString(w, 6, 2);
  draw_volts();
  draw_pulses();
}

// The time axis is FIXED: 0..AXIS_MS_MIN (this is the generator's
// PULSE_WIDTH_MAX_MS) plus PRE_MS of zero before the pulse, so that the ramp on
// screen grows and shrinks together with the PW knob. If the generator sends a
// larger pw, the axis grows to the nearest 5 ms.
#define AXIS_MS_MIN 25
static uint16_t axis_ms() {
  const uint16_t pw = M_PW(g_meta);
  uint16_t a = AXIS_MS_MIN;
  if (pw > a) a = (uint16_t)(((pw + 4) / 5) * 5);
  return (uint16_t)(a + PRE_MS);
}
static inline int ms_to_x(int ox, float ms, float axis) {
  return ox + (int)lroundf((float)(PLOT_W - 1) * ms / axis);
}
static uint16_t axis_step_ms(uint16_t axis) {
  return (axis <= 30 + PRE_MS) ? 5 : (axis <= 60 + PRE_MS ? 10 : 25);
}

// Axis labels around the plot (outside the sprite): on the left bus voltage in
// quarters (325/243/162/81/0 for a 325 V bus) and "V", at the bottom
// 0 5 10 15 20 25 (for the 25 ms axis) and "ms".
static void draw_axes() {
  const uint16_t bus  = M_BUS(g_meta);
  const uint16_t axis = axis_ms();
  tft.fillRect(0, PLOT_Y - 12, PLOT_X - 1, PLOT_H + 24, COL_BG);
  tft.fillRect(PLOT_X - 4, PLOT_Y + PLOT_H, PLOT_W + 14, 12, COL_BG);
  if (g_cv == 0) return;                     // no META yet - nothing to label
  tft.setTextFont(1);
  tft.setTextColor(COL_LBL, COL_BG);
  char s[12];
  tft.setTextDatum(TR_DATUM);
  tft.drawString("V", PLOT_X - 6, PLOT_Y - 12);
  for (int i = 0; i <= 4; i++) {
    const int y = PLOT_Y + (PLOT_H - 1) * i / 4;
    snprintf(s, sizeof s, "%u", (unsigned)((uint32_t)bus * (unsigned)(4 - i) / 4u));
    tft.drawString(s, PLOT_X - 6, y - 3);
  }
  tft.setTextDatum(TL_DATUM);
  tft.drawString("ms", 6, PLOT_Y + PLOT_H + 3);
  const uint16_t step = axis_step_ms(axis);
  // Tick labels are in PULSE time: zero is where the curve starts, i.e.
  // PRE_MS from the left edge, the same as the curve itself.
  for (int ms = 0; ms + PRE_MS <= (int)axis; ms += (int)step) {   // int: uint16 wrapped around at 65535
    const int x = ms_to_x(PLOT_X, (float)(ms + PRE_MS), (float)axis);
    tft.setTextDatum(ms == 0 ? TL_DATUM : ((ms + (int)step + PRE_MS > (int)axis) ? TR_DATUM : TC_DATUM));
    snprintf(s, sizeof s, "%d", ms);
    tft.drawString(s, x, PLOT_Y + PLOT_H + 3);
  }
  tft.setTextDatum(TL_DATUM);
}

// The plot itself: drawn into the sprite and pushed in one go - no flicker.
static void draw_plot() {
  const uint32_t t_draw0 = millis();
  const float bus  = (float)M_BUS(g_meta);
  const float pw   = (float)M_PW(g_meta);
  const float axis = (float)axis_ms();

  TFT_eSPI* d = g_spr_ok ? (TFT_eSPI*)&spr : &tft;
  const int ox = g_spr_ok ? 0 : PLOT_X;
  const int oy = g_spr_ok ? 0 : PLOT_Y;
  const int bottom = oy + PLOT_H - 1;
  const int right  = ox + PLOT_W - 1;

  d->fillRect(ox, oy, PLOT_W, PLOT_H, COL_BG);

  // Dotted grid: 4 divisions in V; in time - the axis step (5 ms on a 25 ms axis).
  for (int i = 1; i < 4; i++) {
    const int y = oy + (PLOT_H - 1) * i / 4;
    for (int x = 0; x < PLOT_W; x += 3) d->drawPixel(ox + x, y, COL_GRID);
  }
  const uint16_t step = axis_step_ms((uint16_t)axis);
  for (uint16_t ms = step; ms + PRE_MS < (uint16_t)axis; ms = (uint16_t)(ms + step)) {
    const int x = ms_to_x(ox, (float)(ms + PRE_MS), axis);    // grid in pulse time, like the tick labels
    for (int y = 0; y < PLOT_H; y += 3) d->drawPixel(x, oy + y, COL_GRID);
  }
  // Axes.
  d->drawFastVLine(ox, oy, PLOT_H, COL_AXIS);
  d->drawFastHLine(ox, bottom, PLOT_W, COL_AXIS);

  if (g_cv > 0) {
    // The envelope is drawn as anti-aliased segments between the NODES of the
    // curve: the line is smooth rather than a staircase of columns. Without a
    // sprite we clip to the plot area with a viewport, so that the anti-aliasing
    // halo does not spill over the labels.
    if (!g_spr_ok) tft.setViewport(PLOT_X, PLOT_Y, PLOT_W, PLOT_H, false);
    const uint8_t N = M_POINTS(g_meta);
    // Node coordinates are FRACTIONAL: the anti-aliased line places them with
    // sub-pixel precision, so the curve comes out smooth, like a plotted
    // function, rather than a staircase of integer pixels.
    const float kx = (float)(PLOT_W - 1) / axis;                 // px per ms
    const float ky = (bus > 0.0f) ? (float)(PLOT_H - 1) / bus : 0.0f;   // px per volt
    const float fb = (float)bottom;
    // PRE_MS of zero, then a sharp rise to wick, then the ramp, and a drop to zero at the end.
    const float x_start = (float)ox + kx * (float)PRE_MS;
    float x_end = (float)ox + kx * ((float)PRE_MS + pw);
    if (x_end > (float)right) x_end = (float)right;
    if (x_end < x_start)      x_end = x_start;
    d->drawWideLine((float)ox, fb, x_start, fb, CURVE_W, COL_CURVE, COL_BG);
    float px = x_start, py = fb;
    for (uint8_t i = 0; i < N && i < CURVE_MAX; i++) {
      float x = (float)ox + kx * ((float)PRE_MS + point_t_ms(i));
      if (x > x_end) x = x_end;
      float y = fb - ky * (bus * (float)g_curve[i] / 255.0f);
      // Not higher than half the line width from the top: otherwise a 100 %
      // plateau is cut in half by the plot boundary.
      if (y < (float)oy + CURVE_W * 0.5f) y = (float)oy + CURVE_W * 0.5f;
      d->drawWideLine(px, py, x, y, CURVE_W, COL_CURVE, COL_BG);
      px = x; py = y;
    }
    d->drawWideLine(px, py, x_end, fb, CURVE_W, COL_CURVE, COL_BG);
    if (!g_spr_ok) tft.resetViewport();
  } else {
    d->setTextFont(2);
    d->setTextDatum(MC_DATUM);
    d->setTextColor(COL_LBL, COL_BG);
    d->drawString(g_link_ok ? "waiting for curve..." : "no data", ox + PLOT_W / 2, oy + PLOT_H / 2);
    d->setTextDatum(TL_DATUM);
  }

  if (g_spr_ok) spr.pushSprite(PLOT_X, PLOT_Y);
  // Speed diagnostics: how long the plot took, maximum over 5 s - to the serial port.
  static uint32_t max_ms = 0, last_report = 0, n_draws = 0;
  const uint32_t took = millis() - t_draw0;
  if (took > max_ms) max_ms = took;
  n_draws++;
  if ((uint32_t)(millis() - last_report) >= 5000) {
    Serial.print(F("plot: ")); Serial.print(n_draws); Serial.print(F(" redraws in 5 s, max "));
    Serial.print(max_ms); Serial.println(F(" ms"));
    max_ms = 0; n_draws = 0; last_report = millis();
  }
}

// Two parameter lines under the plot - with the numbers the curve was computed
// from (META), not the live knob values.
//   peak 325V  wick 44V  fall 18%  0.5 BPS
//   25ms  624 steps @ 25kHz  ~0.4V/step
// peak - the actual top of the ramp in volts (with alpha compensation, as on the
// plot); wick - the knob in bus volts. If the wick knob is above the peak, the
// generator clamps wick to peak: we show the effective value and the knob in
// parentheses. Steps - carrier periods per pulse: carrier = 16 MHz / (pwm_top+1)
// (the generator's RAMP_RES_HZ).
static void draw_info() {
  tft.fillRect(0, INFO_Y, SCR_W, SCR_H - INFO_Y, COL_BG);
  tft.setTextDatum(TL_DATUM);
  if (g_cv == 0) {
    tft.setTextFont(2);
    tft.setTextColor(COL_TXT2, COL_BG);
    tft.drawString("waiting for generator...", 6, INFO_Y);
    return;
  }
  const uint32_t top = M_PWMTOP(g_meta) ? M_PWMTOP(g_meta) : 1;
  const uint32_t bus = M_BUS(g_meta);
  const uint16_t peak = M_PEAK(g_meta), wick_knob = M_WICK(g_meta);
  const uint16_t wick_eff = (wick_knob > peak) ? peak : wick_knob;
  // wick: truncation, not rounding - the generator's CONFIG banner computes its
  // wick volts the same way (duty 88 -> "~44 V" on a 325 V bus, pwm_top 640).
  // peak is rounded, see below.
  const unsigned peak_v = (unsigned)(((uint32_t)curve_max_byte() * bus + 127u) / 255u);   // top of the ramp (rounded), not the knob
  const unsigned wick_v = (unsigned)(wick_eff * bus / top);
  const unsigned knob_v = (unsigned)(wick_knob * bus / top);
  const uint16_t bps = M_BPS10(g_meta);
  char l1[64], l2[64];
  if (wick_eff == wick_knob)
    snprintf(l1, sizeof l1, "peak %uV  wick %uV  fall %u%%  %u.%u BPS",
             peak_v, wick_v, (unsigned)M_FALL100(g_meta), (unsigned)(bps / 10), (unsigned)(bps % 10));
  else   // with the knob in parentheses the line is longer - single spaces to fit into 320 px
    snprintf(l1, sizeof l1, "peak %uV wick %uV(%u) fall %u%% %u.%u BPS",
             peak_v, wick_v, knob_v, (unsigned)M_FALL100(g_meta), (unsigned)(bps / 10), (unsigned)(bps % 10));
  // Carrier: the generator's timer counts up to pwm_top inclusive, the period is
  // pwm_top+1 ticks at 16 MHz (RAMP_PERIOD_TICKS), i.e. 24961 Hz at 640.
  const uint32_t f_hz  = 16000000UL / (top + 1);
  const uint32_t steps = (uint32_t)M_PW(g_meta) * f_hz / 1000UL;
  const uint32_t dv10  = steps ? ((uint32_t)(peak_v > wick_v ? peak_v - wick_v : 0) * 10UL) / steps : 0;
  snprintf(l2, sizeof l2, "%ums  %lu steps @ %lukHz  ~%lu.%luV/step",
           (unsigned)M_PW(g_meta), (unsigned long)steps, (unsigned long)((f_hz + 500UL) / 1000UL),
           (unsigned long)(dv10 / 10), (unsigned long)(dv10 % 10));
  tft.setTextFont(2);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.drawString(l1, 6, INFO_Y);
  tft.setTextFont(1);
  tft.setTextColor(COL_TXT2, COL_BG);
  tft.drawString(l2, 6, INFO_Y + 20);
}

// Whether a curve set arrived within the last RX_BLINK_MS.
#define RX_BLINK_MS 300UL
static inline bool new_curve_recent(uint32_t now) {
  return g_cv > 0 && (uint32_t)(now - g_sig_changed_ms) < RX_BLINK_MS;
}

static void set_led(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r ? LOW : HIGH);
  digitalWrite(PIN_LED_G, g ? LOW : HIGH);
  digitalWrite(PIN_LED_B, b ? LOW : HIGH);
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println(F("CYD RAMP VIEW - QCW ramp display (I2C slave 0x2A, SDA=27 SCL=22)"));

  pinMode(PIN_LED_R, OUTPUT); pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_B, OUTPUT);
  set_led(false, false, true);               // blue: booting

  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);

  // Voltmeter: explicitly 11 dB (0..~3.1 V on the pin, calibrated mV). ESP32 core
  // 3.3.x uses this by default; the line is here so that it does not depend on the core version.
  analogSetPinAttenuation(PIN_VMETER, ADC_11db);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(COL_BG);

  spr.setColorDepth(16);
  g_spr_ok = (spr.createSprite(PLOT_W, PLOT_H) != nullptr);
  Serial.print(F("sprite ")); Serial.print(g_spr_ok ? F("OK") : F("FAILED, drawing direct"));
  Serial.print(F(", free heap ")); Serial.println(ESP.getFreeHeap());

  memset(g_curve, 0, sizeof g_curve);
  memset(r_curve, 0, sizeof r_curve);

  // Slave. The master sets the bus clock; observed with ESP32 core 3.3.x, the
  // slave driver uses this value only for its own timing and FIFO-threshold setup.
  Wire.onReceive(on_receive);
  g_wire_ok = Wire.begin((uint8_t)LINK_SLAVE_ADDR, PIN_LINK_SDA, PIN_LINK_SCL, LINK_FREQ_HZ);
  Serial.print(F("Wire slave ")); Serial.println(g_wire_ok ? F("OK") : F("FAILED, will retry"));

  g_forced = true;
  draw_status();
  draw_axes();
  draw_plot();
  draw_info();
  set_led(true, false, false);               // red: no link yet
}

void loop() {
  // The slave may have failed to start: observed with ESP32 core 3.3.x, the
  // slave driver refuses to start if SDA/SCL are low at that moment (the
  // generator is unpowered - its protection diodes pull the lines down; or the
  // master is in the middle of a frame). Retry once a second until it succeeds.
  if (!g_wire_ok) {
    static uint32_t last_try = 0;
    if ((uint32_t)(millis() - last_try) >= 1000) {
      last_try = millis();
      g_wire_ok = Wire.begin((uint8_t)LINK_SLAVE_ADDR, PIN_LINK_SDA, PIN_LINK_SCL, LINK_FREQ_HZ);
      Serial.println(g_wire_ok ? F("Wire slave OK (retry)") : F("Wire slave: retry failed"));
    }
  }

  // --- auto refresh every AUTO_REFRESH_MIN minutes --------------------------
  if ((uint32_t)(millis() - g_last_refresh_ms) >= (uint32_t)AUTO_REFRESH_MIN * 60000UL) {
    g_last_refresh_ms = millis();
    tft.init();                              // re-initialise the controller
    tft.setRotation(1);
    tft.fillScreen(COL_BG);
    portENTER_CRITICAL(&g_mux);
    r_crc_err = 0;
    portEXIT_CRITICAL(&g_mux);
    g_forced = true;                         // everything from scratch
    Serial.println(F("auto refresh: display re-initialised, crc cleared"));
  }

  // --- snapshot of the receive side under the lock ---------------------------
  bool new_curve = false;
  portENTER_CRITICAL(&g_mux);
  if (g_have_data) {
    const uint8_t N = M_POINTS(r_meta);
    memcpy(g_meta, r_meta, META_LEN);
    memcpy(g_curve, r_curve, (N && N <= CURVE_MAX) ? N : CURVE_MAX);
    g_have_data = false;
    g_cv++;
    new_curve = true;
  }
  g_flags    = r_flags;
  g_pulses   = r_pulses;
  g_fw_major = r_fw_major;
  g_fw_minor = r_fw_minor;
  g_crc_err  = r_crc_err;
  // millis() under the lock and a signed difference: otherwise a frame arriving
  // between taking the time and the check would give an "age" of 0xFFFFFFFF and a false link loss.
  const uint32_t now = millis();
  const int32_t age = (int32_t)(now - g_last_rx_ms);
  if (g_link_ok && age > (int32_t)LINK_TIMEOUT_MS) g_link_ok = false;
  const bool link_ok = g_link_ok;
  portEXIT_CRITICAL(&g_mux);

  // Compare with what is drawn, not with the start of the iteration: on_receive
  // runs in another task and can raise g_link_ok between two passes.
  if (link_ok != g_link_shown) {
    g_link_shown = link_ok;
    Serial.println(link_ok ? F("link up") : F("link LOST"));
    g_forced = true;
  }
  if (new_curve) g_sig_changed_ms = now;
  const bool rx_active = link_ok && new_curve_recent(now);
  g_rx_active = rx_active;

  // --- header: mode on the left, link/shots/crc on the right in its own area --
  vmeter_poll(now);
  const uint32_t ssig = status_signature();
  const uint32_t psig = pulses_signature();
  const uint32_t vsig = volts_signature();
  if (g_forced || ssig != g_status_sig) {
    g_status_sig = ssig;
    g_pulses_sig = psig;
    g_volts_sig  = vsig;
    draw_status();                          // draws the voltmeter and the right part inside
  } else {
    if (psig != g_pulses_sig) { g_pulses_sig = psig; draw_pulses(); }
    if (vsig != g_volts_sig)  { g_volts_sig  = vsig; draw_volts();  }
  }

  // --- axis labels and parameter lines: by META -----------------------------
  const uint32_t isig = info_signature();
  if (g_forced || isig != g_info_sig) {
    g_info_sig = isig;
    draw_info();
    draw_axes();
  }

  // --- plot: curve bytes + META without BPS ----------------------------------
  const uint32_t plsig = plot_signature();
  if (g_forced || plsig != g_plot_sig) {
    g_plot_sig = plsig;
    draw_plot();
  }

  // --- LED: green = link, red = link lost, blinking on E-STOP ----------------
  if (g_flags & FLAG_ESTOP)      set_led((now / 250) & 1, false, false);
  else if (link_ok)              set_led(false, true, rx_active);   // cyan flash on a new curve
  else                           set_led(true, false, false);

  g_forced = false;
  delay(5);                                  // 5 ms response; we draw only on changes
}
