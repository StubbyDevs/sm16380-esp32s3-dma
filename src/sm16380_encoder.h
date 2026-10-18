// SM16380 stream encoder: turns a frame into the exact sequence of line states
// (one 16-bit word per bus cycle) that drives an SM16380 S-PWM panel.
//
// Hardware-free on purpose: SM16380_DMA.cpp streams these words with LCD_CAM +
// GDMA, and test/encoder_test.cpp compiles this header on a PC to check the
// stream against a reference model of the protocol.
//
// Bus word bits (one per panel line):
//   0..5   R1 G1 B1 R2 G2 B2
//   6..10  A B C D E (row address)
//   11     LAT
//   12     OE
//   13     CLK (a data bit, so the stream sets the shape of every panel clock)
//
// Protocol (see extras/PROTOCOL.md):
//   - Config: 32 registers, each written with the FPP/kingdo9 grammar
//     (VSYNC, LAT 14, then 00AA 01AA <reg> F003 0055 0155, latched over the last
//     5 clocks of the chain).
//   - Upload: 32 rows x 16 slots x chips, a 16-bit word per pixel per chip
//     (level << 9), LAT on the last bit of the last chip, then VSYNC.
//   - Scan: CLK never stops (it is the PWM clock). One 4-clock OE pulse per row,
//     row address set 28 clocks before it, 232 clocks per row.
//   - The chip counts OE pulses to know its row; only a long held OE resets the
//     count. So an upload either holds OE (panel blanks briefly, count reset) or
//     keeps scanning through it ("continuous") and is padded to whole scan
//     passes, so the count and the address never drift apart.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace sm16380 {

// One panel: 128 x 64, 1/32 scan, 8 chips per colour lane (16 columns each).
constexpr int PANEL_W = 128, PANEL_H = 64, ROWS = 32, CHIPS = 8;
constexpr int MAX_PANELS = 3;
constexpr int ROW_CLKS = 232;    // panel clocks per row
constexpr int LOOK_BEHIND = 28;  // row address change, clocks before the row's OE pulse
constexpr int OE_CLKS = 4;
constexpr int SCAN_CLKS = ROW_CLKS * ROWS;

constexpr uint16_t BUS_RGB = 0x3f;
constexpr int BUS_ADDR_SHIFT = 6;
constexpr uint16_t BUS_ADDR = 0x1f << BUS_ADDR_SHIFT;
constexpr uint16_t BUS_LAT = 1 << 11, BUS_OE = 1 << 12, BUS_CLK = 1 << 13;
constexpr int BUS_LINES = 14;

// Register values: kingdo9 sm16380sh.profiles, regtype2. The first one is
// 0x0200 | (scan lines - 1).
static const uint16_t CFG_R[32] = {0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x072c, 0x0800, 0x0900,
                                   0x0a02, 0x0b0c, 0x0c08, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100,
                                   0x1200, 0x1308, 0x1414, 0x1500, 0x1630, 0x1700, 0x1801, 0x1904,
                                   0x1a03, 0x1b14, 0x1c12, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200};
static const uint16_t CFG_G[32] = {0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x072c, 0x0800, 0x0900,
                                   0x0a02, 0x0b0c, 0x0c18, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100,
                                   0x1200, 0x1308, 0x1422, 0x1500, 0x1630, 0x1700, 0x1801, 0x1903,
                                   0x1a01, 0x1b14, 0x1c8f, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200};
static const uint16_t CFG_B[32] = {0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x072c, 0x0800, 0x0900,
                                   0x0a02, 0x0b0c, 0x0c30, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100,
                                   0x1200, 0x1308, 0x1432, 0x1500, 0x1630, 0x1700, 0x1801, 0x1903,
                                   0x1a01, 0x1b14, 0x1c8f, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200};

// Which corner panel of the arrangement the controller plugs into.
enum class Corner : uint8_t { TopLeft, TopRight, BottomLeft, BottomRight };

// A buffer of bus words.
struct Stream {
  uint16_t *buf;
  size_t cap, n;
};

// Where the panel's own scan is: the row its OE count is on, the clock within
// the row, OE clocks left, and whether a held-OE opener is still running.
struct ScanState {
  int row, phase, oeLeft;
  bool opener;
};

// Panel p of the chain (0 = the one the controller plugs into) shows the
// 128 x 64 window at (ox, oy) of the picture, upside down when rot.
struct Placement {
  int ox, oy;
  bool rot;
};

class Encoder {
 public:
  // ---- layout
  int panelsX = 1, panelsY = 1, nPanels = 1;
  int width = PANEL_W, height = PANEL_H;  // the whole picture
  int chainChips = CHIPS;
  // The first chip shifted out ends up in the panel farthest from the
  // controller (plain shift-register order). Inside a panel it is the left edge.
  bool firstShiftIsFar = true;
  Placement place[MAX_PANELS] = {{0, 0, false}};

  // Chain order -> grid cells. Rows run away from the controller's corner;
  // snake runs every other row back the other way, those panels mounted
  // upside down so short cables reach.
  void setLayout(int px, int py, Corner first, bool snake) {
    if (px < 1) px = 1;
    if (py < 1) py = 1;
    while (px * py > MAX_PANELS) (px >= py ? px : py)--;
    panelsX = px;
    panelsY = py;
    nPanels = px * py;
    width = PANEL_W * px;
    height = PANEL_H * py;
    chainChips = CHIPS * nPanels;
    bool top = first == Corner::TopLeft || first == Corner::TopRight;
    bool left = first == Corner::TopLeft || first == Corner::BottomLeft;
    for (int k = 0; k < nPanels; k++) {
      int r = k / px, i = k % px;
      bool back = snake && (r & 1);
      int gx = (left != back) ? i : px - 1 - i;
      int gy = top ? r : py - 1 - r;
      place[k] = {gx * PANEL_W, gy * PANEL_H, back};
    }
  }

  // ---- stream state
  const uint16_t *fb = nullptr;  // RGB444 (0x0RGB), width x height, row-major
  Stream *out = nullptr;
  ScanState sc = {};
  uint16_t lines = 0, prev = 0;  // line state now, and before the last change
  int busDiv = 16;               // 160 MHz / busDiv words per second
  int upK = 2;                   // words per panel clock during uploads (2 = fastest)
  bool overflow = false;
  uint32_t clocks = 0;           // panel clocks written

  // Config words per clock: 1.2 us per clock whatever the bus rate.
  int cfgK() const { return 8 * 24 / busDiv; }

  void reset() {
    sc = ScanState{};
    lines = prev = 0;
    clocks = 0;
  }

  // All 32 config registers, each followed by 2 ms with the clock stopped.
  // (The DMA driver writes configBegin() + configReg() and serves the pauses
  // from one shared buffer instead of storing them: same words on the bus.)
  void config() {
    configBegin();
    for (int i = 0; i < 32; i++) {
      configReg(i);
      put(lines, CONFIG_PAUSE_WORDS);
    }
  }
  void configBegin() {
    k_ = cfgK();
    lines &= ~(BUS_OE | BUS_LAT | BUS_RGB);
  }
  void configReg(int i) { writeReg(CFG_R[i], CFG_G[i], CFG_B[i]); }
  static constexpr int CONFIG_PAUSE_WORDS = 2016 * 10;  // ~2 ms at 10 M words/s (10 x 4032-byte descriptors)

  // The frame's upload. continuous: keep scanning through it (no blank) and
  // pad to whole scan passes; otherwise hold OE for the whole upload, which
  // blanks the panel for its length and resets the chip's row count.
  void upload(bool continuous) {
    k_ = upK;
    lines &= ~(BUS_LAT | BUS_RGB);
    uint32_t clocks0 = clocks;
    if (!continuous) {
      sc = ScanState{0, 0, 1 << 30, true};
      setRow(0);
    }
    if (k_ == 2 && !(out->n & 1) && out->n + 2 * (size_t)ROWS * 16 * chainChips * 16 <= out->cap) {
      uploadData2();
    } else {
      for (int y = 0; y < ROWS; y++)
        for (int slot = 0; slot < 16; slot++)
          for (int c = 0; c < chainChips; c++) {
            int p = chipPanel(c), lx = (c % CHIPS) * 16 + slot;
            uint16_t pt = fb[pixIdx(p, lx, y)], pb = fb[pixIdx(p, lx, y + ROWS)];
            bool last = c == chainChips - 1;
            for (int bit = 15; bit >= 0; bit--) {
              uint16_t d = (bit >= 9 && bit <= 12) ? lanes(pt, pb, bit - 9) : 0;
              tick<false>(d, last && bit == 0);
            }
          }
    }
    if (sc.opener) sc.oeLeft = 1;  // held OE: drop it on the next clock, which starts the scan
    tick<false>(0, false);
    lines |= BUS_LAT;  // VSYNC: LAT for 3 clocks
    for (int i = 0; i < 3; i++) tick<false>(0, false);
    lines &= ~BUS_LAT;
    // Continuous: scan on to a whole number of passes, so every upload starts
    // in the same scan state (and its control bits can be reused, see patch).
    if (continuous)
      while ((clocks - clocks0) % SCAN_CLKS) tick<false>(0, false);
  }

  // One full scan pass. From the same start state it is always the same
  // words and ends in the same state.
  void scanPass() {
    for (int i = 0; i < SCAN_CLKS; i++) tick<true>(0, false);
  }

  // Rewrite only the pixel bits of an upload encoded at upK = 2 (control
  // bits, clock and latches stay as they are).
  void patch(Stream &p) const {
    uint32_t *w = (uint32_t *)p.buf;
    const uint32_t keep = ~((uint32_t)BUS_RGB | (uint32_t)BUS_RGB << 16);
    for (int y = 0; y < ROWS; y++)
      for (int slot = 0; slot < 16; slot++)
        for (int k = 0; k < chainChips; k++) {
          int pn = chipPanel(k), lx = (k % CHIPS) * 16 + slot;
          uint16_t pt = fb[pixIdx(pn, lx, y)], pb = fb[pixIdx(pn, lx, y + ROWS)];
          uint32_t *c = w + ((y * 16 + slot) * chainChips + k) * 16 + 3;  // the clocks for bits 12..9
          for (int lb = 3; lb >= 0; lb--, c++) {
            uint32_t d = lanes(pt, pb, lb);
            *c = (*c & keep) | d | d << 16;
          }
        }
  }

  int pixIdx(int p, int lx, int ly) const {
    const Placement &q = place[p];
    return q.rot ? (q.oy + PANEL_H - 1 - ly) * width + q.ox + PANEL_W - 1 - lx : (q.oy + ly) * width + q.ox + lx;
  }
  // The panel chip c of the chain (0 = shifted first) sits in.
  int chipPanel(int c) const { return firstShiftIsFar ? nPanels - 1 - c / CHIPS : c / CHIPS; }

  // Lane bits for level bit lb (0..3) of a top-half and a bottom-half pixel.
  static uint16_t lanes(uint16_t top, uint16_t bot, int lb) {
    return ((top >> (8 + lb)) & 1) | (((top >> (4 + lb)) & 1) << 1) | (((top >> lb) & 1) << 2) |
           (((bot >> (8 + lb)) & 1) << 3) | (((bot >> (4 + lb)) & 1) << 4) | (((bot >> lb) & 1) << 5);
  }

 private:
  int k_ = 2;  // words per clock of the phase being written

  void put(uint16_t w, int count) {
    if (out->n + count > out->cap) {
      overflow = true;
      return;
    }
    uint16_t *p = out->buf + out->n;
    out->n += count;
    while (count-- > 0) *p++ = w;
  }

  // One panel clock with the current lines. Fast (or k = 2): one word low,
  // one high. Slow: the old lines for the first quarter, the new ones from
  // mid low phase, then the high half, so nothing moves next to an edge.
  void clk(bool fast) {
    if (fast || k_ == 2) {
      if (!(out->n & 1) && out->n + 2 <= out->cap) {  // one 32-bit store, low word first
        *(uint32_t *)(out->buf + out->n) = lines | (uint32_t)(lines | BUS_CLK) << 16;
        out->n += 2;
      } else {
        put(lines, 1);
        put(lines | BUS_CLK, 1);
      }
    } else {
      int low = k_ / 2, early = k_ / 4;
      put(prev, early);
      put(lines, low - early);
      put(lines | BUS_CLK, k_ - low);
    }
    prev = lines;
    clocks++;
  }

  void setRow(int row) { lines = (lines & ~BUS_ADDR) | (row << BUS_ADDR_SHIFT); }

  // One clock of data, with the row scan running alongside.
  template <bool FAST>
  void tick(uint16_t data, bool latch) {
    if (!sc.opener) {
      if (sc.phase == ROW_CLKS - LOOK_BEHIND) {
        sc.row = (sc.row + 1) & (ROWS - 1);
        setRow(sc.row);
      }
      if (sc.phase == 0 && sc.oeLeft == 0) sc.oeLeft = OE_CLKS;
    }
    lines = (lines & ~BUS_RGB) | data;
    if (latch) lines |= BUS_LAT;
    if (sc.oeLeft > 0) lines |= BUS_OE;
    clk(FAST);
    if (latch) lines &= ~BUS_LAT;
    if (sc.oeLeft > 0 && --sc.oeLeft == 0) lines &= ~BUS_OE;
    if (sc.opener) {
      if (sc.oeLeft == 0) {
        sc.opener = false;
        sc.phase = ROW_CLKS - LOOK_BEHIND;
      }
    } else if (++sc.phase >= ROW_CLKS) {
      sc.phase = 0;
    }
  }

  void slowClocks(int n) {
    lines &= ~BUS_RGB;
    while (n-- > 0) clk(false);
  }

  void slowLatches(int n) {
    lines = (lines & ~BUS_RGB) | BUS_LAT;
    while (n-- > 0) clk(false);
    lines &= ~BUS_LAT;
  }

  // A config word to every chip in the chain, LAT over its last `latches` clocks.
  void wordRGB(uint16_t r, uint16_t g, uint16_t b, int latches) {
    const int n = chainChips * 16;
    for (int i = 0; i < n; i++) {
      uint16_t mask = (uint16_t)(0x8000 >> (i & 15));
      uint16_t on = 0;
      if (r & mask) on |= (1 << 0) | (1 << 3);
      if (g & mask) on |= (1 << 1) | (1 << 4);
      if (b & mask) on |= (1 << 2) | (1 << 5);
      if (i == n - latches) lines |= BUS_LAT;
      lines = (lines & ~BUS_RGB) | on;
      clk(false);
    }
    lines &= ~(BUS_LAT | BUS_RGB);
  }

  // FalconChristmas/fpp BBShiftPanel grammar for the SM16380.
  void writeReg(uint16_t r, uint16_t g, uint16_t b) {
    lines &= ~BUS_RGB;  // VSYNC
    clk(false);
    slowLatches(3);
    slowClocks(8);
    slowLatches(14);
    slowClocks(8);
    const uint16_t words[6][3] = {{0x00AA, 0x00AA, 0x00AA}, {0x01AA, 0x01AA, 0x01AA}, {r, g, b},
                                  {0xF003, 0xF003, 0xF003}, {0x0055, 0x0055, 0x0055}, {0x0155, 0x0155, 0x0155}};
    for (auto &w : words) {
      wordRGB(w[0], w[1], w[2], 5);
      slowClocks(8);
    }
  }

  // The upload's pixel data at k = 2: the same clocks as the generic loop in
  // upload(), with the state in locals and one 32-bit store per clock (with
  // the state in memory every store forced a reload: ~37 ms a frame on the S3).
  void uploadData2() {
    int row = sc.row, phase = sc.phase, oeLeft = sc.oeLeft;
    bool opener = sc.opener;
    uint16_t L = lines;
    uint32_t *o = (uint32_t *)(out->buf + out->n);
#define SM16380_TICK2(data, latch)                     \
  do {                                                 \
    if (!opener) {                                     \
      if (phase == ROW_CLKS - LOOK_BEHIND) {           \
        row = (row + 1) & (ROWS - 1);                  \
        L = (L & ~BUS_ADDR) | (row << BUS_ADDR_SHIFT); \
      }                                                \
      if (phase == 0 && oeLeft == 0) oeLeft = OE_CLKS; \
    }                                                  \
    L = (L & ~BUS_RGB) | (data);                       \
    if (latch) L |= BUS_LAT;                           \
    if (oeLeft > 0) L |= BUS_OE;                       \
    *o++ = L | (uint32_t)(L | BUS_CLK) << 16;          \
    if (latch) L &= ~BUS_LAT;                          \
    if (oeLeft > 0 && --oeLeft == 0) L &= ~BUS_OE;     \
    if (opener) {                                      \
      if (oeLeft == 0) {                               \
        opener = false;                                \
        phase = ROW_CLKS - LOOK_BEHIND;                \
      }                                                \
    } else if (++phase >= ROW_CLKS) {                  \
      phase = 0;                                       \
    }                                                  \
  } while (0)
    for (int y = 0; y < ROWS; y++)
      for (int slot = 0; slot < 16; slot++)
        for (int c = 0; c < chainChips; c++) {
          int p = chipPanel(c), lx = (c % CHIPS) * 16 + slot;
          uint16_t pt = fb[pixIdx(p, lx, y)], pb = fb[pixIdx(p, lx, y + ROWS)];
          bool last = c == chainChips - 1;
          // word = level << 9: bits 15..13 zero, 12..9 the level, 8..0 zero
          SM16380_TICK2(0, false);
          SM16380_TICK2(0, false);
          SM16380_TICK2(0, false);
          SM16380_TICK2(lanes(pt, pb, 3), false);
          SM16380_TICK2(lanes(pt, pb, 2), false);
          SM16380_TICK2(lanes(pt, pb, 1), false);
          SM16380_TICK2(lanes(pt, pb, 0), false);
          for (int bit = 8; bit >= 1; bit--) SM16380_TICK2(0, false);
          SM16380_TICK2(0, last);
        }
#undef SM16380_TICK2
    sc = ScanState{row, phase, oeLeft, opener};
    lines = prev = L;
    out->n = (uint16_t *)o - out->buf;
    clocks += ROWS * 16 * chainChips * 16;
  }
};

}  // namespace sm16380
