// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 StubbyDevs. SM16380_DMA is free software under the GNU
// GPL v3 or later; see LICENSE. It comes with ABSOLUTELY NO WARRANTY.
// Host checks for the SM16380 stream encoder (test/run.sh).
//
//   1. The DMA word stream gives the same line state as a reference model of
//      the protocol (a direct port of the bit-bang driver this library
//      replaced, which was verified on panels) at every CLK rising edge:
//      config, uploads, scan; held and continuous uploads; several upload speeds.
//   2. A patched upload (pixel bits only) equals a full encode.
//   3. Continuous uploads all start in the same scan state.
//   4. Golden streams: the same stream as the firmware this library was taken
//      from, which runs on real panels (hashes in testGolden). This pins the
//      constants (232 clocks per row, latch counts, register values) that the
//      reference model shares with the encoder.
//   5. Every program the DMA driver builds is whole 64-byte blocks, so the
//      driver's padding never has to add words.
//   6. Chained panels: a shift-register model of the chain, fed the stream,
//      leaves every chip holding the pixels its panel's place in the picture
//      says it should (the expected places are written out by hand).
// The chain direction itself (firstShiftIsFar) can only be checked on a panel.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "../src/sm16380_encoder.h"

using namespace sm16380;

// ---------------------------------------------------------------- reference model
//
// The bit-bang driver: it set lines and toggled CLK. Here each rising edge
// records the line state (CLK bit cleared), which is what the chips sample.
struct Reference {
  const Encoder *layout;  // for the chain mapping and the picture
  std::vector<uint16_t> edges;
  uint16_t lines = 0;
  ScanState sc = {};

  void clk() { edges.push_back(lines); }
  void setRow(int r) { lines = (lines & ~BUS_ADDR) | (r << BUS_ADDR_SHIFT); }

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
    clk();
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

  void clocks(int n) {
    lines &= ~BUS_RGB;
    while (n-- > 0) clk();
  }
  void latches(int n) {
    lines = (lines & ~BUS_RGB) | BUS_LAT;
    while (n-- > 0) clk();
    lines &= ~BUS_LAT;
  }
  void word(uint16_t r, uint16_t g, uint16_t b, int lat) {
    int n = layout->chainChips * 16;
    for (int i = 0; i < n; i++) {
      uint16_t m = 0x8000 >> (i & 15), on = 0;
      if (r & m) on |= 0x09;
      if (g & m) on |= 0x12;
      if (b & m) on |= 0x24;
      if (i == n - lat) lines |= BUS_LAT;
      lines = (lines & ~BUS_RGB) | on;
      clk();
    }
    lines &= ~(BUS_LAT | BUS_RGB);
  }
  void config() {
    lines &= ~(BUS_OE | BUS_LAT | BUS_RGB);
    for (int i = 0; i < 32; i++) {
      lines &= ~BUS_RGB;
      clk();
      latches(3);
      clocks(8);
      latches(14);
      clocks(8);
      const uint16_t w[6][3] = {{0x00AA, 0x00AA, 0x00AA}, {0x01AA, 0x01AA, 0x01AA}, {CFG_R[i], CFG_G[i], CFG_B[i]},
                                {0xF003, 0xF003, 0xF003}, {0x0055, 0x0055, 0x0055}, {0x0155, 0x0155, 0x0155}};
      for (auto &x : w) {
        word(x[0], x[1], x[2], 5);
        clocks(8);
      }
    }
  }
  void upload(bool continuous) {
    lines &= ~(BUS_LAT | BUS_RGB);
    size_t start = edges.size();
    if (!continuous) {
      sc = ScanState{0, 0, 1 << 30, true};
      setRow(0);
    }
    const uint16_t *fb = layout->fb;
    for (int y = 0; y < ROWS; y++)
      for (int slot = 0; slot < 16; slot++)
        for (int c = 0; c < layout->chainChips; c++) {
          int p = layout->chipPanel(c), lx = (c % CHIPS) * 16 + slot;
          uint16_t pt = fb[layout->pixIdx(p, lx, y)], pb = fb[layout->pixIdx(p, lx, y + ROWS)];
          for (int bit = 15; bit >= 0; bit--) {
            uint16_t d = 0;
            if (bit >= 9 && bit <= 12) {
              int lb = bit - 9;
              if (pt & (0x100 << lb)) d |= 1;
              if (pt & (0x010 << lb)) d |= 2;
              if (pt & (0x001 << lb)) d |= 4;
              if (pb & (0x100 << lb)) d |= 8;
              if (pb & (0x010 << lb)) d |= 16;
              if (pb & (0x001 << lb)) d |= 32;
            }
            tick(d, c == layout->chainChips - 1 && bit == 0);
          }
        }
    if (sc.opener) sc.oeLeft = 1;
    tick(0, false);
    lines |= BUS_LAT;
    for (int i = 0; i < 3; i++) tick(0, false);
    lines &= ~BUS_LAT;
    if (continuous)
      while ((edges.size() - start) % SCAN_CLKS) tick(0, false);
  }
  void scanPass() {
    for (int i = 0; i < SCAN_CLKS; i++) tick(0, false);
  }
};

// ---------------------------------------------------------------- helpers

static std::vector<uint16_t> picture(PANEL_W * MAX_PANELS * PANEL_H * MAX_PANELS);
static int fails = 0;
#define CHECK(cond, ...)          \
  do {                            \
    if (!(cond)) {                \
      fails++;                    \
      printf("FAIL: " __VA_ARGS__); \
      printf("\n");               \
    }                             \
  } while (0)

static std::vector<uint16_t> edgesOf(const Stream &s, size_t from = 0) {
  std::vector<uint16_t> e;
  uint16_t prev = from ? s.buf[from - 1] : 0;
  for (size_t i = from; i < s.n; i++) {
    uint16_t w = s.buf[i];
    if (!(prev & BUS_CLK) && (w & BUS_CLK)) e.push_back(w & ~BUS_CLK);
    prev = w;
  }
  return e;
}

static void randomFrame(const Encoder &e) {
  for (int i = 0; i < e.width * e.height; i++) picture[i] = rand() & 0xfff;
}

// ---------------------------------------------------------------- tests

static uint64_t fnv(uint64_t h, uint16_t w) {
  h ^= w;
  return h * 1099511628211ull;
}
static uint64_t hashEdges(const Stream &s, size_t from, size_t to) {
  uint64_t h = 1469598103934665603ull;
  uint16_t prev = from ? s.buf[from - 1] : 0;
  for (size_t i = from; i < to; i++) {
    uint16_t w = s.buf[i];
    if (!(prev & BUS_CLK) && (w & BUS_CLK)) h = fnv(h, w & ~BUS_CLK);
    prev = w;
  }
  return h;
}
static uint64_t hashWords(const Stream &s, size_t from, size_t to) {
  uint64_t h = 1469598103934665603ull;
  for (size_t i = from; i < to; i++) h = fnv(h, s.buf[i]);
  return h;
}

// Config (clock edges: the pause length differs, 2.016 ms here vs 2 ms there),
// a held upload + one scan pass, a continuous upload + one scan pass, word for
// word. Hashes from the firmware's encoder, 2026-10-18.
static void testGolden(int px, uint64_t cfg, uint64_t held, size_t heldN, uint64_t cont, size_t contN) {
  Encoder e;
  e.setLayout(px, 1, Corner::TopLeft, false);
  e.fb = picture.data();
  for (int i = 0; i < e.width * e.height; i++) picture[i] = (i * 2654435761u) >> 20 & 0xfff;
  static std::vector<uint16_t> big(8000000);
  Stream s{big.data(), big.size(), 0};
  e.out = &s;
  e.config();
  size_t a = s.n;
  e.upload(false);
  e.scanPass();
  size_t b = s.n;
  e.upload(true);
  e.scanPass();
  CHECK(hashEdges(s, 0, a) == cfg, "%dx1 golden: config differs from the firmware's", px);
  CHECK(b - a == heldN && hashWords(s, a, b) == held, "%dx1 golden: held upload + scan differs from the firmware's", px);
  CHECK(s.n - b == contN && hashWords(s, b, s.n) == cont, "%dx1 golden: continuous upload + scan differs from the firmware's", px);
  // whole 64-byte blocks (32 words): a continuous upload, a scan pass
  CHECK((s.n - b) % 32 == 0 && (2 * SCAN_CLKS) % 32 == 0, "%dx1: a program is not whole 64-byte blocks", px);
  printf("%dx1: same stream as the firmware (golden hashes)\n", px);
}

static void testAgainstReference() {
  static std::vector<uint16_t> big(8000000);
  for (int cont = 0; cont < 2; cont++)
    for (int k : {2, 3, 4, 8}) {
      Encoder e;
      e.fb = picture.data();
      for (int i = 0; i < PANEL_W * PANEL_H; i++) picture[i] = (i * 2654435761u) >> 20 & 0xfff;
      Stream s{big.data(), big.size(), 0};
      e.out = &s;
      e.upK = k;
      Reference r;
      r.layout = &e;
      // config + held upload (always first), scans, then the mode under test
      r.config(), r.upload(false), r.scanPass(), r.upload(cont), r.scanPass(), r.scanPass(), r.upload(cont), r.scanPass();
      e.config(), e.upload(false), e.scanPass(), e.upload(cont), e.scanPass(), e.scanPass(), e.upload(cont), e.scanPass();
      auto got = edgesOf(s);
      size_t n = std::min(got.size(), r.edges.size()), first = n;
      for (size_t i = 0; i < n; i++)
        if (got[i] != r.edges[i]) {
          first = i;
          break;
        }
      CHECK(got.size() == r.edges.size() && first == n, "%s k=%d: differs at clock %zu (%zu vs %zu clocks)",
            cont ? "continuous" : "held", k, first, got.size(), r.edges.size());
      CHECK(!e.overflow, "overflow");
    }
  printf("stream vs reference: held + continuous, k = 2/3/4/8 checked\n");
}

static void testPatch(Encoder &e) {
  static std::vector<uint16_t> A(3000000), B(3000000);
  e.reset();
  e.upK = 2;
  e.fb = picture.data();
  Stream a{A.data(), A.size(), 0};
  e.out = &a;
  e.scanPass();
  randomFrame(e);
  ScanState s0 = e.sc;
  uint16_t l0 = e.lines;
  a.n = 0;
  e.upload(true);
  ScanState s1 = e.sc;
  uint16_t l1 = e.lines;
  size_t n1 = a.n;
  randomFrame(e);
  e.sc = s0;
  e.lines = e.prev = l0;
  Stream b{B.data(), B.size(), 0};
  e.out = &b;
  e.upload(true);
  CHECK(!memcmp(&e.sc, &s1, sizeof(ScanState)) && e.lines == l1 && b.n == n1,
        "%d panel(s): upload end state depends on the pixels", e.nPanels);
  e.patch(a);
  CHECK(!memcmp(A.data(), B.data(), n1 * 2), "%d panel(s): patched upload differs from a full encode", e.nPanels);
  e.sc = s1;
  e.lines = l1;
  e.scanPass();
  CHECK(!memcmp(&e.sc, &s0, sizeof(ScanState)) && e.lines == l0, "%d panel(s): next upload starts in another scan state",
        e.nPanels);
}

// The chain as wired: data enters panel 0 (the controller's) at its chip for
// x 112..127, runs to its x 0 chip, then on into panel 1 the same way. After
// a latch group the chip at distance d from the controller holds the word
// shifted in (chips - 1 - d)-th. exp[p] = {grid x, grid y, upside down}.
struct Exp {
  int gx, gy;
  bool rot;
};
static void testChain(const char *name, int px, int py, Corner first, bool snake, std::vector<Exp> exp) {
  Encoder e;
  e.setLayout(px, py, first, snake);
  e.fb = picture.data();
  static std::vector<uint16_t> big(4000000);
  Stream s{big.data(), big.size(), 0};
  e.out = &s;
  randomFrame(e);
  e.upload(false);
  auto edges = edgesOf(s);
  int chips = e.chainChips, bad = 0;
  size_t clk = 0;
  for (int g = 0; g < ROWS * 16; g++) {
    std::vector<uint16_t> lanes(chips * 16);
    for (int i = 0; i < chips * 16; i++) lanes[i] = edges[clk + i] & BUS_RGB;
    CHECK(edges[clk + chips * 16 - 1] & BUS_LAT, "%s: no latch at the end of group %d", name, g);
    clk += chips * 16;
    int y = g / 16, slot = g % 16;
    for (int d = 0; d < chips; d++) {  // d: distance from the controller
      int shifted = chips - 1 - d;
      int p = d / CHIPS, lx = 112 - 16 * (d % CHIPS) + slot;
      for (int half = 0; half < 2; half++) {
        int ly = y + half * ROWS;
        int ex = exp[p].rot ? PANEL_W - 1 - lx : lx, ey = exp[p].rot ? PANEL_H - 1 - ly : ly;
        uint16_t px_ = picture[(exp[p].gy * PANEL_H + ey) * e.width + exp[p].gx * PANEL_W + ex];
        for (int lb = 0; lb < 4; lb++) {
          uint16_t lane = lanes[shifted * 16 + (15 - (9 + lb))];  // MSB first: bit b at clock 15 - b
          int want = ((px_ >> (8 + lb)) & 1) | (((px_ >> (4 + lb)) & 1) << 1) | (((px_ >> lb) & 1) << 2);
          int got = half ? (lane >> 3) & 7 : lane & 7;
          if (want != got) bad++;
        }
      }
    }
  }
  CHECK(bad == 0, "%s: %d pixel bits on the wrong chip", name, bad);
  printf("%s: %d panels, every chip's pixels checked\n", name, e.nPanels);
  testPatch(e);
}

int main() {
  testAgainstReference();
  testGolden(1, 0xf210d1b393e32b63ull, 0xa0c30a5553c41799ull, 145928, 0x11a56127c8d49e79ull, 148480);
  testGolden(2, 0xd98c5d7e9a1713c3ull, 0x52fe95301f27ec01ull, 277000, 0xf500e862d95510e1ull, 282112);
  Encoder one;
  testPatch(one);
  printf("1 panel: patch and continuous start state checked\n");
  testChain("2x1, controller on the left", 2, 1, Corner::TopLeft, false, {{0, 0, false}, {1, 0, false}});
  testChain("2x1, controller on the right", 2, 1, Corner::TopRight, false, {{1, 0, false}, {0, 0, false}});
  testChain("1x2 stacked, controller on top", 1, 2, Corner::TopLeft, false, {{0, 0, false}, {0, 1, false}});
  testChain("1x2 snake, controller on top", 1, 2, Corner::TopLeft, true, {{0, 0, false}, {0, 1, true}});
  testChain("3x1, controller on the left", 3, 1, Corner::TopLeft, false, {{0, 0, false}, {1, 0, false}, {2, 0, false}});
  printf(fails ? "%d FAILED\n" : "all passed\n", fails);
  return fails != 0;
}
