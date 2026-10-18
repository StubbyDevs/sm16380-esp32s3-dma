// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 StubbyDevs. SM16380_DMA is free software under the GNU
// GPL v3 or later; see LICENSE. It comes with ABSOLUTELY NO WARRANTY.
// SM16380_DMA: an ESP32-S3 driver for HUB75 LED panels with SM16380 S-PWM
// column drivers (128 x 64, 1/32 scan, binary A-E row decoder such as the
// SM5166).
//
// The S3's LCD_CAM peripheral (i80, 16-bit) and GDMA stream every panel line,
// the row scan included. It loops in hardware with no CPU time, and each new
// frame is spliced in at the end of a scan pass, so updates don't blink or tear.
//
//   #include <SM16380_DMA.h>
//   SM16380Panel panel;
//   void setup() {
//     panel.begin(SM16380Pins::seengreatS3());
//     panel.fill(SM16380Panel::color(255, 0, 0));
//     panel.show();
//   }
//
// Needs an ESP32-S3 with PSRAM (e.g. N16R8, N8R8).
#pragma once

#include <Arduino.h>

#include "sm16380_encoder.h"

#if !CONFIG_IDF_TARGET_ESP32S3
#error "SM16380_DMA needs an ESP32-S3 (LCD_CAM + GDMA)"
#endif

struct SM16380Pins {
  int8_t r1, g1, b1, r2, g2, b2;
  int8_t a, b, c, d, e;
  int8_t lat, oe, clk;

  // Seengreat RGB Matrix Adapter Board (E) V2.x, ESP32-S3-DevKitC-1 socket.
  static SM16380Pins seengreatS3() { return {18, 8, 17, 16, 1, 15, 7, 48, 6, 47, 2, 21, 4, 5}; }
};

struct SM16380Layout {
  uint8_t panelsX = 1, panelsY = 1;  // at most 3 panels in all
  sm16380::Corner first = sm16380::Corner::TopLeft;  // the panel the ESP plugs into
  bool snake = false;  // every other row of panels runs back, mounted upside down
  // Order chips leave the chain in. true for every panel tested so far; flip
  // it if a 2-panel picture shows up with its halves swapped.
  bool firstShiftIsFar = true;
};

struct SM16380Stats {
  uint32_t uploads, patched, restarts, timeouts, overflows;
  uint32_t encodeUs;  // last show(): encode time
  uint32_t switchUs;  // last show(): wait for the previous frame to start
  uint32_t programBytes;
};

class SM16380Panel {
 public:
  // Allocates the buffers (frame and programs in PSRAM, scan chains in
  // internal RAM) and starts the LCD_CAM. The panel shows black until show().
  // False (and nothing allocated) if out of memory, no GDMA channel is free,
  // the layout has more than 3 panels, or it was already called.
  bool begin(const SM16380Pins &pins, const SM16380Layout &layout = SM16380Layout());

  int width() const { return enc_.width; }
  int height() const { return enc_.height; }

  // The frame to draw into: width() x height() pixels, RGB444 as 0x0RGB,
  // row-major. show() sends it; you can draw the next one as soon as show() returns.
  uint16_t *buffer() { return fb_; }

  static uint16_t color(uint8_t r, uint8_t g, uint8_t b) { return (r >> 4) << 8 | (g >> 4) << 4 | (b >> 4); }
  void setPixel(int x, int y, uint16_t rgb444) {
    if ((unsigned)x < (unsigned)enc_.width && (unsigned)y < (unsigned)enc_.height && fb_) fb_[y * enc_.width + x] = rgb444;
  }
  uint16_t getPixel(int x, int y) const {
    return (unsigned)x < (unsigned)enc_.width && (unsigned)y < (unsigned)enc_.height && fb_ ? fb_[y * enc_.width + x] : 0;
  }
  void fill(uint16_t rgb444);
  void clear() { fill(0); }

  // Encode the frame and splice it in after the scan pass now running.
  // Returns once it is encoded and queued; it waits for the previous frame
  // to have started, so at most one frame is ever waiting. When only the
  // pixels changed (the usual case) just their bits are rewritten.
  void show();

  // Bus speed, before begin(): 160 MHz / div words per second, the panel
  // clock is half that. 16 (5 MHz, the default) and 12 (6.7 MHz, ~20% more
  // frames per second) are tested on panels. Faster is refused: from 10 on,
  // the DMA can't read frames out of PSRAM fast enough and stalls. False if
  // out of range or begin() already ran.
  bool setBusDivider(int div);
  int busDivider() const { return enc_.busDiv; }

  // Seamless (default on): keep scanning while a frame uploads, so the panel
  // never goes dark. Off: hold OE through each upload, which blanks the
  // panel for the upload (~13 ms per panel) but also re-syncs its row count.
  void setSeamless(bool on) { seamless_ = on; }

  // The chip config lives in panel RAM and is lost when the panel's 5 V
  // drops, so it's rewritten on the first show() and then every interval
  // (default 30 min; 0 = never). requestConfig() rewrites it on the next show().
  void setConfigInterval(uint32_t ms) { configMs_ = ms; }
  void requestConfig() { configDue_ = true; }

  // Flash writes (NVS, OTA, SPIFFS) block PSRAM access, and a frame streaming
  // from PSRAM would then stall the DMA. Bracket them with these: the call
  // waits until only the scan (in internal RAM) is running, and holds
  // show() until the release. Safe from another task.
  void flashHold();
  void flashRelease();

  SM16380Stats stats() const { return stats_; }

 private:
  struct Chain {
    sm16380::Stream s;
    void *desc;  // dma_descriptor_t[]
    int maxDesc, nDesc;
    bool psram;
  };
  struct ProgKey {
    sm16380::ScanState sc;
    uint16_t lines, prev;
    bool continuous;
  };

  static const int PAUSE_WORDS = 2016;  // one 4032-byte descriptor of idle lines

  int pauseDescs() const;
  bool alloc(Chain &c, size_t words, bool psram, int maxDesc = 0);
  void freeAll();
  void buildConfig();
  void finish(Chain &c);
  void *firstDesc(Chain &c);
  void link(Chain &from, void *to);
  void restart();
  bool waitSwitch();
  bool showLocked(bool config);

  sm16380::Encoder enc_;
  uint16_t *fb_ = nullptr, *pause_ = nullptr;
  Chain scan_[2] = {}, prog_[2] = {}, cfg_ = {};
  int progIdx_ = 0, activeScan_ = 0, pendingScan_ = 0;
  uint32_t pendingTarget_ = 0;
  bool running_ = false, needResync_ = false;
  ProgKey progKey_[2] = {};
  bool progReusable_[2] = {};
  sm16380::ScanState progEndSc_[2] = {};
  uint16_t progEndLines_[2] = {};
  bool scanCached_[2] = {};
  sm16380::ScanState scanStartSc_[2] = {}, scanEndSc_[2] = {};
  uint16_t scanStartLines_[2] = {}, scanEndLines_[2] = {};
  void *dmaChan_ = nullptr;
  int dmaCh_ = 0;
  void *lock_ = nullptr;  // SemaphoreHandle_t
  bool seamless_ = true, configDue_ = true;
  uint32_t configMs_ = 30 * 60 * 1000, lastConfig_ = 0;
  SM16380Stats stats_ = {};
};
