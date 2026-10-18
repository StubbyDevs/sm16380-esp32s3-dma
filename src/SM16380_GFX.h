// Optional Adafruit_GFX front end (text, lines, shapes, bitmaps). Needs the
// "Adafruit GFX Library"; include this header instead of SM16380_DMA.h.
//
//   SM16380_GFX gfx;
//   gfx.begin(SM16380Pins::seengreatS3());
//   gfx.setTextColor(gfx.color565(255, 200, 0));
//   gfx.print("Hello");
//   gfx.show();
#pragma once

#include <Adafruit_GFX.h>

#include "SM16380_DMA.h"

class SM16380_GFX : public Adafruit_GFX {
 public:
  SM16380_GFX() : Adafruit_GFX(sm16380::PANEL_W, sm16380::PANEL_H) {}

  bool begin(const SM16380Pins &pins, const SM16380Layout &layout = SM16380Layout()) {
    if (!panel.begin(pins, layout)) return false;
    WIDTH = _width = panel.width();
    HEIGHT = _height = panel.height();
    return true;
  }

  // RGB565 in (what Adafruit_GFX uses), RGB444 on the panel.
  void drawPixel(int16_t x, int16_t y, uint16_t c) override {
    switch (getRotation()) {
      case 1: { int16_t t = x; x = WIDTH - 1 - y; y = t; break; }
      case 2: x = WIDTH - 1 - x; y = HEIGHT - 1 - y; break;
      case 3: { int16_t t = x; x = y; y = HEIGHT - 1 - t; break; }
    }
    panel.setPixel(x, y, to444(c));
  }
  void fillScreen(uint16_t c) override { panel.fill(to444(c)); }

  void show() { panel.show(); }
  static uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return (r & 0xf8) << 8 | (g & 0xfc) << 3 | b >> 3; }
  static uint16_t to444(uint16_t c) { return (c >> 12) << 8 | ((c >> 7) & 0xf) << 4 | ((c >> 1) & 0xf); }

  SM16380Panel panel;
};
