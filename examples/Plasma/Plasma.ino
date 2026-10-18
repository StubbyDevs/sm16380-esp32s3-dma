// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 StubbyDevs. SM16380_DMA is free software under the GNU
// GPL v3 or later; see LICENSE. It comes with ABSOLUTELY NO WARRANTY.
// A full-screen animation as fast as the encoder goes; prints the frame rate.
#include <SM16380_DMA.h>

SM16380Panel panel;
static uint8_t sine[256];

void setup() {
  Serial.begin(115200);
  // panel.setBusDivider(12);  // ~20% faster (tested); the default is 16
  if (!panel.begin(SM16380Pins::seengreatS3())) {
    Serial.println("panel.begin failed (is PSRAM enabled?)");
    while (true) delay(1000);
  }
  for (int i = 0; i < 256; i++) sine[i] = 128 + 127 * sinf(i * 2 * PI / 256);
}

void loop() {
  static uint32_t t = 0, frames = 0, t0 = millis();
  int w = panel.width(), h = panel.height();
  uint16_t *fb = panel.buffer();
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++) {
      uint8_t v = sine[(x * 3 + t) & 255] / 2 + sine[(y * 5 + t * 2) & 255] / 2;
      uint8_t r = sine[(v + t) & 255], g = sine[(v + 85) & 255], b = sine[(v + 170 - t) & 255];
      fb[y * w + x] = SM16380Panel::color(r, g, b);
    }
  panel.show();
  t++;
  if (++frames == 200) {
    SM16380Stats s = panel.stats();
    Serial.printf("%.1f fps (encode %lu us, %lu of %lu frames patched)\n", frames * 1000.0f / (millis() - t0), (unsigned long)s.encodeUs,
                  (unsigned long)s.patched, (unsigned long)s.uploads);
    frames = 0;
    t0 = millis();
  }
}
