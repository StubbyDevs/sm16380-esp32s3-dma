// Colour bars and 16-step gradients: what an S-PWM panel is good at.
#include <SM16380_DMA.h>

SM16380Panel panel;

void setup() {
  Serial.begin(115200);
  if (!panel.begin(SM16380Pins::seengreatS3())) {
    Serial.println("panel.begin failed (is PSRAM enabled?)");
    return;
  }
  int w = panel.width(), h = panel.height();
  const uint16_t bars[8] = {0xfff, 0xff0, 0x0ff, 0x0f0, 0xf0f, 0xf00, 0x00f, 0x000};
  for (int y = 0; y < h / 2; y++)
    for (int x = 0; x < w; x++) panel.setPixel(x, y, bars[x * 8 / w]);
  // red, green, blue and white ramps over the 16 levels
  for (int y = h / 2; y < h; y++) {
    int band = (y - h / 2) * 4 / (h / 2);
    for (int x = 0; x < w; x++) {
      uint16_t v = x * 16 / w;
      const uint16_t c[4] = {(uint16_t)(v << 8), (uint16_t)(v << 4), v, (uint16_t)(v << 8 | v << 4 | v)};
      panel.setPixel(x, y, c[band]);
    }
  }
  panel.show();
}

void loop() {
  // Nothing to do: the scan runs in hardware. Re-sending the frame now and
  // then lets the periodic config rewrite happen (a panel power blip loses it).
  delay(60000);
  panel.show();
}
