// Two panels side by side as one 256 x 64 picture: ESP -> left panel ->
// HUB75 cable -> right panel. Each panel shows its own number, so a swapped
// or upside-down layout is easy to spot (see SM16380Layout).
#include <SM16380_DMA.h>

SM16380Panel panel;

// 3 x 5 digits, scaled up
static const uint8_t DIGITS[3][5] = {{2, 6, 2, 2, 7}, {6, 1, 2, 4, 7}, {6, 1, 2, 1, 6}};

static void digit(int d, int x0, int y0, int scale, uint16_t c) {
  for (int r = 0; r < 5; r++)
    for (int col = 0; col < 3; col++)
      if (DIGITS[d][r] >> (2 - col) & 1)
        for (int dy = 0; dy < scale; dy++)
          for (int dx = 0; dx < scale; dx++) panel.setPixel(x0 + col * scale + dx, y0 + r * scale + dy, c);
}

void setup() {
  Serial.begin(115200);
  SM16380Layout layout;
  layout.panelsX = 2;
  layout.first = sm16380::Corner::TopLeft;  // the ESP plugs into the left panel
  if (!panel.begin(SM16380Pins::seengreatS3(), layout)) {
    Serial.println("panel.begin failed (is PSRAM enabled?)");
    return;
  }
  const uint16_t colours[2] = {0x0af, 0xfa0};
  for (int p = 0; p < 2; p++) {
    for (int x = 0; x < 128; x++) panel.setPixel(p * 128 + x, 0, colours[p]);  // top edge: which panel is which
    digit(p, p * 128 + 52, 12, 8, colours[p]);
  }
  panel.show();
}

void loop() { delay(1000); }
