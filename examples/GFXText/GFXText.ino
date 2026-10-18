// Scrolling text with Adafruit_GFX (install "Adafruit GFX Library").
#include <SM16380_GFX.h>

SM16380_GFX gfx;
static const char *MSG = "Smooth S-PWM scrolling on an ESP32-S3";

void setup() {
  Serial.begin(115200);
  if (!gfx.begin(SM16380Pins::seengreatS3())) {
    Serial.println("panel.begin failed (is PSRAM enabled?)");
    while (true) delay(1000);
  }
  gfx.setTextWrap(false);
  gfx.setTextSize(2);
}

void loop() {
  static int x = gfx.width();
  int len = strlen(MSG) * 12;
  gfx.fillScreen(0);
  gfx.setTextColor(gfx.color565(255, 180, 0));
  gfx.setCursor(x, 24);
  gfx.print(MSG);
  gfx.drawFastHLine(0, 0, gfx.width(), gfx.color565(0, 80, 255));
  gfx.drawFastHLine(0, gfx.height() - 1, gfx.width(), gfx.color565(0, 80, 255));
  gfx.show();
  if (--x < -len) x = gfx.width();
  delay(16);
}
