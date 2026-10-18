# Changelog

## Unreleased

- `setBusDivider()`: 12 (6.7 MHz panel clock) tested on panels, +19% fps (one panel 80 fps, two 42).
  Below 12 is refused: the DMA can't stream from PSRAM that fast and stalls.
- Config pauses stay at least 2 ms at any bus rate.
- Review fixes: failed begin() is inert and frees memory; layouts over 3 panels refused; config only
  marked sent once it went out; config pauses served from one shared buffer (~1.3 MB less PSRAM);
  golden-stream tests against the firmware the driver came from.

## 0.1.0

First release. SM16380 128x64 1/32-scan panels on the ESP32-S3: LCD_CAM + GDMA hardware scan, seamless
(continuous) uploads, pixel-only patching, up to 3 chained panels, stall recovery, periodic config rewrite,
flash-write hold, Adafruit_GFX front end, host encoder tests.
