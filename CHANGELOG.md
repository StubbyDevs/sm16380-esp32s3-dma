# Changelog

## 0.1.0 (2026-10-18)

First release, tested on panels: 128x64 P2.5 SM16380SA + SM5166PF, one and two chained, on an
ESP32-S3-DevKitC-1 N16R8 with the Seengreat adapter V2.x, arduino-esp32 3.3.8.

- LCD_CAM + GDMA hardware scan; frames spliced in at the end of a scan pass (no tearing).
- Seamless (continuous) uploads, padded to whole scan passes: no blink on update.
- Pixel-only patching of unchanged programs; cached scan chains.
- Up to 3 chained panels (side by side, stacked, snake).
- `setBusDivider()`: 16 (default) or 12, measured 67.4 / 80.0 fps on one panel, 35.5 / 43.4 fps on two.
  Below 12 is refused (the DMA can't stream from PSRAM fast enough and stalls).
- Stall detection with restart and re-sync; periodic config rewrite; `flashHold()` / `flashRelease()`.
- A failed `begin()` frees its memory and leaves the object inert; layouts over 3 panels are refused.
- Config pauses served from one shared buffer (~1.3 MB PSRAM for one panel).
- Adafruit_GFX front end; examples Basic, Plasma, TwoPanels, GFXText.
- Host tests: stream vs a reference model, golden hashes, patching, chain mapping.
