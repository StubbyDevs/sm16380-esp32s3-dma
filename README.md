# SM16380_DMA

A DMA driver for HUB75 LED panels with **SM16380 S-PWM** column drivers, for the **ESP32-S3**.
The row scan loops in hardware with no CPU time, and new frames go in at the end of a scan pass.
Updates don't blink or tear, gradients are smooth, and a 128x64 panel animates at ~67 fps.

S-PWM panels (SM16380, ICN2153, FM6353, ...) look much better than the usual HUB75
shift-register panels: smooth low levels, no scan lines on camera, high refresh. But they aren't
shift registers. Each chip has its own frame memory and makes the PWM itself, and has to be
configured, uploaded to and scanned with its own protocol. So the popular HUB75 libraries don't
drive them. This library does, for the SM16380, on a $10 ESP32-S3.

## Features

- **No blink on update ("seamless").** The scan keeps running through each upload, and every upload
  is padded to whole scan passes, so the chip's row count never drifts (see [the protocol notes](extras/PROTOCOL.md)).
- **Hardware scan.** LCD_CAM (i80, 16-bit) + GDMA stream every line, the panel clock included. The scan
  is a circular DMA chain in internal RAM: no interrupts or timing jitter, and it keeps running while
  WiFi and TLS run.
- **Fast updates.** Two program buffers are pipelined. When only the pixels changed, just the pixel bits
  are re-encoded. The scan chains are cached.
- **Up to 3 chained panels** as one picture (e.g. 256x64 or 128x192), side by side, stacked or snake-wired.
- **Self-healing.** A DMA stall is detected, and the driver restarts and re-syncs the panel. The chip config
  is rewritten periodically (it lives in panel RAM and is lost on a 5 V blip).
- **Safe flash writes.** `flashHold()` / `flashRelease()` keep the DMA off PSRAM while NVS/OTA writes run.
- Optional **Adafruit_GFX** front end (text, shapes, bitmaps).
- An encoder test suite that runs on any PC.

## Hardware

Tested on:

| | |
|---|---|
| Panel | 128x64 P2.5 (320x160 mm), 1/32 scan, HUB75E. **SM16380SA** column drivers, **SM5166PF** row decoder |
| Controller | ESP32-S3-DevKitC-1 **N16R8** (any S3 with PSRAM should work) |
| Adapter | Seengreat RGB Matrix Adapter Board (E) V2.x, S3 socket (74HCT245 level shifters) |
| Chains | 1 and 2 panels on the glass. 3 panels: tested in simulation |

Requirements and limits:

- **ESP32-S3 with PSRAM** (Arduino: *Tools > PSRAM > OPI PSRAM* for N16R8/N8R8).
- Panel: **128x64, 1/32 scan, binary A-E row addressing** (SM5166 or similar decoder), SM16380 (SA/SH) columns.
  Other sizes, scan rates or shift-register row drivers need encoder changes ([contributions welcome](#contributing)).
- Use a 5 V level shifter (the adapter above has one). On 1.8 V-IO modules (e.g. N32R16**V**), GPIO47/48 are 1.8 V.
  Keep row lines off them, or use an N16R8.
- 14 GPIOs, any that the GPIO matrix can route. Avoid 35-37 on octal-PSRAM modules (used by the PSRAM) and 19/20 (USB).

### Pins

`SM16380Pins::seengreatS3()` is the Seengreat adapter's S3 socket:

| R1 | G1 | B1 | R2 | G2 | B2 | A | B | C | D | E | LAT | OE | CLK |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 18 | 8 | 17 | 16 | 1 | 15 | 7 | 48 | 6 | 47 | 2 | 21 | 4 | 5 |

For your own wiring: `SM16380Pins pins = {r1, g1, b1, r2, g2, b2, a, b, c, d, e, lat, oe, clk};`

## Install

- **Arduino IDE / arduino-cli:** copy this folder into your `libraries/` folder, or *Sketch > Include Library > Add .ZIP Library*.
- **PlatformIO:** `lib_deps = https://github.com/StubbyDevs/sm16380-esp32s3-dma.git`, plus
  `board_build.arduino.memory_type = qio_opi` (N16R8) and `build_flags = -DBOARD_HAS_PSRAM`.

Built against arduino-esp32 **3.3.x** (ESP-IDF 5.5). It uses the IDF's private GDMA API (`esp_private/gdma.h`),
so other core versions may need small changes.

## Quick start

```cpp
#include <SM16380_DMA.h>

SM16380Panel panel;

void setup() {
  panel.begin(SM16380Pins::seengreatS3());
  panel.fill(SM16380Panel::color(0, 0, 255));
  panel.setPixel(10, 10, 0xfff);   // RGB444: 0x0RGB
  panel.show();
}

void loop() {}
```

Two panels as one 256x64 picture:

```cpp
SM16380Layout layout;
layout.panelsX = 2;                          // side by side
layout.first = sm16380::Corner::TopLeft;     // the ESP plugs into the left panel
panel.begin(SM16380Pins::seengreatS3(), layout);
```

Examples: `Basic` (colour bars and gradients), `Plasma` (full-speed animation, prints fps), `TwoPanels` (layout check),
`GFXText` (scrolling text with Adafruit_GFX).

## API

| | |
|---|---|
| `bool begin(pins, layout = {})` | Allocate buffers and start the LCD_CAM. False if out of memory (no PSRAM?) |
| `int width()`, `int height()` | The whole picture (e.g. 256 x 64 for 2x1) |
| `uint16_t *buffer()` | The frame, RGB444 `0x0RGB`, row-major. Draw here, then `show()` |
| `setPixel(x, y, c)`, `getPixel(x, y)`, `fill(c)`, `clear()` | Drawing helpers |
| `static color(r, g, b)` | 8-bit RGB to RGB444 |
| `show()` | Encode the frame and queue it for the end of the current scan pass. You can draw the next frame as soon as it returns |
| `setSeamless(bool)` | Default on. Off: OE is held through each upload (the panel blanks ~13 ms per panel) |
| `setConfigInterval(ms)`, `requestConfig()` | Chip config rewrite: first `show()`, then every 30 min by default |
| `flashHold()`, `flashRelease()` | Bracket flash writes (NVS, OTA, file systems). Safe from any task |
| `stats()` | Uploads, patched frames, restarts, timeouts, encode time |

`SM16380_GFX` (include `SM16380_GFX.h`) is an `Adafruit_GFX` with `begin()`, `show()` and the panel at `.panel`.

**Colour depth:** 4 bits per channel (16 levels). The chip's S-PWM spreads the levels over many scan passes,
so even level 1 is smooth.

## Performance

At the default bus rate (10 M words/s; one panel clock = 2 words while scanning and uploading):

| | 1 panel | 2 panels | 3 panels |
|---|---|---|---|
| Upload (one frame) | ~13.4 ms | ~26.7 ms | ~40 ms |
| Max frame rate | **~67 fps** | ~35 fps | ~24 fps |
| Scan pass / row refresh | 1.49 ms (~670 Hz) | same | same |
| PSRAM used | ~2.6 MB | ~3.7 MB | ~4.8 MB |
| Internal RAM | ~60 KB (two scan chains) + descriptors | same | same |

Frame rate is limited by the upload, not the CPU: encoding a frame where only pixels changed takes a few ms.
Most of the PSRAM is the config program, which is long because every register goes through the whole chain
at a slow clock.

## How it works

The ESP32-S3's LCD_CAM peripheral in i80 mode sends one 16-bit word per bus cycle, from GDMA. The 14 panel
lines (6 colour, 5 address, LAT, OE, CLK) are bits of that word. CLK is a **data bit**, not the LCD's WR strobe,
so the stream controls the shape of every panel clock: slow clocks for config, two-word clocks for data, the
row address changing 28 clocks before each OE pulse.

```
             ┌──────────── scan chain A (internal RAM, loops) ◄─┐
GDMA ──► LCD_CAM ──► 14 GPIOs ──► panel                         │
             └─ at the end of a pass: program (PSRAM) ──► scan chain B (loops)
                 [config] + upload + VSYNC
```

A new frame is encoded into a **program** in PSRAM whose last descriptor points at a fresh scan chain. It's
spliced in by pointing the running scan chain's last descriptor at the program, so it starts exactly at the end
of a pass. The two scan chains take turns, and so do the two program buffers: frame N+1 is encoded while
frame N is still streaming.

The details that took the longest to find (the chip counts OE pulses to track its row, why seamless uploads
drift, config grammar and timing) are written up in [extras/PROTOCOL.md](extras/PROTOCOL.md).

## Gotchas

- **Flash writes stall the DMA** if a program is streaming from PSRAM at the time. Wrap `Preferences`/NVS
  writes, OTA writes and file-system writes in `flashHold()` / `flashRelease()`.
- **Two panels show swapped halves?** Set `layout.firstShiftIsFar = false`. Or rather, check `layout.first`
  first: it should be the corner panel your ESP plugs into.
- **The panel shows garbage after its 5 V supply blips.** The chip config is lost; it comes back at the next
  config rewrite. Call `requestConfig()` if you can detect the blip, or shorten `setConfigInterval()`.
- Call `show()` from one task. `flashHold()` may be called from any task.

## Testing

```
./test/run.sh
```

This compiles the encoder (`src/sm16380_encoder.h`, hardware-free) on your PC and checks:
- the DMA word stream against a reference model of the protocol at every clock edge
  (config, held and continuous uploads, several upload speeds);
- that patched uploads equal full encodes;
- that continuous uploads keep the scan state;
- that in 2x1, 1x2, snake and 3x1 chains every chip ends up holding the right pixels.

CI runs these tests and compiles the examples.

## Contributing

Wanted, with a panel to test on:

- Other S-PWM drivers: **ICN2153**, ICN2055/ICND2065, FM6353/6363/6373, DP3264, MBI5264. Each needs its own config
  and upload grammar, but the DMA machinery is the same.
- Other geometries: 1/16 and 1/40+ scan, and shift-register row drivers (ICN2013, DP32020, SM5368), as on 160x80
  and 172x86 panels.
- More colour depth (the SM16380 takes up to 13 bits per channel).

Please include photos of the panel's chip markings, and run `./test/run.sh` with your change.

## Acknowledgements

The protocol was worked out on the glass, with these as references:
- [kingdo9/rpi-rgb-led-matrix_pwm_experiment](https://github.com/kingdo9/rpi-rgb-led-matrix_pwm_experiment): the SM16380SH register profiles (`regtype2` values used here).
- [FalconChristmas/fpp](https://github.com/FalconChristmas/fpp) `BBShiftPanel`: the SM16380 config command grammar.
- [board707/DMD_STM32](https://github.com/board707/DMD_STM32) `DMD_RGB_SM16380SH`: an STM32/RP2040 driver for the same chip.
- [mrcodetastic/ESP32-HUB75-MatrixPanel-DMA](https://github.com/mrcodetastic/ESP32-HUB75-MatrixPanel-DMA): the reference for LCD_CAM + GDMA on the S3.

## License

MIT, see [LICENSE](LICENSE).
