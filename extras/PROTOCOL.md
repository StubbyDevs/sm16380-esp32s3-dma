# Driving the SM16380: protocol notes

Everything here was found or confirmed on a real panel: 128x64 P2.5, 1/32 scan, SM16380SA column drivers
(8 per colour lane), SM5166PF row decoder, HUB75E. The SM16380SH appears to behave the same (it is what the
references document).

## The model

The SM16380 is **not a shift register**. Each chip has its own frame SRAM and generates the PWM itself. The
controller:

1. writes the **config** registers once (and again whenever panel power drops);
2. **uploads** a frame into the chips' SRAM;
3. keeps **scanning**: it clocks CLK continuously (CLK is the chip's PWM clock) and walks the row
   address, pulsing OE once per row.

The chip spreads each pixel's level over many scan passes ("scrambled" PWM), which is why low levels look
smooth and why the panel looks flicker-free on camera.

## Lines

Standard HUB75E: R1 G1 B1 (top half), R2 G2 B2 (bottom half), A-E (row address, binary, 32 rows), LAT, OE, CLK.
Unlike a plain HUB75 panel, OE is not a blanking line here: the chip **counts OE pulses** (see below).

## 1. Config

32 registers, written one at a time. Each register write is:

```
VSYNC          1 clock, then LAT high for 3 clocks
8 clocks
LAT high for 14 clocks
8 clocks
six words, each followed by 8 clocks:
  00AA, 01AA, <register>, F003, 0055, 0155
```

A "word" goes to **every chip in the chain** (16 clocks x chips, MSB first), with LAT high over its **last 5
clocks**. R, G and B lanes each get their own register value (the R/G/B tables differ in a few registers),
and the top and bottom lanes get the same value.

Clock period: ~1.2 us worked well (it's slow, but config happens rarely). After each register, there's a ~2 ms
pause with the clock stopped. A config rewrite flashes the whole screen: the scan stops for the ~100 ms the
config takes, and the held-OE upload after it blanks the panel again. Holding OE high through the config
works (the chip still takes it) but doesn't remove the flash. An untried idea: scan during the pauses
between registers instead of stopping the clock.

Register values: kingdo9's `sm16380sh.profiles`, **regtype2** (`src/sm16380_encoder.h`, `CFG_R/G/B`). The
first register is `0x0200 | (scan lines - 1)`: `0x021f` for 1/32 scan.

The config lives in panel RAM: **a 5 V blip loses it** and the panel shows garbage until it's rewritten.

## 2. Upload

For each of 32 rows, for each of 16 slots (column within a chip), for each chip in the chain: a 16-bit word,
MSB first. **LAT on the last bit of the last chip** of each (row, slot) group. Then VSYNC (LAT high 3 clocks).

Pixel word: `level << 9`, a 4-bit level in bits 12..9 and everything else 0. The chips accept more bits;
4 is what the tested profile uses.

Column order: the first chip shifted out ends up in the chip **farthest** from the input (plain shift-register
order). Within a panel the input enters at the chip for x = 112..127, so the first chip's data lands on
x = 0..15 of the farthest panel.

Data, LAT and OE should change mid low-phase of CLK, away from both edges. One chip on the test panel
(columns 96-111) dropped bits when data changed right at the falling edge.

## 3. Scan

- CLK never stops. **232 clocks per row.**
- One **4-clock OE pulse** at the start of each row.
- The row address (A-E) changes **28 clocks before** that row's OE pulse.
- A full pass: 32 x 232 = 7,424 clocks.

With interrupts able to pause a bit-banged clock mid-row, that row stays lit longer and shows as a faint
bright line rolling down the panel. A DMA stream avoids this entirely.

## The rule that matters: the chip counts OE pulses

The SM16380 tracks **which row is being displayed by counting OE pulses**, not by reading A-E at the right
moment. Only a **long held OE** (as at the start of an upload with OE held high) resets the count.

That's fine if every upload holds OE (the panel blanks for the length of the upload, ~13 ms per panel at
5 MHz). But a "seamless" upload, one that keeps scanning while data goes in, has to keep the count and
the address in step:

| Attempt | What you see |
|---|---|
| Scan during upload, restart the scan after it with a short opener pulse | Every other frame wrong; colour shifts |
| Same, opener tuned | Wrong areas **creep down the panel**, a few rows per frame, then wrap |
| Scan straight through the upload (no opener, no reset), pad the upload to a **whole number of scan passes** | Correct, every frame, no blink |

The last one is what this library does (`continuous` mode). The upload's clocks are also scan clocks, so the
OE pulses keep coming every 232 clocks. Padding to whole passes means every frame starts and ends in the
same scan state. A held-OE upload is still used after config and after a DMA restart, to re-sync the count.

## Timing used by this library

| | |
|---|---|
| Bus | 10 M words/s (160 MHz / 16), 1 word = all 14 lines |
| Scan clock | 2 words per clock = 5 MHz panel clock |
| Upload clock | 2 words per clock (configurable 2-8) |
| Config clock | 12 words per clock = 1.2 us |
| Scan pass | 14,848 words = 1.485 ms |
| One-panel upload | 65,536 clocks + padding = 9 scan passes = 13.4 ms |
| Frame period (back to back) | upload + at least 1 scan pass = 14.9 ms, ~67 fps |

## Things that didn't matter / didn't work

- No row rotation is needed: SRAM row r shows on address r when the first OE pulse after the opener goes to row 1.
- Uploading 2x faster than the first working bit-bang clock was the sweet spot for bit-banging (1x and 4x both
  dropped more bits on the marginal chip). With DMA's clean edges, 5 MHz is fine.
- Levels 1-3 flickered when bit-banged (slower, jittery scan); with the DMA scan they're steady.

## References

- kingdo9/rpi-rgb-led-matrix_pwm_experiment: SM16380SH profiles, documented at 60 fps on a Pi.
- FalconChristmas/fpp `BBShiftPanel`: the register write grammar.
- board707/DMD_STM32 `DMD_RGB_SM16380SH`: STM32 and RP2040 implementation.
