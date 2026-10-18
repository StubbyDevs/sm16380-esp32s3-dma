// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 StubbyDevs. SM16380_DMA is free software under the GNU
// GPL v3 or later; see LICENSE. It comes with ABSOLUTELY NO WARRANTY.
// LCD_CAM + GDMA streaming for SM16380_DMA.h.
//
// LCD_CAM in i80 mode streams 16-bit words from GDMA, one every 100 ns
// (160 MHz / busDiv). The panel CLK is bus bit 13, not the LCD's own WR clock,
// so the stream sets the shape of every panel clock (sm16380_encoder.h).
//
// The scan pass (32 rows x 232 clocks) is a circular descriptor chain in
// internal RAM that loops in hardware. A new frame is a program in PSRAM:
// (config,) upload, VSYNC; its last descriptor points at a fresh scan chain.
// It is spliced in by pointing the running chain's last descriptor at it, so
// it always starts at the end of a pass, and the old chain is never entered
// again. Two scan chains and two program buffers take turns.

#include "SM16380_DMA.h"

#include <esp_cache.h>
#include <esp_heap_caps.h>
#include <esp_private/gdma.h>
#include <esp_private/periph_ctrl.h>
#include <esp_rom_gpio.h>
#include <freertos/semphr.h>
#include <hal/dma_types.h>
#include <hal/gdma_ll.h>
#include <soc/gdma_struct.h>
#include <soc/gpio_sig_map.h>
#include <soc/lcd_cam_struct.h>

using namespace sm16380;

static const size_t DESC_BYTES = 4032;  // 63 x 64 bytes, under the 4095 limit
#define DESC(c) ((dma_descriptor_t *)(c).desc)
#define CHAN ((gdma_channel_handle_t)dmaChan_)

bool SM16380Panel::alloc(Chain &c, size_t words, bool psram, int maxDesc) {
  c.s.cap = words;
  c.psram = psram;
  size_t bytes = (words + 32) * 2;
  c.s.buf = (uint16_t *)heap_caps_aligned_alloc(64, bytes, psram ? MALLOC_CAP_SPIRAM : (MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  c.maxDesc = maxDesc ? maxDesc : bytes / DESC_BYTES + 2;
  c.desc = heap_caps_aligned_alloc(8, c.maxDesc * sizeof(dma_descriptor_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  c.s.n = 0;
  return c.s.buf && c.desc;
}

// Pad to a 64-byte multiple (stretching the last clock phase), cut into
// descriptors, push the words out of the cache. The caller links the last one.
void SM16380Panel::finish(Chain &c) {
  sm16380::Stream &s = c.s;
  while (s.n % 32) {
    s.buf[s.n] = s.buf[s.n - 1];
    s.n++;
  }
  size_t bytes = s.n * 2, off = 0;
  int i = 0;
  dma_descriptor_t *d = DESC(c);
  while (off < bytes) {
    size_t len = min(bytes - off, DESC_BYTES);
    d[i].dw0.size = len;
    d[i].dw0.length = len;
    d[i].dw0.err_eof = 0;
    d[i].dw0.suc_eof = 0;
    d[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
    d[i].buffer = (uint8_t *)s.buf + off;
    d[i].next = &d[i + 1];
    off += len;
    i++;
  }
  c.nDesc = i;
  d[i - 1].next = nullptr;
  if (c.psram) esp_cache_msync(s.buf, bytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
}

static void freeChain(void *&buf, void *&desc) {
  heap_caps_free(buf);
  heap_caps_free(desc);
  buf = desc = nullptr;
}

// Everything begin() allocated, after a failure in it.
void SM16380Panel::freeAll() {
  Chain *all[] = {&scan_[0], &scan_[1], &prog_[0], &prog_[1], &cfg_};
  for (Chain *c : all) {
    void *b = c->s.buf;
    freeChain(b, c->desc);
    *c = Chain{};
  }
  heap_caps_free(fb_);
  heap_caps_free(pause_);
  fb_ = pause_ = nullptr;
  enc_.fb = nullptr;
}

// The config program: 32 register writes, each followed by a 2 ms pause
// with the clock stopped. The pauses are identical (every register write
// ends with the same idle lines), so they are 10 descriptors each pointing
// at one shared 4 KB buffer instead of ~1.3 MB of stored words.
void SM16380Panel::buildConfig() {
  sm16380::Stream &s = cfg_.s;
  s.n = 0;
  enc_.out = &s;
  enc_.configBegin();
  size_t start[32], end[32];
  for (int i = 0; i < 32; i++) {
    start[i] = s.n;
    enc_.configReg(i);
    while (s.n % 32 && s.n < s.cap + 32) {  // whole 64-byte blocks: stretches the last clock's high half
      s.buf[s.n] = s.buf[s.n - 1];
      s.n++;
    }
    end[i] = s.n;
  }
  for (int i = 0; i < PAUSE_WORDS; i++) pause_[i] = enc_.lines;
  dma_descriptor_t *d = DESC(cfg_);
  int n = 0;
  auto add = [&](uint8_t *buf, size_t len) {
    d[n].dw0.size = len;
    d[n].dw0.length = len;
    d[n].dw0.err_eof = 0;
    d[n].dw0.suc_eof = 0;
    d[n].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
    d[n].buffer = buf;
    d[n].next = &d[n + 1];
    n++;
  };
  for (int i = 0; i < 32 && n < cfg_.maxDesc; i++) {
    for (size_t off = start[i] * 2; off < end[i] * 2; off += DESC_BYTES)
      add((uint8_t *)s.buf + off, min(end[i] * 2 - off, DESC_BYTES));
    for (int j = 0; j < pauseDescs(); j++) add((uint8_t *)pause_, PAUSE_WORDS * 2);
  }
  cfg_.nDesc = n;
  d[n - 1].next = nullptr;
  esp_cache_msync(s.buf, s.n * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
}

bool SM16380Panel::setBusDivider(int div) {
  if (lock_ || div < 12 || div > 64) {
    log_e("SM16380: bus divider %d refused (12..64, before begin())", div);
    return false;
  }
  enc_.busDiv = div;
  return true;
}

// Pause descriptors after each config register: at least 2 ms at this bus rate.
int SM16380Panel::pauseDescs() const {
  int words = 2000 * 160 / enc_.busDiv;
  return (words + PAUSE_WORDS - 1) / PAUSE_WORDS;
}

void *SM16380Panel::firstDesc(Chain &c) { return &DESC(c)[0]; }

void SM16380Panel::link(Chain &from, void *to) { DESC(from)[from.nDesc - 1].next = (dma_descriptor_t *)to; }

bool SM16380Panel::begin(const SM16380Pins &pins, const SM16380Layout &layout) {
  if (lock_) {
    log_e("SM16380: begin() called twice");
    return false;
  }
  int n = layout.panelsX * layout.panelsY;
  if (layout.panelsX < 1 || layout.panelsY < 1 || n > MAX_PANELS) {
    log_e("SM16380: %dx%d panels: 1 to %d panels in all", layout.panelsX, layout.panelsY, MAX_PANELS);
    return false;
  }
  enc_.firstShiftIsFar = layout.firstShiftIsFar;
  enc_.setLayout(layout.panelsX, layout.panelsY, layout.first, layout.snake);
  const int chips = enc_.chainChips;

  fb_ = (uint16_t *)heap_caps_calloc(enc_.width * enc_.height, 2, MALLOC_CAP_SPIRAM);
  pause_ = (uint16_t *)heap_caps_aligned_alloc(64, PAUSE_WORDS * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  // Config: 32 registers, each ~34 clocks plus six words through the whole
  // chain (padded to 64 bytes); the pauses are shared (buildConfig).
  size_t regWords = (size_t)(40 + 6 * (chips * 16 + 8)) * enc_.cfgK() + 32;
  size_t cfgWords = 32 * regWords;
  int cfgDesc = 32 * ((int)(regWords * 2 / DESC_BYTES) + 2 + pauseDescs());
  // Upload: every chip's 16 bits for 32 rows x 16 slots, plus up to a pass of padding.
  size_t upWords = (size_t)(ROWS * 16 * chips * 16 + SCAN_CLKS + 64) * enc_.upK;
  size_t scanWords = 2 * SCAN_CLKS;
  bool ok = fb_ && pause_ && alloc(scan_[0], scanWords, false) && alloc(scan_[1], scanWords, false) &&
            alloc(cfg_, cfgWords, true, cfgDesc) && alloc(prog_[0], upWords, true) && alloc(prog_[1], upWords, true);
  if (!ok) {
    log_e("SM16380: out of memory (needs ~%u KB PSRAM)", (unsigned)((cfgWords + 2 * upWords) * 2 / 1024));
    freeAll();
    return false;
  }
  enc_.fb = fb_;

  gdma_channel_handle_t ch;
  gdma_channel_alloc_config_t cc = {};
  cc.direction = GDMA_CHANNEL_DIRECTION_TX;
  if (gdma_new_ahb_channel(&cc, &ch) != ESP_OK) {
    log_e("SM16380: no free GDMA channel");
    freeAll();
    return false;
  }
  dmaChan_ = ch;

  const int8_t bus[BUS_LINES] = {pins.r1, pins.g1, pins.b1, pins.r2, pins.g2, pins.b2, pins.a,
                                 pins.b, pins.c, pins.d, pins.e, pins.lat, pins.oe, pins.clk};
  for (int i = 0; i < BUS_LINES; i++) {
    pinMode(bus[i], OUTPUT);
    digitalWrite(bus[i], LOW);
  }

  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);
  LCD_CAM.lcd_user.lcd_reset = 1;
  delayMicroseconds(1000);
  LCD_CAM.lcd_clock.lcd_clk_sel = 3;  // PLL_F160M
  LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;
  LCD_CAM.lcd_clock.lcd_clkcnt_n = 1;
  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 1;  // PCLK = LCD_CLK
  LCD_CAM.lcd_clock.lcd_clkm_div_num = enc_.busDiv;
  LCD_CAM.lcd_clock.lcd_clkm_div_a = 1;
  LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;  // i80
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0;
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;
  LCD_CAM.lcd_misc.lcd_bk_en = 1;
  LCD_CAM.lcd_data_dout_mode.val = 0;
  LCD_CAM.lcd_user.lcd_always_out_en = 1;  // run as long as the DMA has data (forever)
  LCD_CAM.lcd_user.lcd_8bits_order = 0;
  LCD_CAM.lcd_user.lcd_bit_order = 0;
  LCD_CAM.lcd_user.lcd_2byte_en = 1;
  LCD_CAM.lcd_user.lcd_dummy = 1;
  LCD_CAM.lcd_user.lcd_dummy_cyclelen = 1;
  LCD_CAM.lcd_user.lcd_cmd = 0;
  for (int i = 0; i < BUS_LINES; i++) esp_rom_gpio_connect_out_signal(bus[i], LCD_DATA_OUT0_IDX + i, false, false);

  gdma_connect(ch, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
  gdma_strategy_config_t st = {};
  st.owner_check = false;
  st.auto_update_desc = false;
  gdma_apply_strategy(ch, &st);
  gdma_transfer_config_t tc = {};
  tc.max_data_burst_size = 64;
  tc.access_ext_mem = true;
  gdma_config_transfer(ch, &tc);
  int group = 0;
  gdma_get_group_channel_id(ch, &group, &dmaCh_);
  gdma_reset(ch);
  delayMicroseconds(1000);
  LCD_CAM.lcd_user.lcd_dout = 1;
  LCD_CAM.lcd_user.lcd_update = 1;
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  lock_ = xSemaphoreCreateMutex();  // last: show() and flashHold() do nothing until here
  log_i("SM16380: %dx%d (%d panels), GDMA channel %d, %.2f M words/s", enc_.width, enc_.height, enc_.nPanels, dmaCh_,
        160.0 / enc_.busDiv);
  return true;
}

void SM16380Panel::fill(uint16_t rgb444) {
  if (!fb_) return;
  for (int i = 0, n = enc_.width * enc_.height; i < n; i++) fb_[i] = rgb444;
}

// Stop the LCD and the DMA and forget every cached stream. The next upload
// starts them again with a config and a held-OE upload, which also re-syncs
// the chip's row count (a seamless upload never does).
void SM16380Panel::restart() {
  stats_.restarts++;
  log_w("SM16380: DMA stalled, restarting");
  gdma_stop(CHAN);
  LCD_CAM.lcd_user.lcd_start = 0;
  LCD_CAM.lcd_user.lcd_update = 1;
  LCD_CAM.lcd_user.lcd_reset = 1;
  gdma_reset(CHAN);
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  running_ = false;
  pendingTarget_ = 0;
  progReusable_[0] = progReusable_[1] = false;
  scanCached_[0] = scanCached_[1] = false;
  enc_.reset();  // the lines go low with the LCD stopped
  needResync_ = true;
}

// Wait until the DMA has reached the scan chain of the last splice (until it
// has, the other scan chain may still be running). False if it never did: the
// DMA stalled and has been restarted.
bool SM16380Panel::waitSwitch() {
  if (!pendingTarget_) return true;
  uint32_t t1 = micros();
  while (gdma_ll_tx_get_eof_desc_addr(&GDMA, dmaCh_) != pendingTarget_) {
    if (micros() - t1 > 1000000) {
      stats_.timeouts++;
      restart();
      return false;
    }
    vTaskDelay(1);
  }
  stats_.switchUs = micros() - t1;
  activeScan_ = pendingScan_;
  pendingTarget_ = 0;
  return true;
}

void SM16380Panel::flashHold() {
  if (!lock_) return;
  xSemaphoreTake((SemaphoreHandle_t)lock_, portMAX_DELAY);
  waitSwitch();
}

void SM16380Panel::flashRelease() {
  if (lock_) xSemaphoreGive((SemaphoreHandle_t)lock_);
}

void SM16380Panel::show() {
  if (!lock_) return;
  xSemaphoreTake((SemaphoreHandle_t)lock_, portMAX_DELAY);
  uint32_t now = millis();
  bool config = configDue_ || (configMs_ && now - lastConfig_ >= configMs_);
  if (showLocked(config) && config) {  // only once it has really gone out
    configDue_ = false;
    lastConfig_ = now;
  }
  xSemaphoreGive((SemaphoreHandle_t)lock_);
}

// Encode the frame (after the config when asked) and splice it in after the
// current scan pass. The next call encodes into the other program buffer
// while this one is still going out, and only waits before splicing.
// False if nothing went out (stream overflow).
//
// A stall restarts the DMA (restart() clears pendingTarget_), and the retry
// below then starts it afresh: waitSwitch() returns at once in it, so a
// stall costs one retry, never a deeper recursion.
bool SM16380Panel::showLocked(bool config) {
  uint32_t t0 = micros();
  if (needResync_) {
    needResync_ = false;
    config = true;
  }
  Chain &prog = prog_[progIdx_];
  bool continuous = seamless_ && !config;  // a config upload always resets the row count too
  ScanState keepSc = enc_.sc;
  uint16_t keepLines = enc_.lines, keepPrev = enc_.prev;
  enc_.overflow = false;
  if (config) {
    // Its own chain, ahead of the upload. The last config may still be going
    // out (two in a row), so wait for that splice first.
    if (!waitSwitch()) return showLocked(true);
    buildConfig();
  }
  // This buffer is free: the splice that used it two frames ago finished
  // before the last one. Same start state and mode: only the pixels change.
  ProgKey key;
  memset(&key, 0, sizeof(key));  // padding too: compared with memcmp
  key.sc = enc_.sc;
  key.lines = enc_.lines;
  key.prev = enc_.prev;
  key.continuous = continuous;
  if (enc_.upK == 2 && progReusable_[progIdx_] && !memcmp(&key, &progKey_[progIdx_], sizeof(key))) {
    enc_.patch(prog.s);
    enc_.sc = progEndSc_[progIdx_];
    enc_.lines = enc_.prev = progEndLines_[progIdx_];
    stats_.patched++;
  } else {
    enc_.out = &prog.s;
    prog.s.n = 0;
    enc_.upload(continuous);
    progReusable_[progIdx_] = enc_.upK == 2 && !enc_.overflow;  // the layout patch() knows
    progKey_[progIdx_] = key;
    progEndSc_[progIdx_] = enc_.sc;
    progEndLines_[progIdx_] = enc_.lines;
  }
  finish(prog);
  uint32_t encodeUs = micros() - t0;

  if (!waitSwitch()) return showLocked(config);  // restarted under this program: again, as a resync

  // The scan chain carries on from wherever the program left the scan
  // state; one pass brings it back to the same state.
  uint32_t t2 = micros();
  int next = activeScan_ ^ 1;
  Chain &s = scan_[next];
  if (scanCached_[next] && !memcmp(&enc_.sc, &scanStartSc_[next], sizeof(ScanState)) &&
      enc_.lines == scanStartLines_[next]) {
    enc_.sc = scanEndSc_[next];
    enc_.lines = enc_.prev = scanEndLines_[next];
  } else {
    scanStartSc_[next] = enc_.sc;
    scanStartLines_[next] = enc_.lines;
    enc_.out = &s.s;
    s.s.n = 0;
    enc_.scanPass();
    scanEndSc_[next] = enc_.sc;
    scanEndLines_[next] = enc_.lines;
    scanCached_[next] = !enc_.overflow;
  }
  finish(s);
  if (enc_.overflow) {
    enc_.sc = keepSc;  // nothing goes out, so the encoder state stays with the hardware
    enc_.lines = keepLines;
    enc_.prev = keepPrev;
    progReusable_[progIdx_] = false;
    stats_.overflows++;
    log_w("SM16380: stream overflow, frame skipped");
    return false;
  }
  link(s, firstDesc(s));
  DESC(s)[0].dw0.suc_eof = 1;  // marks arrival on this chain (eof_des_addr)
  link(prog, firstDesc(s));
  void *start = firstDesc(prog);
  if (config) {
    link(cfg_, start);
    start = firstDesc(cfg_);
  }
  stats_.encodeUs = encodeUs + (micros() - t2);
  stats_.programBytes = prog.s.n * 2 + (config ? cfg_.s.n * 2 : 0);
  __sync_synchronize();

  if (!running_) {
    // The same resets a restart does: a first start without them stalls.
    LCD_CAM.lcd_user.lcd_reset = 1;
    gdma_reset(CHAN);
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    gdma_start(CHAN, (intptr_t)start);
    delayMicroseconds(100);
    LCD_CAM.lcd_user.lcd_update = 1;
    LCD_CAM.lcd_user.lcd_start = 1;
    running_ = true;
  } else {
    link(scan_[activeScan_], start);
    __sync_synchronize();
  }
  pendingTarget_ = (uint32_t)firstDesc(s);
  pendingScan_ = next;
  progIdx_ ^= 1;
  stats_.uploads++;
  return true;
}
