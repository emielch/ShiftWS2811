/*  ShiftWS2811 - High Performance WS2811 LED Display Library
    http://www.pjrc.com/teensy/td_libs_ShiftWS2811.html
    Copyright (c) 2020 Paul Stoffregen, PJRC.COM, LLC

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "ShiftWS2811.h"

#include <Arduino.h>

#if defined(__IMXRT1062__)

#define TH_TL 1.25e-6
#define T0H 0.30e-6
#define T1H 0.75e-6

// Ordinary RGB data is converted to GPIO bitmasks on-the-fly using
// a transmit buffer sized for 2 DMA transfers.  The larger this setting,
// the more interrupt latency ShiftWS2811 can tolerate, but the transmit
// buffer grows in size.  For good performance, the buffer should be kept
// smaller than the half the Cortex-M7 data cache.
#define BYTES_PER_DMA 2

uint8_t ShiftWS2811::defaultPinList[8] = {16, 17, 18, 19, 20, 21, 22, 23};
uint16_t ShiftWS2811::stripLen;
// uint8_t ShiftWS2811::brightness = 255;
void *ShiftWS2811::frameBuffer;
void *ShiftWS2811::drawBuffer;
uint8_t ShiftWS2811::params;
DMAChannel ShiftWS2811::dma2;
static DMASetting dma2next;
static uint32_t numbytes;

static uint8_t numpins;
static uint8_t pinlist[NUM_DIGITAL_PINS];  // = {2, 14, 7, 8, 6, 20, 21, 5};
static uint8_t pin_bitnum[NUM_DIGITAL_PINS];
static uint8_t pin_offset[NUM_DIGITAL_PINS];

DMAMEM static uint32_t bitmask[4] __attribute__((used, aligned(32)));
// *8 for 8 bits in a byte, *8 for 8 bits in a SR, *3 for WS2811 waveform, *2 for circular buffer (filling buffer while DMA sends the other half)
DMAMEM static uint32_t bitdata[BYTES_PER_DMA * 8 * 8 * 3 * 2] __attribute__((used, aligned(32)));
volatile uint32_t framebuffer_index = 0;
volatile bool dma_first;

static uint32_t update_begin_micros = 0;
static elapsedMicros sinceFinish = 0;
static volatile bool transferring = false;

ShiftWS2811::ShiftWS2811(uint32_t numPerStrip, void *frameBuf, void *drawBuf, uint8_t config, uint8_t numPins, const uint8_t *pinList) {
  stripLen = numPerStrip;
  frameBuffer = frameBuf;
  drawBuffer = drawBuf;
  params = config;
  if (numPins > NUM_DIGITAL_PINS) numPins = NUM_DIGITAL_PINS;
  numpins = numPins;
  memcpy(pinlist, pinList, numpins);
}

void ShiftWS2811::begin(uint32_t numPerStrip, void *frameBuf, void *drawBuf, uint8_t config, uint8_t numPins, const uint8_t *pinList) {
  stripLen = numPerStrip;
  frameBuffer = frameBuf;
  drawBuffer = drawBuf;
  params = config;
  if (numPins > NUM_DIGITAL_PINS) numPins = NUM_DIGITAL_PINS;
  numpins = numPins;
  memcpy(pinlist, pinList, numpins);
  begin();
}

int ShiftWS2811::numPixels(void) {
  return stripLen * numpins;
}

extern "C" void xbar_connect(unsigned int input, unsigned int output);  // in pwm.c
static volatile uint32_t *standard_gpio_addr(volatile uint32_t *fastgpio) {
  return (volatile uint32_t *)((uint32_t)fastgpio - 0x01E48000);
}

void ShiftWS2811::begin(void) {
  transferring = false;
  if ((params & 0x1F) < 6) {
    numbytes = stripLen * 3;  // RGB formats
  } else {
    numbytes = stripLen * 4;  // RGBW formats
  }

  // configure which pins to use
  memset(bitmask, 0, sizeof(bitmask));
  for (uint32_t i = 0; i < numpins; i++) {
    uint8_t pin = pinlist[i];
    if (pin >= NUM_DIGITAL_PINS) continue;  // ignore illegal pins
    uint8_t bit = digitalPinToBit(pin);
    uint8_t offset = ((uint32_t)portOutputRegister(pin) - (uint32_t)&GPIO6_DR) >> 14;
    if (offset > 3) continue;  // ignore unknown pins
    pin_bitnum[i] = bit;
    pin_offset[i] = offset;
    uint32_t mask = 1 << bit;
    bitmask[offset] |= mask;
    *(&IOMUXC_GPR_GPR26 + offset) &= ~mask;
    *standard_gpio_addr(portModeRegister(pin)) |= mask;
  }
  arm_dcache_flush_delete(bitmask, sizeof(bitmask));

  //----------------- TIMERS/CLOCKS ----------------------
  // pin 14 (TMR3.2) & 15 (TMR3.3)

  TMR3_ENBL = 0;  // turn off all timers

  TMR3_SCTRL0 = TMR_SCTRL_OEN | TMR_SCTRL_FORCE;
  TMR3_CSCTRL0 = 0;  // reset to 0, is set by pwm init code otherwise
  // TMR3_CNTR0 = 0;
  TMR3_LOAD0 = 0;
  TMR3_COMP10 = 14;                                                                       // 700KHz resulting WS2811 frequency
  TMR3_CTRL0 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(3);  // Timer Channel Control Register p.3079
  // Count Mode: Count rising edges of primary source | Primary Count Source: IP bus clock divide by 1 prescaler
  // | Count Length: Count until compare, then re-initialize | Output Mode: Toggle OFLAG output on successful compare

  // SHIFT CLOCK
  TMR3_SCTRL3 = TMR_SCTRL_OEN | TMR_SCTRL_OPS | TMR_SCTRL_VAL | TMR_SCTRL_FORCE;  // Timer Channel Status and Control Register  // enable output | invert polarity | set output to ~1
  TMR3_CSCTRL3 = 0;                                                               // Timer Channel Comparator Status and Control Register  // reset to 0, is set by pwm init code otherwise
  // TMR3_CNTR3 = 7;
  TMR3_LOAD3 = 65537 - 8;                                                                 // low time  (65537 - x) -
  TMR3_COMP13 = 7;                                                                        // high time (0 = always low, max = LOAD-1)
  TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(6);  // Control Register // .... | Output Mode: Set on compare, cleared on counter rollover
  *(portConfigRegister(15)) = 1;                                                          // set pin 15 to output TMR3 ch1 OFLAG

  // STORE CLOCK
  TMR3_SCTRL2 = TMR_SCTRL_OEN | TMR_SCTRL_OPS | TMR_SCTRL_VAL | TMR_SCTRL_FORCE;  // Timer Channel Status and Control Register  // enable output | invert polarity | set output to ~1
  TMR3_CSCTRL2 = 0;                                                               // reset to 0, is set by pwm init code otherwise
  // TMR3_CNTR2 = 65537 - 32;
  TMR3_LOAD2 = 65537 - 110;                                                               // low time  (65537 - x) -
  TMR3_COMP12 = 10;                                                                       // high time (0 = always low, max = LOAD-1)
  TMR3_CTRL2 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(6);  // Control Register
  *(portConfigRegister(14)) = 1;                                                          // set pin 14 to output TMR3 ch1 OFLAG

  // route the timer outputs through XBAR to edge trigger DMA request
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
  xbar_connect(XBARA1_IN_QTIMER3_TIMER0, XBARA1_OUT_DMA_CH_MUX_REQ30);

  XBARA1_CTRL0 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(3) | XBARA_CTRL_DEN0;

  // configure DMA channels
  dma2next.TCD->SADDR = bitdata;
  dma2next.TCD->SOFF = 4;
  dma2next.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
  dma2next.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_MLOFFYES_NBYTES(4);
  dma2next.TCD->SLAST = 0;
  dma2next.TCD->DADDR = &GPIO1_DR;
  dma2next.TCD->DOFF = 0;
  dma2next.TCD->CITER_ELINKNO = BYTES_PER_DMA * 8 * 3 - 1;
  dma2next.TCD->DLASTSGA = (int32_t)(dma2next.TCD);
  dma2next.TCD->BITER_ELINKNO = BYTES_PER_DMA * 8 * 3 - 1;
  dma2next.TCD->CSR = DMA_TCD_CSR_DONE;

  dma2.begin();
  dma2 = dma2next;  // copies TCD
  dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_XBAR1_0);
  dma2.attachInterrupt(isr);

  // set up the buffers
  uint32_t bufsize = numbytes * numpins;
  memset(frameBuffer, 0, bufsize);
  if (drawBuffer) {
    memset(drawBuffer, 0, bufsize);
  } else {
    drawBuffer = frameBuffer;
  }

  for (uint i = 0; i < sizeof(bitdata) / 4; i++) {
    bitdata[i] = i % 24 < 8 ? bitmask[0] : 0;
  }
  arm_dcache_flush_delete(bitdata, sizeof(bitdata));
}

static void fillbits(uint32_t *dest, const uint8_t *pixels, int n, uint32_t mask) {
  do {
    uint8_t pix = *pixels++;
    if ((pix & 0x80))
      *dest |= mask;
    else
      *dest &= ~mask;
    dest += 24;  // *8 for pins on SR, *3 for WS2811 waveform (=24)
    if ((pix & 0x40))
      *dest |= mask;
    else
      *dest &= ~mask;
    dest += 24;
    if ((pix & 0x20))
      *dest |= mask;
    else
      *dest &= ~mask;
    dest += 24;
    if ((pix & 0x10))
      *dest |= mask;
    else
      *dest &= ~mask;
    dest += 24;
    if ((pix & 0x08))
      *dest |= mask;
    else
      *dest &= ~mask;
    dest += 24;
    if ((pix & 0x04))
      *dest |= mask;
    else
      *dest &= ~mask;
    dest += 24;
    if ((pix & 0x02))
      *dest |= mask;
    else
      *dest &= ~mask;
    dest += 24;
    if ((pix & 0x01))
      *dest |= mask;
    else
      *dest &= ~mask;
    dest += 24;
  } while (--n > 0);
}

void ShiftWS2811::show(void) {
  // wait for any prior DMA operation
  while (transferring)
    ;  // wait

  // it's ok to copy the drawing buffer to the frame buffer
  // during the 50us WS2811 reset time
  if (drawBuffer != frameBuffer) {
    memcpy(frameBuffer, drawBuffer, numbytes * numpins * 8);
  }

  // disable timers
  TMR3_ENBL = 0;

  // force all timer outputs to logic low
  TMR3_SCTRL3 = TMR_SCTRL_OEN | TMR_SCTRL_OPS | TMR_SCTRL_VAL | TMR_SCTRL_FORCE;
  TMR3_SCTRL2 = TMR_SCTRL_OEN | TMR_SCTRL_OPS | TMR_SCTRL_VAL | TMR_SCTRL_FORCE;

  // clear any prior pending DMA requests
  XBARA1_CTRL0 |= XBARA_CTRL_STS1 | XBARA_CTRL_STS0;
  XBARA1_CTRL1 |= XBARA_CTRL_STS0;

  // fill the DMA transmit buffer
  // digitalWriteFast(12, HIGH);
  // memset(bitdata, 0, sizeof(bitdata));

  uint32_t count = numbytes;
  if (count > BYTES_PER_DMA * 2) count = BYTES_PER_DMA * 2;
  framebuffer_index = count;

  for (uint32_t i = 0; i < numpins; i++) {
    if (pin_offset[i] > 0) continue;
    for (uint32_t j = 0; j < 8; j++) {  // 8 pins on the SR
      fillbits(bitdata + 8 + 7 - j, (uint8_t *)frameBuffer + i * numbytes * 8 + j * numbytes, count, 1 << pin_bitnum[i]);
    }
  }
  arm_dcache_flush_delete(bitdata, sizeof(bitdata));

  // set up DMA transfers
  if (numbytes <= BYTES_PER_DMA * 2) {
    dma2.TCD->SADDR = bitdata;
    dma2.TCD->DADDR = &GPIO1_DR;
    dma2.TCD->CITER_ELINKNO = count * 8 * 8 * 3;
    dma2.TCD->CSR = DMA_TCD_CSR_DREQ | DMA_TCD_CSR_INTMAJOR;
  } else {
    dma2.TCD->SADDR = bitdata;
    dma2.TCD->DADDR = &GPIO1_DR;
    dma2.TCD->CITER_ELINKNO = BYTES_PER_DMA * 8 * 8 * 3 - 1;
    dma2.TCD->CSR = 0;
    dma2.TCD->CSR = DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_ESG;
    dma2next.TCD->SADDR = bitdata + BYTES_PER_DMA * 8 * 8 * 3;
    dma2next.TCD->CITER_ELINKNO = BYTES_PER_DMA * 8 * 8 * 3 - 1;
    if (numbytes <= BYTES_PER_DMA * 3) {
      dma2next.TCD->CSR = DMA_TCD_CSR_ESG;
    } else {
      dma2next.TCD->CSR = DMA_TCD_CSR_ESG | DMA_TCD_CSR_INTMAJOR;
    }
    dma_first = true;
  }
  dma2.clearComplete();
  dma2.enable();

  // // initialize timers // for TMR3_COMP10 = 13
  // TMR3_CNTR0 = 13;
  // TMR3_CNTR3 = 65537 - 24; // - 17 is exactly aligned, -18 to stagger signals
  // TMR3_CNTR2 = 0; // 7 is exactly aligned, 6 to stagger signals

  // initialize timers // for TMR3_COMP10 = 14
  TMR3_CNTR0 = 14;
  TMR3_CNTR3 = 65537 - 20;  // -17 is exactly aligned, -18 to stagger signals
  TMR3_CNTR2 = 3;           // 8 is exactly aligned, 7 to stagger signals

  // wait for WS2812 reset
  while (sinceFinish < 80)
    ;
  // start everything running!
  // TMR3_ENBL |= 0b1101;  // enable TMR3, channel 0, 2
  TMR3_ENBL |= 0b1001;  // enable TMR3, channel 0, 3
  delayNanoseconds(30);
  TMR3_ENBL |= 0b0100;  // enable TMR3, channel 2
  update_begin_micros = micros();
  transferring = true;
}

void ShiftWS2811::isr(void) {
  // first ack the interrupt
  dma2.clearInterrupt();

  if (framebuffer_index >= numbytes) {
    delayNanoseconds(50);
    TMR3_ENBL = 0;  // turn off all timers
    sinceFinish = 0;
    transferring = false;
    return;
  }

  // fill (up to) half the transmit buffer with new data
  uint32_t *dest;
  if (dma_first) {
    dma_first = false;
    dest = bitdata;
  } else {
    dma_first = true;
    dest = bitdata + BYTES_PER_DMA * 8 * 8 * 3;
  }
  uint32_t index = framebuffer_index;
  uint32_t count = numbytes - framebuffer_index;
  if (count > BYTES_PER_DMA) count = BYTES_PER_DMA;
  framebuffer_index = index + count;

  for (uint32_t i = 0; i < numpins; i++) {
    if (pin_offset[i] > 0) continue;
    for (uint32_t j = 0; j < 8; j++) {  // 8 pins on the SR
      fillbits(dest + 8 + 7 - j, (uint8_t *)frameBuffer + index + i * numbytes * 8 + j * numbytes, count, 1 << pin_bitnum[i]);
    }
  }
  arm_dcache_flush_delete(dest, sizeof(bitdata) / 2);

  // queue it for the next DMA transfer
  dma2next.TCD->SADDR = dest;
  dma2next.TCD->CITER_ELINKNO = count * 8 * 8 * 3 - 1;
  uint32_t remain = numbytes - (index + count);
  if (remain == 0) {
    dma2next.TCD->CSR = DMA_TCD_CSR_DREQ | DMA_TCD_CSR_INTMAJOR;
  } else if (remain <= BYTES_PER_DMA) {
    dma2next.TCD->CSR = DMA_TCD_CSR_ESG;
  } else {
    dma2next.TCD->CSR = DMA_TCD_CSR_ESG | DMA_TCD_CSR_INTMAJOR;
  }
}

int ShiftWS2811::busy(void) {
  if (!dma2.complete())
    ;                                                                  // DMA still running
  if (micros() - update_begin_micros < numbytes * 10 + 300) return 1;  // WS2812 reset
  return 0;
}

// For Teensy 4.x, the pixel data is stored in ordinary RGB format.  Translation
// from 24 bit color to GPIO bitmasks is done on-the-fly by fillbits().  This is
// different from Teensy 3.x, where the data was stored as bytes to write directly
// to the GPIO output register.

void ShiftWS2811::setPixel(uint32_t num, int color) {
  if ((params & 0x1F) < 6) {
    switch (params & 7) {
      case WS2811_RBG:
        color = (color & 0xFF0000) | ((color << 8) & 0x00FF00) | ((color >> 8) & 0x0000FF);
        break;
      case WS2811_GRB:
        color = ((color << 8) & 0xFF0000) | ((color >> 8) & 0x00FF00) | (color & 0x0000FF);
        break;
      case WS2811_GBR:
        color = ((color << 16) & 0xFF0000) | ((color >> 8) & 0x00FFFF);
        break;
      case WS2811_BRG:
        color = ((color << 8) & 0xFFFF00) | ((color >> 16) & 0x0000FF);
        break;
      case WS2811_BGR:
        color = ((color << 16) & 0xFF0000) | (color & 0x00FF00) | ((color >> 16) & 0x0000FF);
        break;
      default:
        break;
    }
    uint8_t *dest = (uint8_t *)drawBuffer + num * 3;
    *dest++ = color >> 16;
    *dest++ = color >> 8;
    *dest++ = color;
  } else {
    uint8_t b = color;
    uint8_t g = color >> 8;
    uint8_t r = color >> 16;
    uint8_t w = color >> 24;
    uint32_t c = 0;
    switch (params & 0x1F) {
      case WS2811_RGBW:
        c = (r << 24) | (g << 16) | (b << 8) | w;
        break;
      case WS2811_RBGW:
        c = (r << 24) | (b << 16) | (g << 8) | w;
        break;
      case WS2811_GRBW:
        c = (g << 24) | (r << 16) | (b << 8) | w;
        break;
      case WS2811_GBRW:
        c = (g << 24) | (b << 16) | (r << 8) | w;
        break;
      case WS2811_BRGW:
        c = (b << 24) | (r << 16) | (g << 8) | w;
        break;
      case WS2811_BGRW:
        c = (b << 24) | (b << 16) | (r << 8) | w;
        break;
      case WS2811_WRGB:
        c = (w << 24) | (r << 16) | (g << 8) | b;
        break;
      case WS2811_WRBG:
        c = (w << 24) | (r << 16) | (b << 8) | g;
        break;
      case WS2811_WGRB:
        c = (w << 24) | (g << 16) | (r << 8) | b;
        break;
      case WS2811_WGBR:
        c = (w << 24) | (g << 16) | (b << 8) | r;
        break;
      case WS2811_WBRG:
        c = (w << 24) | (b << 16) | (r << 8) | g;
        break;
      case WS2811_WBGR:
        c = (w << 24) | (b << 16) | (g << 8) | r;
        break;
      case WS2811_RWGB:
        c = (r << 24) | (w << 16) | (g << 8) | b;
        break;
      case WS2811_RWBG:
        c = (r << 24) | (w << 16) | (b << 8) | g;
        break;
      case WS2811_GWRB:
        c = (g << 24) | (w << 16) | (r << 8) | b;
        break;
      case WS2811_GWBR:
        c = (g << 24) | (w << 16) | (b << 8) | r;
        break;
      case WS2811_BWRG:
        c = (b << 24) | (w << 16) | (r << 8) | g;
        break;
      case WS2811_BWGR:
        c = (b << 24) | (w << 16) | (g << 8) | r;
        break;
      case WS2811_RGWB:
        c = (r << 24) | (g << 16) | (w << 8) | b;
        break;
      case WS2811_RBWG:
        c = (r << 24) | (b << 16) | (w << 8) | g;
        break;
      case WS2811_GRWB:
        c = (g << 24) | (r << 16) | (w << 8) | b;
        break;
      case WS2811_GBWR:
        c = (g << 24) | (b << 16) | (w << 8) | r;
        break;
      case WS2811_BRWG:
        c = (b << 24) | (r << 16) | (w << 8) | g;
        break;
      case WS2811_BGWR:
        c = (b << 24) | (g << 16) | (w << 8) | r;
        break;
    }
    uint8_t *dest = (uint8_t *)drawBuffer + num * 4;
    *dest++ = c >> 24;
    *dest++ = c >> 16;
    *dest++ = c >> 8;
    *dest++ = c;
  }
}

int ShiftWS2811::getPixel(uint32_t num) {
  int color = 0;

  if ((params & 0x1F) < 6) {
    const uint8_t *p = (uint8_t *)drawBuffer + num * 3;
    color = p[2] | (p[1] << 8) | (p[0] << 16);
    switch (params & 7) {
      case WS2811_RBG:
        color = (color & 0xFF0000) | ((color << 8) & 0x00FF00) | ((color >> 8) & 0x0000FF);
        break;
      case WS2811_GRB:
        color = ((color << 8) & 0xFF0000) | ((color >> 8) & 0x00FF00) | (color & 0x0000FF);
        break;
      case WS2811_GBR:
        color = ((color << 8) & 0xFFFF00) | ((color >> 16) & 0x0000FF);
        break;
      case WS2811_BRG:
        color = ((color << 16) & 0xFF0000) | ((color >> 8) & 0x00FFFF);
        break;
      case WS2811_BGR:
        color = ((color << 16) & 0xFF0000) | (color & 0x00FF00) | ((color >> 16) & 0x0000FF);
        break;
      default:
        break;
    }
  } else {
    const uint8_t *p = (uint8_t *)drawBuffer + num * 4;
    uint8_t r = *p++;
    uint8_t g = *p++;
    uint8_t b = *p++;
    uint8_t w = *p++;
    switch (params & 0x1F) {
      case WS2811_RGBW:
        color = (r << 16) | (g << 8) | b | (w << 24);
        break;
      case WS2811_RBGW:
        color = (r << 16) | (b << 8) | g | (w << 24);
        break;
      case WS2811_GRBW:
        color = (g << 16) | (r << 8) | b | (w << 24);
        break;
      case WS2811_GBRW:
        color = (g << 16) | (b << 8) | r | (w << 24);
        break;
      case WS2811_BRGW:
        color = (b << 16) | (r << 8) | g | (w << 24);
        break;
      case WS2811_BGRW:
        color = (b << 16) | (g << 8) | r | (w << 24);
        break;
      case WS2811_WRGB:
        color = (w << 16) | (r << 8) | g | (b << 24);
        break;
      case WS2811_WRBG:
        color = (w << 16) | (r << 8) | b | (g << 24);
        break;
      case WS2811_WGRB:
        color = (w << 16) | (g << 8) | r | (b << 24);
        break;
      case WS2811_WGBR:
        color = (w << 16) | (g << 8) | b | (r << 24);
        break;
      case WS2811_WBRG:
        color = (w << 16) | (b << 8) | r | (g << 24);
        break;
      case WS2811_WBGR:
        color = (w << 16) | (b << 8) | g | (r << 24);
        break;
      case WS2811_RWGB:
        color = (r << 16) | (w << 8) | g | (b << 24);
        break;
      case WS2811_RWBG:
        color = (r << 16) | (w << 8) | b | (g << 24);
        break;
      case WS2811_GWRB:
        color = (g << 16) | (w << 8) | r | (b << 24);
        break;
      case WS2811_GWBR:
        color = (g << 16) | (w << 8) | b | (r << 24);
        break;
      case WS2811_BWRG:
        color = (b << 16) | (w << 8) | r | (g << 24);
        break;
      case WS2811_BWGR:
        color = (b << 16) | (w << 8) | g | (r << 24);
        break;
      case WS2811_RGWB:
        color = (r << 16) | (g << 8) | w | (b << 24);
        break;
      case WS2811_RBWG:
        color = (r << 16) | (b << 8) | w | (g << 24);
        break;
      case WS2811_GRWB:
        color = (g << 16) | (r << 8) | w | (b << 24);
        break;
      case WS2811_GBWR:
        color = (g << 16) | (b << 8) | w | (r << 24);
        break;
      case WS2811_BRWG:
        color = (b << 16) | (r << 8) | w | (g << 24);
        break;
      case WS2811_BGWR:
        color = (b << 16) | (g << 8) | w | (r << 24);
        break;
    }
  }
  return color;
}

#endif  // __IMXRT1062__
