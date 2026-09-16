/*  ShiftWS2811 - 128 channel WS2811 LED driver through 74HC595 shift registers
    FlexIO2 + eDMA implementation for Teensy 4.1

    Copyright (c) 2020 Paul Stoffregen, PJRC.COM, LLC (OctoWS2811 origins)
    Copyright (c) 2026 Emiel Harmsen

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

#include "ShiftWS2811_fill.h"
#include "gammaLUT.h"

#if defined(__IMXRT1062__)

/* ===========================================================================
   How this works
   ===========================================================================
   Board (see the ShiftWS2811_PCB KiCad project):
     - 8 chains of 2x74AHCT595 (16 outputs each).  DATA of chain i comes
       straight from a Teensy pin, SRCLK of all chains from pin 10 and RCLK
       (store) of all chains from pin 11.
     - Pin 11 also, RC delayed, enables the 595 outputs (through the inverting
       74AHCT240) and tri-states the COMMON_WF drivers (74AHCT245 U3):
           STORE high -> the 595 outputs drive the LED data lines (data phase)
           STORE low  -> the lines follow COMMON_WF through 1k resistors
     - Pin 12 carries COMMON_WF: high during the leading T0H phase of every
       WS2811 bit and low during the trailing low phase.

   One WS2811 bit period (SHIFTWS2811_BIT_NS), everything in FlexIO clocks:

     COMMON  ____/~~~~~~~~~~~~~~~~~~~~~~~~\_____________________________/~~~~
     STORE   _____________/~~~~~~~~~~~~~~~~~~~~~~~~~~~~\_____________________
     LED     ____/~~~~~~~~~<      595 data (0 or 1)   >_____________________
                 <- T0H -> <-         DATA_NS        -> <-      low     ->

   The STORE rising edge latches the 16 bits that were shifted into the 595s
   during the previous bit period; the shift clock therefore runs continuously
   with exactly 16 shifts per bit period and the STORE edge is placed midway
   between two shift clock edges.

   FlexIO2 does all of the timing, the CPU only converts colour data:

     shifter 0..7  transmit mode, 32-bit parallel shift, chained (INSRC).
                   Shifter 0 drives FlexIO2 pins D[lo..lo+width], a range that
                   covers all data pins (pads in the range that are not muxed
                   to FlexIO are unaffected).  Every shift clock outputs one
                   32-bit word, so the 8 shifters hold 8 shifts and the DMA
                   refills all 8 SHIFTBUF registers (32 bytes) twice per bit
                   period.  Shifter 0's status flag is the DMA request.
     timer 0       shift clock, dual 8-bit baud mode, pin 10.  Starts when the
                   DMA has filled SHIFTBUF7 and runs until the frame is done.
     timer 1, 2    one-shot delays started together with timer 0.  Their
                   falling edges start the COMMON and STORE PWM timers, which
                   fixes the phase of all waveforms to the cycle.
     timer 6, 7    COMMON and STORE, dual 8-bit PWM mode, pins 12 and 11.
     timer 5       counts STORE edges (decrements on the STORE pin) and, after
                   the last bit of the frame, disables itself, which disables
                   timers 6 and 7 with it ("disable on timer N-1 disable").
     timer 4       reset gap: started by timer 5's end, interrupts after
                   SHIFTWS2811_RESET_US so the next frame can begin.

   The eDMA streams 32-byte groups from a double buffered conversion buffer
   into SHIFTBUF0..7 (destination address modulo 32).  After the last data
   group a self-linking TCD keeps writing zeros until the frame end interrupt
   stops the shift clock, so the shifters never underrun at the frame end.  A
   real underrun during a frame sets SHIFTERR, which is counted per frame and
   available through underruns().
   =========================================================================== */

/* ------------------------------------------------------------------ tuning */

#ifndef SHIFTWS2811_BIT_NS
#define SHIFTWS2811_BIT_NS 1250 /* WS2811 bit period (800 kHz) */
#endif
#ifndef SHIFTWS2811_T0H_NS
#define SHIFTWS2811_T0H_NS 300 /* high time of a 0 bit (COMMON high before STORE) */
#endif
#ifndef SHIFTWS2811_DATA_NS
#define SHIFTWS2811_DATA_NS 400 /* 595 outputs enabled; a 1 bit is high for T0H + DATA */
#endif
#ifndef SHIFTWS2811_RESET_US
#define SHIFTWS2811_RESET_US 80 /* low time between frames */
#endif
#ifndef SHIFTWS2811_BYTES_PER_DMA
#define SHIFTWS2811_BYTES_PER_DMA 6 /* LED bytes converted per DMA interrupt (two buffers) */
#endif
#ifndef SHIFTWS2811_SHIFT_DIV
#define SHIFTWS2811_SHIFT_DIV 3 /* shift clock period = 2 * (DIV + 1) FlexIO clocks */
#endif
#ifndef SHIFTWS2811_TRIGGER_LATENCY
#define SHIFTWS2811_TRIGGER_LATENCY 1 /* FlexIO clocks from a timer output edge to the start of the timer it enables */
#endif
#ifndef SHIFTWS2811_DMA_BURST
#define SHIFTWS2811_DMA_BURST 1 /* 1: 32-byte burst source reads, 0: 32-bit source reads */
#endif
#ifndef SHIFTWS2811_DMA_PRIORITY
#define SHIFTWS2811_DMA_PRIORITY 1 /* 1: give the DMA channel the highest fixed priority of its group */
#endif
#ifndef SHIFTWS2811_PIN_SHIFT_CLK
#define SHIFTWS2811_PIN_SHIFT_CLK 10 /* 74HC595 SRCLK   (FlexIO2 D0) */
#endif
#ifndef SHIFTWS2811_PIN_STORE
#define SHIFTWS2811_PIN_STORE 11 /* 74HC595 RCLK / output enable (FlexIO2 D2) */
#endif
#ifndef SHIFTWS2811_PIN_COMMON
#define SHIFTWS2811_PIN_COMMON 12 /* COMMON_WF (FlexIO2 D1) */
#endif

/* ---------------------------------------------------------- derived values */

static const uint32_t CYC_PER_SHIFT = 2 * (SHIFTWS2811_SHIFT_DIV + 1);
static const uint32_t CYC_PER_BIT = SHIFTWS_SR_LEN * CYC_PER_SHIFT;
static const uint32_t FLEXIO_HZ = (uint32_t)((uint64_t)CYC_PER_BIT * 1000000000ull / SHIFTWS2811_BIT_NS);

static constexpr uint32_t ns2cyc(uint32_t ns) {
  return (uint32_t)(((uint64_t)ns * CYC_PER_BIT + SHIFTWS2811_BIT_NS / 2) / SHIFTWS2811_BIT_NS);
}

static const uint32_t T0H_CYC = ns2cyc(SHIFTWS2811_T0H_NS);
static const uint32_t DATA_CYC = ns2cyc(SHIFTWS2811_DATA_NS);
static const uint32_t STORE_HIGH_CYC = DATA_CYC;
static const uint32_t STORE_LOW_CYC = CYC_PER_BIT - STORE_HIGH_CYC;
/* COMMON stays high into the data phase; while STORE is high the COMMON drivers
   are tri-stated anyway, so this only guarantees there is never a gap. */
static const uint32_t COMMON_HIGH_CYC = T0H_CYC + DATA_CYC / 2;
static const uint32_t COMMON_LOW_CYC = CYC_PER_BIT - COMMON_HIGH_CYC;
/* One-shot delays measured from the shift clock start.  The first STORE edge
   comes one bit period after the shift clock starts, i.e. exactly between the
   16th and 17th shift clock rising edge. */
static const uint32_t DELAY_STORE_CYC = CYC_PER_BIT - SHIFTWS2811_TRIGGER_LATENCY;
static const uint32_t DELAY_COMMON_CYC = CYC_PER_BIT - T0H_CYC - SHIFTWS2811_TRIGGER_LATENCY;
static const uint32_t GAP_CYC = (uint32_t)((uint64_t)SHIFTWS2811_RESET_US * FLEXIO_HZ / 1000000);
static const double LED_TIME = 24.0 * SHIFTWS2811_BIT_NS * 1e-9; /* seconds per RGB LED */

static_assert(SHIFTWS2811_SHIFT_DIV >= 0 && SHIFTWS2811_SHIFT_DIV <= 255, "SHIFTWS2811_SHIFT_DIV must fit 8 bits");
static_assert(FLEXIO_HZ <= 120000000u, "FlexIO clock above 120 MHz: raise SHIFTWS2811_BIT_NS or lower SHIFTWS2811_SHIFT_DIV");
static_assert(T0H_CYC >= 1 && T0H_CYC + DATA_CYC < CYC_PER_BIT, "T0H + DATA must be shorter than the bit period");
static_assert(STORE_HIGH_CYC >= 1 && STORE_HIGH_CYC <= 256 && STORE_LOW_CYC >= 1 && STORE_LOW_CYC <= 256,
              "STORE PWM phases must be 1..256 FlexIO clocks");
static_assert(COMMON_HIGH_CYC >= 1 && COMMON_HIGH_CYC <= 256 && COMMON_LOW_CYC >= 1 && COMMON_LOW_CYC <= 256,
              "COMMON PWM phases must be 1..256 FlexIO clocks");
static_assert(DELAY_COMMON_CYC >= 1 && DELAY_STORE_CYC >= 1, "trigger latency too large");
static_assert(GAP_CYC >= 1 && GAP_CYC <= 65535, "reset gap does not fit a 16-bit timer");
static_assert(SHIFTWS2811_BYTES_PER_DMA >= 1, "need at least one LED byte per DMA buffer");

/* FlexIO2 timer allocation.  Timer 0 is the only one that can not use "enable
   on timer N-1 enable"; timers 6 and 7 use "disable on timer N-1 disable". */
enum {
  TMR_SHIFT = 0,        /* shift clock */
  TMR_DELAY_COMMON = 1, /* one-shot, starts with TMR_SHIFT */
  TMR_DELAY_STORE = 2,  /* one-shot, starts with TMR_DELAY_COMMON */
  TMR_GAP = 4,          /* reset gap after the frame */
  TMR_END = 5,          /* counts STORE edges, ends the frame */
  TMR_COMMON = 6,       /* COMMON PWM, disabled with TMR_END */
  TMR_STORE = 7,        /* STORE PWM, disabled with TMR_COMMON */
};
static constexpr uint32_t TRG_TIMER(uint32_t n) { return 4 * n + 3; }   /* TRGSEL: timer N output */
static constexpr uint32_t TRG_SHIFTER(uint32_t n) { return 4 * n + 1; } /* TRGSEL: shifter N status flag */
static const int NUM_SHIFTERS = 8;

/* --------------------------------------------------------------- variables */

bool ShiftWS2811::gammaCorrection;
uint8_t ShiftWS2811::ditherBits;
uint8_t ShiftWS2811::ditherCycle;
double ShiftWS2811::brightness = 100;
uint8_t ShiftWS2811::defaultPinList[8] = {6, 7, 8, 9, 34, 35, 36, 37};
uint16_t ShiftWS2811::stripLen;
void *ShiftWS2811::frontBuffer;
void *ShiftWS2811::backBuffer;
void *ShiftWS2811::drawBuffer;
uint8_t ShiftWS2811::params;
DMAChannel ShiftWS2811::dma;

static uint32_t numbytes; /* colour bytes per shift register output (3 or 4 per LED) */
static uint8_t numpins;
static uint8_t pinlist[SHIFTWS_MAX_CHAINS];
static uint8_t flexPinLo;        /* first FlexIO2 pin of the parallel data range */
static uint8_t flexPWidth;       /* SHIFTCFG[PWIDTH] */
static uint8_t flexShiftClkPin;  /* FlexIO2 pin indices of the timer outputs */
static uint8_t flexStorePin;
static uint8_t flexCommonPin;
static uint32_t shiftClkTimctl;  /* TIMCTL value that starts the shift clock */
static ShiftWSFillPlan plan;
static uint8_t zeroLUT[256];

static const uint32_t WORDS_PER_GROUP = 8; /* one DMA minor loop = SHIFTBUF0..7 */
static const uint32_t HALF_WORDS = SHIFTWS2811_BYTES_PER_DMA * SHIFTWS_BITS_PER_BYTE * SHIFTWS_SR_LEN; /* worst case: 16 words per bit */
DMAMEM static uint32_t bitdata[2][HALF_WORDS] __attribute__((used, aligned(32)));
DMAMEM static uint32_t zeroWords[WORDS_PER_GROUP] __attribute__((used, aligned(32)));
static DMASetting dmanext; /* loaded by scatter/gather when the running buffer completes */
static DMASetting dmazero; /* endless stream of zero words after the last data buffer */

static volatile uint32_t fillCol;  /* next LED byte column to convert */
static volatile uint8_t fillHalf;  /* bitdata half to convert into next */
static volatile bool transferring = false;
static volatile bool new_frame = false;
static volatile uint32_t underrunCount = 0;
static volatile uint32_t frameCount = 0;
static volatile uint32_t stallCount = 0;
static uint8_t initError = 0;

/* -------------------------------------------------------------- utilities */

static inline uint32_t umin(uint32_t a, uint32_t b) { return a < b ? a : b; }

/* FlexIO2 pin index of a Teensy pin, 0xFF if the pin is not on FlexIO2.  The
   FlexIO2 pins are exactly the GPIO2 pads (GPIO_B0_xx -> D[xx], GPIO_B1_xx ->
   D[16+xx]) with the same bit numbers, so the fast-GPIO bit is the index. */
static uint8_t flexio2Pin(uint8_t pin) {
  if (pin >= CORE_NUM_DIGITAL) return 0xFF;
  if (portOutputRegister(pin) != &GPIO7_DR) return 0xFF;
  return digitalPinToBit(pin);
}

static uint32_t gcd32(uint32_t a, uint32_t b) {
  while (b) {
    uint32_t t = a % b;
    a = b;
    b = t;
  }
  return a;
}

/* Program the otherwise unused video PLL (PLL5) and the FlexIO2 clock root so
   that FlexIO2 runs at exactly hz.  FlexIO3 shares this clock root.  Same
   register sequence the Teensy Audio library uses for the audio PLL. */
FLASHMEM static bool setupFlexIO2Clock(uint32_t hz) {
  static const uint8_t postDiv[3] = {1, 2, 4}, postCode[3] = {2, 1, 0}; /* PLL_VIDEO[POST_DIV_SELECT] */
  static const uint8_t vidDiv[3] = {1, 2, 4}, vidCode[3] = {0, 1, 3};   /* MISC2[VIDEO_DIV] */
  uint32_t bestTotal = 0;
  int bestP = 0, bestV = 0;
  uint32_t bestPred = 1, bestPodf = 1;
  for (int p = 0; p < 3; p++) {
    for (int v = 0; v < 3; v++) {
      for (uint32_t pred = 1; pred <= 8; pred++) {
        for (uint32_t podf = 1; podf <= 8; podf++) {
          uint32_t total = postDiv[p] * vidDiv[v] * pred * podf;
          uint64_t vco = (uint64_t)hz * total;
          if (vco < 650000000ull || vco > 1300000000ull) continue;
          if (bestTotal == 0 || total < bestTotal) {
            bestTotal = total;
            bestP = p;
            bestV = v;
            bestPred = pred;
            bestPodf = podf;
          }
        }
      }
    }
  }
  if (bestTotal == 0) return false;
  const uint32_t vco = hz * bestTotal;
  const uint32_t mult = vco / 24000000; /* 27..54 given the VCO range */
  const uint32_t rem = vco % 24000000;
  const uint32_t g = rem ? gcd32(rem, 24000000) : 1;
  const uint32_t num = rem ? rem / g : 0;
  const uint32_t den = rem ? 24000000 / g : 1;

  CCM_ANALOG_PLL_VIDEO = CCM_ANALOG_PLL_VIDEO_BYPASS | CCM_ANALOG_PLL_VIDEO_ENABLE |
                         CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(postCode[bestP]) | CCM_ANALOG_PLL_VIDEO_DIV_SELECT(mult);
  CCM_ANALOG_PLL_VIDEO_NUM = num;
  CCM_ANALOG_PLL_VIDEO_DENOM = den;
  CCM_ANALOG_PLL_VIDEO_CLR = CCM_ANALOG_PLL_VIDEO_POWERDOWN;
  uint32_t start = micros();
  while (!(CCM_ANALOG_PLL_VIDEO & CCM_ANALOG_PLL_VIDEO_LOCK)) {
    if (micros() - start > 20000) return false;
  }
  CCM_ANALOG_MISC2 = (CCM_ANALOG_MISC2 & ~CCM_ANALOG_MISC2_VIDEO_DIV(3)) | CCM_ANALOG_MISC2_VIDEO_DIV(vidCode[bestV]);
  CCM_ANALOG_PLL_VIDEO_CLR = CCM_ANALOG_PLL_VIDEO_BYPASS;

  CCM_CCGR3 &= ~CCM_CCGR3_FLEXIO2(3);
  CCM_CSCMR2 = (CCM_CSCMR2 & ~CCM_CSCMR2_FLEXIO2_CLK_SEL(3)) | CCM_CSCMR2_FLEXIO2_CLK_SEL(2); /* PLL5 */
  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_FLEXIO2_CLK_PRED(7) | CCM_CS1CDR_FLEXIO2_CLK_PODF(7))) |
               CCM_CS1CDR_FLEXIO2_CLK_PRED(bestPred - 1) | CCM_CS1CDR_FLEXIO2_CLK_PODF(bestPodf - 1);
  CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);
  return true;
}

/* Put the shifters in transmit mode.  Re-entering transmit mode sets all
   status flags, which is what makes the DMA prefill SHIFTBUF0..7 before the
   shift clock starts, so this is done at the start of every frame. */
static void configureShifters(void) {
  IMXRT_FLEXIO_t &f = IMXRT_FLEXIO2_S;
  for (int i = 0; i < NUM_SHIFTERS; i++) f.SHIFTCTL[i] = 0;
  for (int i = 0; i < NUM_SHIFTERS; i++) {
    f.SHIFTCFG[i] = FLEXIO_SHIFTCFG_PWIDTH(flexPWidth) | (i < NUM_SHIFTERS - 1 ? FLEXIO_SHIFTCFG_INSRC : 0) |
                    FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);
    f.SHIFTCTL[i] = FLEXIO_SHIFTCTL_TIMSEL(TMR_SHIFT) | FLEXIO_SHIFTCTL_TIMPOL /* shift on the falling edge */ |
                    FLEXIO_SHIFTCTL_PINCFG(i == 0 ? 3 : 0) | FLEXIO_SHIFTCTL_PINSEL(flexPinLo) | FLEXIO_SHIFTCTL_SMOD(2);
  }
}

FLASHMEM static void configureTimers(uint32_t numBits) {
  IMXRT_FLEXIO_t &f = IMXRT_FLEXIO2_S;
  for (int i = 0; i < 8; i++) f.TIMCTL[i] = 0;

  /* one-shot delays: 16-bit counter, output high while running, disabled at compare */
  f.TIMCMP[TMR_DELAY_COMMON] = DELAY_COMMON_CYC - 1;
  f.TIMCFG[TMR_DELAY_COMMON] = FLEXIO_TIMCFG_TIMOUT(0) | FLEXIO_TIMCFG_TIMDEC(0) | FLEXIO_TIMCFG_TIMRST(0) |
                               FLEXIO_TIMCFG_TIMDIS(2) | FLEXIO_TIMCFG_TIMENA(1) /* with timer 0 */;
  f.TIMCTL[TMR_DELAY_COMMON] = FLEXIO_TIMCTL_TRGSRC | FLEXIO_TIMCTL_PINCFG(0) | FLEXIO_TIMCTL_TIMOD(3);

  f.TIMCMP[TMR_DELAY_STORE] = DELAY_STORE_CYC - 1;
  f.TIMCFG[TMR_DELAY_STORE] = FLEXIO_TIMCFG_TIMOUT(0) | FLEXIO_TIMCFG_TIMDEC(0) | FLEXIO_TIMCFG_TIMRST(0) |
                              FLEXIO_TIMCFG_TIMDIS(2) | FLEXIO_TIMCFG_TIMENA(1) /* with timer 1 */;
  f.TIMCTL[TMR_DELAY_STORE] = FLEXIO_TIMCTL_TRGSRC | FLEXIO_TIMCTL_PINCFG(0) | FLEXIO_TIMCTL_TIMOD(3);

  /* frame end: count both edges of the STORE pin, two per bit */
  f.TIMCMP[TMR_END] = 2 * numBits - 1;
  f.TIMCFG[TMR_END] = FLEXIO_TIMCFG_TIMOUT(0) | FLEXIO_TIMCFG_TIMDEC(2) /* pin edges */ | FLEXIO_TIMCFG_TIMRST(0) |
                      FLEXIO_TIMCFG_TIMDIS(2) | FLEXIO_TIMCFG_TIMENA(6) /* rising edge of timer 1 */;
  f.TIMCTL[TMR_END] = FLEXIO_TIMCTL_TRGSEL(TRG_TIMER(TMR_DELAY_COMMON)) | FLEXIO_TIMCTL_TRGSRC |
                      FLEXIO_TIMCTL_PINCFG(0) | FLEXIO_TIMCTL_PINSEL(flexStorePin) | FLEXIO_TIMCTL_TIMOD(3);

  /* reset gap: started by the falling edge of the frame end timer */
  f.TIMCMP[TMR_GAP] = GAP_CYC - 1;
  f.TIMCFG[TMR_GAP] = FLEXIO_TIMCFG_TIMOUT(0) | FLEXIO_TIMCFG_TIMDEC(0) | FLEXIO_TIMCFG_TIMRST(0) |
                      FLEXIO_TIMCFG_TIMDIS(2) | FLEXIO_TIMCFG_TIMENA(6);
  f.TIMCTL[TMR_GAP] = FLEXIO_TIMCTL_TRGSEL(TRG_TIMER(TMR_END)) | FLEXIO_TIMCTL_TRGPOL | FLEXIO_TIMCTL_TRGSRC |
                      FLEXIO_TIMCTL_PINCFG(0) | FLEXIO_TIMCTL_TIMOD(3);

  /* COMMON waveform: PWM started by the falling edge of one-shot 1 */
  f.TIMCMP[TMR_COMMON] = ((COMMON_LOW_CYC - 1) << 8) | (COMMON_HIGH_CYC - 1);
  f.TIMCFG[TMR_COMMON] = FLEXIO_TIMCFG_TIMOUT(0) | FLEXIO_TIMCFG_TIMDEC(0) | FLEXIO_TIMCFG_TIMRST(0) |
                         FLEXIO_TIMCFG_TIMDIS(1) /* with timer 5 */ | FLEXIO_TIMCFG_TIMENA(6);
  f.TIMCTL[TMR_COMMON] = FLEXIO_TIMCTL_TRGSEL(TRG_TIMER(TMR_DELAY_COMMON)) | FLEXIO_TIMCTL_TRGPOL | FLEXIO_TIMCTL_TRGSRC |
                         FLEXIO_TIMCTL_PINCFG(3) | FLEXIO_TIMCTL_PINSEL(flexCommonPin) | FLEXIO_TIMCTL_TIMOD(2);

  /* STORE waveform: PWM started by the falling edge of one-shot 2 */
  f.TIMCMP[TMR_STORE] = ((STORE_LOW_CYC - 1) << 8) | (STORE_HIGH_CYC - 1);
  f.TIMCFG[TMR_STORE] = FLEXIO_TIMCFG_TIMOUT(0) | FLEXIO_TIMCFG_TIMDEC(0) | FLEXIO_TIMCFG_TIMRST(0) |
                        FLEXIO_TIMCFG_TIMDIS(1) /* with timer 6 */ | FLEXIO_TIMCFG_TIMENA(6);
  f.TIMCTL[TMR_STORE] = FLEXIO_TIMCTL_TRGSEL(TRG_TIMER(TMR_DELAY_STORE)) | FLEXIO_TIMCTL_TRGPOL | FLEXIO_TIMCTL_TRGSRC |
                        FLEXIO_TIMCTL_PINCFG(3) | FLEXIO_TIMCTL_PINSEL(flexStorePin) | FLEXIO_TIMCTL_TIMOD(2);

  /* shift clock: dual 8-bit baud mode, reload every 8 words; starts when
     SHIFTBUF7 has been written (all buffers full) and runs until stopped */
  const uint32_t shiftsPerReload = WORDS_PER_GROUP * plan.layout.shiftsPerWord;
  f.TIMCMP[TMR_SHIFT] = ((shiftsPerReload * 2 - 1) << 8) | SHIFTWS2811_SHIFT_DIV;
  f.TIMCFG[TMR_SHIFT] = FLEXIO_TIMCFG_TIMOUT(1) /* low when enabled */ | FLEXIO_TIMCFG_TIMDEC(0) | FLEXIO_TIMCFG_TIMRST(0) |
                        FLEXIO_TIMCFG_TIMDIS(0) | FLEXIO_TIMCFG_TIMENA(2) /* trigger high */;
  shiftClkTimctl = FLEXIO_TIMCTL_TRGSEL(TRG_SHIFTER(NUM_SHIFTERS - 1)) | FLEXIO_TIMCTL_TRGPOL /* flag low = full */ |
                   FLEXIO_TIMCTL_TRGSRC | FLEXIO_TIMCTL_PINCFG(3) | FLEXIO_TIMCTL_PINSEL(flexShiftClkPin) |
                   FLEXIO_TIMCTL_TIMOD(1);

  f.TIMIEN = (1 << TMR_END) | (1 << TMR_GAP);
}

/* TCD for one conversion buffer: `words` words to SHIFTBUF0..7 in 32-byte
   groups, then scatter/gather to `next`. */
static void setDataTcd(DMABaseClass::TCD_t *t, const uint32_t *src, uint32_t words, const void *next) {
  t->SADDR = src;
#if SHIFTWS2811_DMA_BURST
  t->SOFF = 32;
  t->ATTR = DMA_TCD_ATTR_SSIZE(5) /* 32-byte burst */ | DMA_TCD_ATTR_DMOD(5) | DMA_TCD_ATTR_DSIZE(2);
#else
  t->SOFF = 4;
  t->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DMOD(5) | DMA_TCD_ATTR_DSIZE(2);
#endif
  t->NBYTES_MLOFFNO = 4 * WORDS_PER_GROUP;
  t->SLAST = 0;
  t->DADDR = (volatile void *)&IMXRT_FLEXIO2_S.SHIFTBUF[0];
  t->DOFF = 4;
  t->CITER_ELINKNO = words / WORDS_PER_GROUP;
  t->BITER_ELINKNO = words / WORDS_PER_GROUP;
  t->DLASTSGA = (int32_t)(uint32_t)next;
  t->CSR = DMA_TCD_CSR_ESG | DMA_TCD_CSR_INTMAJOR;
}

FLASHMEM static void setZeroTcd(DMABaseClass::TCD_t *t) {
  setDataTcd(t, zeroWords, WORDS_PER_GROUP * 0x7FFF, t);
  t->SOFF = 0; /* read the same 8 zero words forever */
  t->CSR = DMA_TCD_CSR_ESG;
}

#if SHIFTWS2811_DMA_PRIORITY
/* Swap fixed priorities with the top channel of our group so that pending
   requests of other channels do not delay the SHIFTBUF refill. */
FLASHMEM static void raiseDmaPriority(uint8_t ch) {
  volatile uint8_t *dchpri = &DMA_DCHPRI3; /* byte swapped within each word */
  const uint8_t top = (ch & 0x10) | 0x0F;
  if (top == ch) return;
  const uint32_t mine = (ch & 0x1C) | (3 - (ch & 0x03));
  const uint32_t theirs = (top & 0x1C) | (3 - (top & 0x03));
  const uint8_t myPri = dchpri[mine] & 0x0F;
  const uint8_t theirPri = dchpri[theirs] & 0x0F;
  dchpri[theirs] = (dchpri[theirs] & (DMA_DCHPRI_ECP | DMA_DCHPRI_DPA)) | myPri;
  dchpri[mine] = theirPri; /* not preemptable, may preempt */
}
#endif

/* ---------------------------------------------------------------- public */

FLASHMEM ShiftWS2811::ShiftWS2811(uint32_t numPerStrip, void *frontBuf, void *backBuf, void *drawBuf, uint8_t config, uint8_t numPins,
                         const uint8_t *pinList, bool gammaCorr, byte ditBits) {
  stripLen = numPerStrip;
  frontBuffer = frontBuf;
  backBuffer = backBuf;
  drawBuffer = drawBuf;
  params = config;
  if (numPins > SHIFTWS_MAX_CHAINS) numPins = SHIFTWS_MAX_CHAINS;
  numpins = numPins;
  memcpy(pinlist, pinList, numpins);
  gammaCorrection = gammaCorr;
  setDitherBits(ditBits);
}

FLASHMEM void ShiftWS2811::begin(uint32_t numPerStrip, void *frontBuf, void *backBuf, void *drawBuf, uint8_t config, uint8_t numPins,
                        const uint8_t *pinList, bool gammaCorr, byte ditBits) {
  stripLen = numPerStrip;
  frontBuffer = frontBuf;
  backBuffer = backBuf;
  drawBuffer = drawBuf;
  params = config;
  if (numPins > SHIFTWS_MAX_CHAINS) numPins = SHIFTWS_MAX_CHAINS;
  numpins = numPins;
  memcpy(pinlist, pinList, numpins);
  gammaCorrection = gammaCorr;
  setDitherBits(ditBits);
  begin();
}

int ShiftWS2811::numPixels(void) { return stripLen * numpins * SHIFTWS_SR_LEN; }

FLASHMEM void ShiftWS2811::begin(void) {
  initError = 0;
  transferring = false;
  new_frame = false;
  setBrightness(brightness);
  numbytes = ((params & 0x1F) < 6) ? stripLen * 3 : stripLen * 4;
  const uint32_t numBits = numbytes * SHIFTWS_BITS_PER_BYTE;

  if (numpins == 0 || numpins > SHIFTWS_MAX_CHAINS) {
    initError = ERR_NUM_PINS;
    return;
  }
  if (numBits == 0 || 2 * numBits - 1 > 0xFFFF) {
    initError = ERR_STRIP_LENGTH;
    return;
  }

  /* map the pins onto FlexIO2 */
  flexShiftClkPin = flexio2Pin(SHIFTWS2811_PIN_SHIFT_CLK);
  flexStorePin = flexio2Pin(SHIFTWS2811_PIN_STORE);
  flexCommonPin = flexio2Pin(SHIFTWS2811_PIN_COMMON);
  uint8_t fx[SHIFTWS_MAX_CHAINS];
  uint8_t lo = 31, hi = 0;
  for (uint32_t i = 0; i < numpins; i++) {
    fx[i] = flexio2Pin(pinlist[i]);
    if (fx[i] == 0xFF) {
      initError = ERR_PIN_NOT_FLEXIO;
      return;
    }
    if (fx[i] < lo) lo = fx[i];
    if (fx[i] > hi) hi = fx[i];
  }
  if (flexShiftClkPin == 0xFF || flexStorePin == 0xFF || flexCommonPin == 0xFF) {
    initError = ERR_PIN_NOT_FLEXIO;
    return;
  }
  flexPinLo = lo;
  flexPWidth = (hi > lo) ? (hi - lo) : 1; /* a single pin still needs a 4-bit shift */
  const uint8_t rangeHi = lo + flexPWidth;
  const uint8_t timerPins[3] = {flexShiftClkPin, flexStorePin, flexCommonPin};
  for (int t = 0; t < 3; t++) {
    if (timerPins[t] >= lo && timerPins[t] <= rangeHi) {
      initError = ERR_PIN_RANGE;
      return;
    }
    for (uint32_t i = 0; i < numpins; i++) {
      if (fx[i] == timerPins[t]) {
        initError = ERR_PIN_RANGE;
        return;
      }
    }
  }
  const uint8_t shiftWidth = flexPWidth <= 3 ? 4 : flexPWidth <= 7 ? 8 : flexPWidth <= 15 ? 16 : 32;
  uint8_t lanes[SHIFTWS_MAX_CHAINS];
  for (uint32_t i = 0; i < numpins; i++) lanes[i] = fx[i] - lo;
  plan.layout = shiftws_makeLayout(shiftWidth);
  plan.numGroups = (numpins + SHIFTWS_CHAINS_PER_GROUP - 1) / SHIFTWS_CHAINS_PER_GROUP;
  shiftws_buildSpread(&plan);
  shiftws_buildExpand(&plan, lanes, numpins);
  memset(zeroLUT, 0, sizeof(zeroLUT));

  /* conversion buffers and the zero group live in OCRAM for the DMA */
  memset(bitdata, 0, sizeof(bitdata));
  arm_dcache_flush_delete(bitdata, sizeof(bitdata));
  memset(zeroWords, 0, sizeof(zeroWords));
  arm_dcache_flush_delete(zeroWords, sizeof(zeroWords));

  /* FlexIO2 clock and module */
  if (!setupFlexIO2Clock(FLEXIO_HZ)) {
    initError = ERR_PLL;
    return;
  }
  IMXRT_FLEXIO_t &f = IMXRT_FLEXIO2_S;
  f.CTRL = FLEXIO_CTRL_SWRST;
  f.CTRL = 0;
  asm volatile("dsb");
  if ((f.PARAM & 0xFF) < (uint32_t)NUM_SHIFTERS || ((f.PARAM >> 8) & 0xFF) < 8) {
    initError = ERR_FLEXIO;
    return;
  }
  f.CTRL = FLEXIO_CTRL_FLEXEN;
  configureShifters();
  configureTimers(numBits);
  f.SHIFTSDEN = 1; /* shifter 0 status flag -> DMA request */

  /* pins: everything on FlexIO2 is mux mode ALT4 */
  for (uint32_t i = 0; i < numpins; i++) *(portConfigRegister(pinlist[i])) = 4;
  *(portConfigRegister(SHIFTWS2811_PIN_SHIFT_CLK)) = 4;
  *(portConfigRegister(SHIFTWS2811_PIN_STORE)) = 4;
  *(portConfigRegister(SHIFTWS2811_PIN_COMMON)) = 4;

  /* DMA channel: FlexIO2 shifter 0 request -> 32-byte groups into SHIFTBUF0..7 */
  dma.begin();
  if (dma.TCD == nullptr || dma.channel >= DMA_NUM_CHANNELS) {
    initError = ERR_DMA;
    return;
  }
  dma.disable();
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_FLEXIO2_REQUEST0);
  dma.attachInterrupt(isr, 64);
#if SHIFTWS2811_DMA_PRIORITY
  raiseDmaPriority(dma.channel);
#endif
  setZeroTcd(dmazero.TCD);

  attachInterruptVector(IRQ_FLEXIO2, flexisr);
  NVIC_SET_PRIORITY(IRQ_FLEXIO2, 96);
  NVIC_ENABLE_IRQ(IRQ_FLEXIO2);

  /* pixel buffers */
  uint32_t bufsize = numbytes * numpins * SHIFTWS_SR_LEN;
  memset(frontBuffer, 0, bufsize);
  if (drawBuffer) {
    memset(drawBuffer, 0, bufsize);
  } else {
    drawBuffer = frontBuffer;
  }
}

FLASHMEM void ShiftWS2811::setBrightness(double bri) {
  brightness = max(bri, 0);
  gammaLUTCalc(brightness, gammaCorrection);
}

FLASHMEM byte ShiftWS2811::setDitherBits(byte ditBits) {
  ditherBits = ditBits;
  if (ditherBits == 255) {
    ditherBits = 0;
    float frameTime = stripLen * LED_TIME;
    while (frameTime * 2 < (1. / 33)) {
      frameTime *= 2;
      ditherBits++;
    }
  }
  if (ditherBits > MAX_DITHER_BITS) ditherBits = MAX_DITHER_BITS;
  ditherCycle = 0;
  return ditherBits;
}

uint32_t ShiftWS2811::underruns(void) { return underrunCount; }
uint32_t ShiftWS2811::frames(void) { return frameCount; }
uint32_t ShiftWS2811::stalls(void) { return stallCount; }
uint8_t ShiftWS2811::error(void) { return initError; }
uint32_t ShiftWS2811::bitPeriodNs(void) { return SHIFTWS2811_BIT_NS; }
uint32_t ShiftWS2811::frameTimeUs(void) {
  return (uint32_t)(((uint64_t)numbytes * 8 + 1) * SHIFTWS2811_BIT_NS / 1000) + SHIFTWS2811_RESET_US;
}

void ShiftWS2811::fillAllBits(uint32_t *dest, uint32_t index, uint32_t count) {
  shiftws_fillColumns(&plan, dest, index, count);
  arm_dcache_flush_delete(dest, count * SHIFTWS_BITS_PER_BYTE * plan.layout.wordsPerBit * sizeof(uint32_t));
}

/* Stop everything and re-arm the timers.  Used when a frame never finished,
   which can only happen if the FlexIO chain is not behaving as designed. */
void ShiftWS2811::restartEngine(void) {
  IMXRT_FLEXIO_t &f = IMXRT_FLEXIO2_S;
  f.TIMCTL[TMR_SHIFT] = 0;
  dma.disable();
  configureTimers(numbytes * SHIFTWS_BITS_PER_BYTE);
  f.SHIFTERR = 0xFF;
  f.TIMSTAT = 0xFF;
  transferring = false;
}

void ShiftWS2811::show(void) {
  if (initError) return;
  // wait till the last new frame has been attended to; if the frame engine
  // never comes back, restart it instead of hanging (see stalls())
  const uint32_t start = micros();
  const uint32_t limit = 4 * frameTimeUs() + 10000;
  while (new_frame) {
    if (micros() - start > limit) {
      stallCount++;
      restartEngine();
      break;
    }
  }
  if (drawBuffer != backBuffer) memcpy(backBuffer, drawBuffer, numbytes * numpins * SHIFTWS_SR_LEN);
  new_frame = true;
  if (!transferring) transfer();
}

int ShiftWS2811::busy(void) { return transferring; }

/* Start one frame.  Called from show() when idle, otherwise from the FlexIO
   interrupt at the end of the reset gap. */
void ShiftWS2811::transfer(void) {
  if (new_frame) {  // point frontBuffer to the newly copied frame waiting in backBuffer
    void *temp = backBuffer;
    backBuffer = frontBuffer;
    frontBuffer = temp;
    new_frame = false;
  }

  /* dithering: advance one step per frame, offset by shift register output */
  const uint32_t ditherMask = (1u << ditherBits) - 1;
  ditherCycle = (ditherCycle + 1) & ditherMask;
  const uint8_t *base = (const uint8_t *)frontBuffer;
  for (uint32_t idx = 0; idx < SHIFTWS_MAX_CHAINS * SHIFTWS_SR_LEN; idx++) {
    if (idx < (uint32_t)numpins * SHIFTWS_SR_LEN) {
      plan.pixels[idx] = base + idx * numbytes;
      plan.lut[idx] = gammaLUT + (((ditherCycle + idx + 1) & ditherMask) << 8);
    } else {
      plan.pixels[idx] = base;
      plan.lut[idx] = zeroLUT;
    }
  }

  /* convert the first one or two buffers */
  const uint32_t wordsPerColumn = SHIFTWS_BITS_PER_BYTE * plan.layout.wordsPerBit;
  fillCol = 0;
  fillHalf = 0;
  uint32_t count = umin(numbytes, SHIFTWS2811_BYTES_PER_DMA);
  fillAllBits(bitdata[0], 0, count);
  fillCol = count;
  setDataTcd(dma.TCD, bitdata[0], count * wordsPerColumn, dmanext.TCD);
  if (fillCol < numbytes) {
    count = umin(numbytes - fillCol, SHIFTWS2811_BYTES_PER_DMA);
    fillAllBits(bitdata[1], fillCol, count);
    fillCol += count;
    setDataTcd(dmanext.TCD, bitdata[1], count * wordsPerColumn, fillCol < numbytes ? (void *)dmanext.TCD : (void *)dmazero.TCD);
  } else {
    dmanext = dmazero;
  }

  /* arm FlexIO: fresh shifters (status flags set -> DMA prefill), clear stale flags */
  IMXRT_FLEXIO_t &f = IMXRT_FLEXIO2_S;
  configureShifters();
  f.SHIFTERR = 0xFF;
  f.TIMSTAT = 0xFF;

  dma.clearComplete();
  dma.clearInterrupt();
  dma.enable();  /* prefill of SHIFTBUF0..7 happens now */

  /* the shift clock starts as soon as SHIFTBUF7 is written; timers 1, 2 and
     5 start with it and the waveform timers follow at their fixed delays */
  f.TIMCTL[TMR_SHIFT] = shiftClkTimctl;
  transferring = true;
}

/* DMA interrupt: one conversion buffer has been consumed, refill it. */
void ShiftWS2811::isr(void) {
  dma.clearInterrupt();
  asm volatile("dsb");
  if (fillCol >= numbytes) return;  // the zero stream is running now
  uint32_t *dest = bitdata[fillHalf];
  const uint32_t count = umin(numbytes - fillCol, SHIFTWS2811_BYTES_PER_DMA);
  fillAllBits(dest, fillCol, count);
  fillCol += count;
  fillHalf ^= 1;
  setDataTcd(dmanext.TCD, dest, count * SHIFTWS_BITS_PER_BYTE * plan.layout.wordsPerBit,
             fillCol < numbytes ? (void *)dmanext.TCD : (void *)dmazero.TCD);
}

/* FlexIO interrupt: end of frame (timer 5) and end of the reset gap (timer 4). */
void ShiftWS2811::flexisr(void) {
  IMXRT_FLEXIO_t &f = IMXRT_FLEXIO2_S;
  const uint32_t status = f.TIMSTAT;
  if (status & (1 << TMR_END)) {
    f.TIMSTAT = 1 << TMR_END;
    f.TIMCTL[TMR_SHIFT] = 0;  // stop the shift clock; STORE and COMMON already stopped in hardware
    dma.disable();
    if (f.SHIFTERR & 0xFF) {
      underrunCount++;
      f.SHIFTERR = 0xFF;
    }
    frameCount++;
  }
  if (status & (1 << TMR_GAP)) {
    f.TIMSTAT = 1 << TMR_GAP;
    if ((gammaCorrection && ditherBits > 0) || new_frame)  // keep dithering the current frame, or show the new one
      transfer();
    else
      transferring = false;
  }
  asm volatile("dsb");
}

// For Teensy 4.x, the pixel data is stored in ordinary RGB format.  Translation
// from 24 bit color to the shift register bit stream is done on-the-fly.

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
