/*  ShiftWS2811 - bit transposition core
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

/* This header has no hardware dependencies and is valid C99 as well as C++, so
   it can be compiled and tested on a PC (see test/host_fill_test.c).  It
   converts ordinary per-LED colour bytes into the word stream that the FlexIO
   shifters clock into the 74HC595 chains.

   Stream layout
   -------------
   One WS2811 bit period consists of SHIFTWS_SR_LEN (16) shift clocks.  On
   every shift clock the FlexIO shifter outputs `shiftWidth` bits in parallel
   (one bit per FlexIO pin of the selected range); the data pin of shift
   register chain i is bit `lane[i]` of that group.  Shift k (k = 0 first)
   carries the bit for shift register output j = 15 - k, because the first bit
   clocked into a chain ends up at its far end.

   Groups are packed LSB first into 32-bit words: shiftsPerWord = 32/shiftWidth
   consecutive shifts share one word, the first shift in the lowest bits.  A
   WS2811 bit therefore occupies wordsPerBit = 16 / shiftsPerWord words and one
   LED byte (8 WS2811 bits, MSB first) occupies 8 * wordsPerBit words.

   Pixel data
   ----------
   The colour bytes of one shift register output form one contiguous stream
   (byte c is the c-th colour byte of that LED strip).  Streams of the 16
   outputs of a chain follow each other, and the chains follow each other, so
   the stream of (chain i, output j) starts `i * chainStride + j * numbytes`
   bytes into the buffer.  The fill reads the 8 chains of a group with a fixed
   stride, so the buffer must be readable for numGroups * 8 chains even when
   fewer pins are used (the surplus chains contribute nothing because their
   lane bits are absent from the expand table).

   Gamma/dither tables: one 256-byte table per shift register output, one for
   the even and one for the odd chains (this is exactly the phase pattern
   `(cycle + 16*chain + output + 1) mod 2^ditherBits` of the original driver:
   16*chain mod 32 only depends on the chain's parity).

   Algorithm
   ---------
   For every (output j, group, column c) the 8 corrected pixel bytes of the
   chains in the group are looked up in a 256 x 64-bit table that spreads bit
   (7 - s) of the byte to bit 8*s, and OR-ed at bit offset i (the chain index).
   Byte s of the 64-bit accumulator then holds bit (7 - s) of all 8 chains,
   chain i at bit i, and a 256 x 32-bit table maps it onto the lane positions
   of the word of WS2811 bit s.  The two 32-bit halves of the accumulator are
   handled separately because a spread value shifted by at most 7 never
   crosses a word boundary (its bits sit at 8*s), which keeps everything in
   plain 32-bit registers.

   Cost on Cortex-M7: about 85 instructions and 40 memory accesses per column
   and output (8 pixel bytes), i.e. about 1400 instructions per LED byte
   column, all from zero-wait-state DTCM when the tables and pixels live there.
   The previous version of this file needed 3000+ instructions per column
   because of per-chain pointer tables and 64-bit shifts. */

#ifndef SHIFTWS2811_FILL_H
#define SHIFTWS2811_FILL_H

#include <stdint.h>
#include <string.h>

#define SHIFTWS_MAX_CHAINS 16 /* data pins, processed in groups of 8 */
#define SHIFTWS_CHAINS_PER_GROUP 8
#define SHIFTWS_MAX_GROUPS (SHIFTWS_MAX_CHAINS / SHIFTWS_CHAINS_PER_GROUP)
#define SHIFTWS_SR_LEN 16 /* outputs per shift register chain */
#define SHIFTWS_BITS_PER_BYTE 8

#if defined(__GNUC__)
#define SHIFTWS_INLINE static inline __attribute__((always_inline))
#else
#define SHIFTWS_INLINE static inline
#endif

typedef struct {
  uint8_t shiftWidth;    /* bits per shift clock: 4, 8, 16 or 32 */
  uint8_t shiftsPerWord; /* 32 / shiftWidth */
  uint8_t wordsPerBit;   /* SHIFTWS_SR_LEN / shiftsPerWord */
} ShiftWSLayout;

typedef struct {
  /* spread[x]: bit (7 - s) of x is moved to bit 8*s, s = 0..7.  First member
     so that the plan pointer itself addresses it. */
  uint64_t spread[256];
  /* Maps 8 chain bits (bit i = chain g*8+i) to their lane positions. */
  uint32_t expand[SHIFTWS_MAX_GROUPS][256];
  /* Stream of the first chain of group g at output j (see "Pixel data"). */
  const uint8_t *pixels[SHIFTWS_MAX_GROUPS][SHIFTWS_SR_LEN];
  /* 256-entry gamma/dither table for output j, [0] even chains, [1] odd. */
  const uint8_t *lut[2][SHIFTWS_SR_LEN];
  uint32_t chainStride; /* bytes from the stream of chain i to that of chain i+1 */
  ShiftWSLayout layout;
  uint8_t numGroups;
} ShiftWSFillPlan;

SHIFTWS_INLINE ShiftWSLayout shiftws_makeLayout(uint8_t shiftWidth) {
  ShiftWSLayout l;
  l.shiftWidth = shiftWidth;
  l.shiftsPerWord = (uint8_t)(32 / shiftWidth);
  l.wordsPerBit = (uint8_t)(SHIFTWS_SR_LEN / l.shiftsPerWord);
  return l;
}

static inline void shiftws_buildSpread(ShiftWSFillPlan *p) {
  int x, s;
  for (x = 0; x < 256; x++) {
    uint64_t r = 0;
    for (s = 0; s < 8; s++) {
      if ((x >> (7 - s)) & 1) r |= (uint64_t)1 << (8 * s);
    }
    p->spread[x] = r;
  }
}

/* lanes[i] = bit position of chain i within a shift group (0 .. shiftWidth-1). */
static inline void shiftws_buildExpand(ShiftWSFillPlan *p, const uint8_t *lanes, uint8_t numChains) {
  int g, v, i;
  for (g = 0; g < SHIFTWS_MAX_GROUPS; g++) {
    for (v = 0; v < 256; v++) {
      uint32_t out = 0;
      for (i = 0; i < SHIFTWS_CHAINS_PER_GROUP; i++) {
        int chain = g * SHIFTWS_CHAINS_PER_GROUP + i;
        if (chain < numChains && ((v >> i) & 1)) out |= (uint32_t)1 << lanes[chain];
      }
      p->expand[g][v] = out;
    }
  }
}

/* One chain: correct the pixel byte, spread its bits into the accumulator at
   bit offset i and step to the stream of the next chain. */
#define SHIFTWS_CHAIN(i, LUT)                  \
  do {                                         \
    const uint64_t sp_ = p->spread[(LUT)[*q]]; \
    q += stride;                               \
    accLo |= (uint32_t)sp_ << (i);             \
    accHi |= (uint32_t)(sp_ >> 32) << (i);     \
  } while (0)

/* Convert `count` columns of shift register output j.  `direct` (a compile
   time constant at every call site) selects the fast path for 32-bit shifts
   with a single group, where every destination word belongs to exactly one
   (column, output) pair and is written once without a read-modify-write.
   Otherwise the words are OR-ed into a zeroed destination. */
SHIFTWS_INLINE void shiftws_fillOutput(const ShiftWSFillPlan *p, uint32_t *dest, uint32_t col, uint32_t count,
                                       uint32_t j, int direct) {
  const uint32_t wpb = direct ? (uint32_t)SHIFTWS_SR_LEN : p->layout.wordsPerBit;
  const uint32_t spw = direct ? 1u : p->layout.shiftsPerWord;
  const uint32_t k = SHIFTWS_SR_LEN - 1 - j; /* shift that carries output j */
  const uint32_t bitOff = (k % spw) * p->layout.shiftWidth;
  const uint32_t stride = p->chainStride;
  const uint8_t *const lutE = p->lut[0][j];
  const uint8_t *const lutO = p->lut[1][j];
  const uint32_t numGroups = direct ? 1u : p->numGroups;
  uint32_t g;

  for (g = 0; g < numGroups; g++) {
    const uint8_t *pix = p->pixels[g][j] + col;
    const uint32_t *const ex = p->expand[g];
    uint32_t *dst = dest + k / spw;
    uint32_t c;
    for (c = 0; c < count; c++) {
      const uint8_t *q = pix;
      uint32_t accLo = 0, accHi = 0;
      SHIFTWS_CHAIN(0, lutE);
      SHIFTWS_CHAIN(1, lutO);
      SHIFTWS_CHAIN(2, lutE);
      SHIFTWS_CHAIN(3, lutO);
      SHIFTWS_CHAIN(4, lutE);
      SHIFTWS_CHAIN(5, lutO);
      SHIFTWS_CHAIN(6, lutE);
      SHIFTWS_CHAIN(7, lutO);
      if (direct) {
        dst[0 * wpb] = ex[accLo & 0xFF];
        dst[1 * wpb] = ex[(accLo >> 8) & 0xFF];
        dst[2 * wpb] = ex[(accLo >> 16) & 0xFF];
        dst[3 * wpb] = ex[accLo >> 24];
        dst[4 * wpb] = ex[accHi & 0xFF];
        dst[5 * wpb] = ex[(accHi >> 8) & 0xFF];
        dst[6 * wpb] = ex[(accHi >> 16) & 0xFF];
        dst[7 * wpb] = ex[accHi >> 24];
      } else {
        dst[0 * wpb] |= ex[accLo & 0xFF] << bitOff;
        dst[1 * wpb] |= ex[(accLo >> 8) & 0xFF] << bitOff;
        dst[2 * wpb] |= ex[(accLo >> 16) & 0xFF] << bitOff;
        dst[3 * wpb] |= ex[accLo >> 24] << bitOff;
        dst[4 * wpb] |= ex[accHi & 0xFF] << bitOff;
        dst[5 * wpb] |= ex[(accHi >> 8) & 0xFF] << bitOff;
        dst[6 * wpb] |= ex[(accHi >> 16) & 0xFF] << bitOff;
        dst[7 * wpb] |= ex[accHi >> 24] << bitOff;
      }
      pix++;
      dst += SHIFTWS_BITS_PER_BYTE * wpb;
    }
  }
}

#undef SHIFTWS_CHAIN

/* Convert `count` LED byte columns starting at column `col` into `dest`.
   dest must hold count * 8 * wordsPerBit words. */
static inline void shiftws_fillColumns(const ShiftWSFillPlan *p, uint32_t *dest, uint32_t col, uint32_t count) {
  uint32_t j;
  if (p->layout.wordsPerBit == SHIFTWS_SR_LEN && p->numGroups == 1) {
    for (j = 0; j < (uint32_t)SHIFTWS_SR_LEN; j++) shiftws_fillOutput(p, dest, col, count, j, 1);
  } else {
    memset(dest, 0, count * SHIFTWS_BITS_PER_BYTE * p->layout.wordsPerBit * sizeof(uint32_t));
    for (j = 0; j < (uint32_t)SHIFTWS_SR_LEN; j++) shiftws_fillOutput(p, dest, col, count, j, 0);
  }
}

#endif /* SHIFTWS2811_FILL_H */
