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

   Algorithm
   ---------
   For every (column c, shift k) the 8 (gamma/dither corrected) pixel bytes of
   the chains in a group are spread into a 64-bit accumulator so that byte s of
   the accumulator holds bit (7 - s) of all 8 chains.  Each of those 8 bytes is
   then mapped through a 256-entry table onto the lane positions of the chains
   and OR-ed into the word of WS2811 bit s.  This touches every destination
   word once with 8 loads + 8 stores instead of the 64 test-and-or operations
   of a bit-by-bit approach. */

#ifndef SHIFTWS2811_FILL_H
#define SHIFTWS2811_FILL_H

#include <stdint.h>
#include <string.h>

#define SHIFTWS_MAX_CHAINS 16 /* data pins, processed in groups of 8 */
#define SHIFTWS_CHAINS_PER_GROUP 8
#define SHIFTWS_MAX_GROUPS (SHIFTWS_MAX_CHAINS / SHIFTWS_CHAINS_PER_GROUP)
#define SHIFTWS_SR_LEN 16 /* outputs per shift register chain */
#define SHIFTWS_BITS_PER_BYTE 8

typedef struct {
  uint8_t shiftWidth;    /* bits per shift clock: 4, 8, 16 or 32 */
  uint8_t shiftsPerWord; /* 32 / shiftWidth */
  uint8_t wordsPerBit;   /* SHIFTWS_SR_LEN / shiftsPerWord */
} ShiftWSLayout;

typedef struct {
  ShiftWSLayout layout;
  uint8_t numGroups;
  /* Per (chain, shift register output), index chain * SHIFTWS_SR_LEN + j: the
     byte stream of that LED column (byte c of the stream is the c-th colour
     byte of the strip) and the 256 entry gamma/dither table to apply.  Unused
     chains must point at a valid stream and at an all-zero table. */
  const uint8_t *pixels[SHIFTWS_MAX_CHAINS * SHIFTWS_SR_LEN];
  const uint8_t *lut[SHIFTWS_MAX_CHAINS * SHIFTWS_SR_LEN];
  /* Maps 8 chain bits (bit i = chain g*8+i) to their lane positions. */
  uint32_t expand[SHIFTWS_MAX_GROUPS][256];
  /* spread[x]: bit (7 - s) of x is moved to bit 8*s, s = 0..7. */
  uint64_t spread[256];
} ShiftWSFillPlan;

static inline ShiftWSLayout shiftws_makeLayout(uint8_t shiftWidth) {
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

/* Convert `count` LED byte columns starting at column `col` into `dest`.
   dest must hold count * 8 * wordsPerBit words. */
static inline void shiftws_fillColumns(const ShiftWSFillPlan *p, uint32_t *dest, uint32_t col, uint32_t count) {
  const uint32_t wordsPerBit = p->layout.wordsPerBit;
  const uint32_t spw = p->layout.shiftsPerWord;
  const uint32_t width = p->layout.shiftWidth;
  const uint32_t numGroups = p->numGroups;
  uint32_t c, k, g;

  memset(dest, 0, count * SHIFTWS_BITS_PER_BYTE * wordsPerBit * sizeof(uint32_t));

  for (c = 0; c < count; c++) {
    uint32_t *bitBase = dest + c * SHIFTWS_BITS_PER_BYTE * wordsPerBit;
    const uint32_t column = col + c;
    for (k = 0; k < (uint32_t)SHIFTWS_SR_LEN; k++) {
      const uint32_t j = SHIFTWS_SR_LEN - 1 - k;
      const uint32_t wordOff = k / spw;
      const uint32_t bitOff = (k % spw) * width;
      for (g = 0; g < numGroups; g++) {
        const uint8_t *const *pix = &p->pixels[g * SHIFTWS_CHAINS_PER_GROUP * SHIFTWS_SR_LEN];
        const uint8_t *const *lut = &p->lut[g * SHIFTWS_CHAINS_PER_GROUP * SHIFTWS_SR_LEN];
        const uint32_t *ex = p->expand[g];
        uint64_t acc = 0;
        int i, s;
#pragma GCC unroll 8
        for (i = 0; i < SHIFTWS_CHAINS_PER_GROUP; i++) {
          const uint8_t x = lut[i * SHIFTWS_SR_LEN + j][pix[i * SHIFTWS_SR_LEN + j][column]];
          acc |= p->spread[x] << i;
        }
#pragma GCC unroll 8
        for (s = 0; s < SHIFTWS_BITS_PER_BYTE; s++) {
          const uint32_t v = ex[(acc >> (8 * s)) & 0xFF];
          bitBase[s * wordsPerBit + wordOff] |= v << bitOff;
        }
      }
    }
  }
}

#endif /* SHIFTWS2811_FILL_H */
