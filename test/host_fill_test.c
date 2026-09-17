/* Host-side test for ShiftWS2811_fill.h (not compiled by PlatformIO/Arduino).

     gcc -std=c99 -O2 -Wall -Wextra -I.. host_fill_test.c -o host_fill_test && ./host_fill_test

   Compares the fast transposition against a straightforward bit-by-bit
   reference for all shift widths, all chain counts, several chunk sizes and
   random data, and spot checks the physical layout used on the QuinCube
   board.  With -DBENCH it also times the QuinCube configuration. */

#include <stdio.h>
#include <stdlib.h>
#ifdef BENCH
#include <time.h>
#endif

#include "ShiftWS2811_fill.h"

/* Reference: for every chain/output/column/bit set the single bit it maps to. */
static void referenceFill(const ShiftWSFillPlan *p, const uint8_t *lanes, int numChains, uint32_t *dest,
                          uint32_t col, uint32_t count) {
  const uint32_t wpb = p->layout.wordsPerBit, spw = p->layout.shiftsPerWord, W = p->layout.shiftWidth;
  int i, j, s;
  uint32_t c;
  memset(dest, 0, count * 8 * wpb * 4);
  for (i = 0; i < numChains; i++) {
    for (j = 0; j < SHIFTWS_SR_LEN; j++) {
      const int k = SHIFTWS_SR_LEN - 1 - j;
      const uint8_t *stream = p->pixels[i / SHIFTWS_CHAINS_PER_GROUP][j] + (i % SHIFTWS_CHAINS_PER_GROUP) * p->chainStride;
      const uint8_t *lut = p->lut[i & 1][j];
      for (c = 0; c < count; c++) {
        uint8_t pix = lut[stream[col + c]];
        for (s = 0; s < 8; s++) {
          if (pix & (0x80 >> s)) {
            dest[(c * 8 + s) * wpb + k / spw] |= (uint32_t)1 << ((k % spw) * W + lanes[i]);
          }
        }
      }
    }
  }
}

static uint32_t rnd(void) {
  static uint32_t x = 0x12345678;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return x;
}

static ShiftWSFillPlan plan;

/* Point the plan at a pixel buffer laid out like the driver's: chain i,
   output j at (i * 16 + j) * numbytes. */
static void setStreams(const uint8_t *pixels, uint32_t numbytes, const uint8_t *luts /* [2][16][256] */) {
  int g, j;
  plan.chainStride = SHIFTWS_SR_LEN * numbytes;
  for (g = 0; g < SHIFTWS_MAX_GROUPS; g++)
    for (j = 0; j < SHIFTWS_SR_LEN; j++)
      plan.pixels[g][j] = pixels + ((uint32_t)g * SHIFTWS_CHAINS_PER_GROUP * SHIFTWS_SR_LEN + j) * numbytes;
  for (j = 0; j < SHIFTWS_SR_LEN; j++) {
    plan.lut[0][j] = luts + (0 * SHIFTWS_SR_LEN + j) * 256;
    plan.lut[1][j] = luts + (1 * SHIFTWS_SR_LEN + j) * 256;
  }
}

int main(void) {
  const int numbytes = 37; /* odd length to exercise partial chunks */
  const uint8_t widths[4] = {4, 8, 16, 32};
  const uint32_t chunks[3] = {12, 5, 1};
  int failures = 0, tests = 0;
  int wi, ci, numChains, i, j;
  uint32_t col;

  shiftws_buildSpread(&plan);

  uint8_t *pixels = malloc(SHIFTWS_MAX_CHAINS * SHIFTWS_SR_LEN * numbytes);
  uint8_t *luts = malloc(2 * SHIFTWS_SR_LEN * 256);
  const size_t maxWords = (size_t)numbytes * 8 * 16 + 8;
  uint32_t *a = malloc(maxWords * 4), *b = malloc(maxWords * 4);

  for (wi = 0; wi < 4; wi++) {
    const uint8_t W = widths[wi];
    for (numChains = 1; numChains <= SHIFTWS_MAX_CHAINS; numChains++) {
      uint8_t lanes[SHIFTWS_MAX_CHAINS] = {0};
      uint8_t pool[32];
      int poolSize = W;
      size_t n;
      uint32_t wordsPerColumn;

      if (numChains > W) continue; /* every chain needs its own lane */
      plan.layout = shiftws_makeLayout(W);
      plan.numGroups = (uint8_t)((numChains + SHIFTWS_CHAINS_PER_GROUP - 1) / SHIFTWS_CHAINS_PER_GROUP);

      /* random distinct lanes within the shift width */
      for (i = 0; i < W; i++) pool[i] = (uint8_t)i;
      for (i = 0; i < numChains; i++) {
        int idx = rnd() % poolSize;
        lanes[i] = pool[idx];
        pool[idx] = pool[--poolSize];
      }
      shiftws_buildExpand(&plan, lanes, (uint8_t)numChains);

      /* random pixel streams (also for unused chains: they must be ignored)
         and random LUTs (one per output and chain parity) */
      for (n = 0; n < (size_t)SHIFTWS_MAX_CHAINS * SHIFTWS_SR_LEN * numbytes; n++) pixels[n] = (uint8_t)rnd();
      for (n = 0; n < (size_t)2 * SHIFTWS_SR_LEN * 256; n++) luts[n] = (uint8_t)rnd();
      setStreams(pixels, (uint32_t)numbytes, luts);

      wordsPerColumn = 8 * plan.layout.wordsPerBit;
      for (ci = 0; ci < 3; ci++) {
        const uint32_t chunk = chunks[ci];
        memset(a, 0xAA, maxWords * 4);
        memset(b, 0xAA, maxWords * 4);
        for (col = 0; col < (uint32_t)numbytes; col += chunk) {
          uint32_t count = (uint32_t)numbytes - col < chunk ? (uint32_t)numbytes - col : chunk;
          shiftws_fillColumns(&plan, &a[col * wordsPerColumn], col, count);
          referenceFill(&plan, lanes, numChains, &b[col * wordsPerColumn], col, count);
        }
        tests++;
        if (memcmp(a, b, maxWords * 4) != 0) {
          size_t w;
          failures++;
          printf("MISMATCH width=%d chains=%d chunk=%u\n", W, numChains, chunk);
          for (w = 0; w < maxWords; w++) {
            if (a[w] != b[w]) {
              printf("  word %zu: fast %08x ref %08x\n", w, a[w], b[w]);
              break;
            }
          }
        }
      }
    }
  }

  /* Spot check of the physical layout for the QuinCube board: 8 chains on
     FlexIO2 pins 10,17,16,11,29,28,18,19 with PINSEL=10 -> lanes 0,7,6,1,19,18,8,9. */
  {
    const uint8_t lanes[8] = {0, 7, 6, 1, 19, 18, 8, 9};
    static uint8_t ident[2 * SHIFTWS_SR_LEN * 256];
    static uint8_t stream[SHIFTWS_MAX_CHAINS * SHIFTWS_SR_LEN];
    uint32_t out[8 * 16];
    int ok, w;

    plan.layout = shiftws_makeLayout(32);
    plan.numGroups = 1;
    shiftws_buildExpand(&plan, lanes, 8);
    for (i = 0; i < 2 * SHIFTWS_SR_LEN; i++)
      for (j = 0; j < 256; j++) ident[i * 256 + j] = (uint8_t)j;
    memset(stream, 0, sizeof(stream));
    stream[2 * SHIFTWS_SR_LEN + 15] = 0x80; /* chain 2 (pin 8 -> lane 6), output 15, MSB */
    stream[4 * SHIFTWS_SR_LEN + 0] = 0x01;  /* chain 4 (pin 34 -> lane 19), output 0, LSB */
    setStreams(stream, 1, ident);
    shiftws_fillColumns(&plan, out, 0, 1);
    /* output 15 is emitted at shift 0 of WS2811 bit 0 -> word 0, lane 6
       output 0 is emitted at shift 15 of WS2811 bit 7 -> word 7*16+15, lane 19 */
    ok = out[0] == (1u << 6) && out[7 * 16 + 15] == (1u << 19);
    for (w = 0; w < 128; w++)
      if (w != 0 && w != 7 * 16 + 15 && out[w] != 0) ok = 0;
    tests++;
    if (!ok) {
      failures++;
      printf("LAYOUT spot check failed: out[0]=%08x out[127]=%08x\n", out[0], out[127]);
    }
  }

#ifdef BENCH
  /* QuinCube configuration: 8 chains, 125 RGB LEDs, 12 columns per call */
  {
    const uint8_t lanes[8] = {0, 7, 6, 1, 19, 18, 8, 9};
    const uint32_t nb = 375;
    uint8_t *px = malloc(8 * SHIFTWS_SR_LEN * nb);
    uint8_t *lt = malloc(2 * SHIFTWS_SR_LEN * 256);
    uint32_t *out = malloc(12 * 128 * 4);
    size_t n;
    int rep;
    clock_t t0;
    double secs;
    for (n = 0; n < (size_t)8 * SHIFTWS_SR_LEN * nb; n++) px[n] = (uint8_t)rnd();
    for (n = 0; n < (size_t)2 * SHIFTWS_SR_LEN * 256; n++) lt[n] = (uint8_t)rnd();
    plan.layout = shiftws_makeLayout(32);
    plan.numGroups = 1;
    shiftws_buildExpand(&plan, lanes, 8);
    setStreams(px, nb, lt);
    t0 = clock();
    for (rep = 0; rep < 2000; rep++)
      for (col = 0; col < nb; col += 12) shiftws_fillColumns(&plan, out, col, nb - col < 12 ? nb - col : 12);
    secs = (double)(clock() - t0) / CLOCKS_PER_SEC;
    printf("bench: %.1f ns per column (%u columns per frame)\n", secs / 2000 / nb * 1e9, nb);
    free(px);
    free(lt);
    free(out);
  }
#endif

  free(pixels);
  free(luts);
  free(a);
  free(b);
  printf("%d tests, %d failures\n", tests, failures);
  return failures ? 1 : 0;
}
