#include <Arduino.h>

#define MAX_DITHER_BITS 5

uint8_t gammaLUT[256 << MAX_DITHER_BITS];

void gammaLUTCalc(double bri, bool gammaCorr = true, uint8_t finalAdjust = 0) {
  if (gammaCorr) {
    if (bri > 8)
      bri = pow(((bri + 16) / 116), 3);
    else
      bri = bri / 903.3;

    for (int dither = 0; dither < 1 << MAX_DITHER_BITS; dither++) {
      int ditherValue = 0;
      int dread = 1 << MAX_DITHER_BITS;
      int dout = 1;
      for (int d = 0; d < MAX_DITHER_BITS; d++) {
        dread >>= 1;
        if (dither & dread)
          ditherValue |= dout;
        dout <<= 1;
      }
      ditherValue = (ditherValue << (8 - MAX_DITHER_BITS));

      for (int n = 0; n < 256; n++) {
        double brightness = n / 2.55;
        double value;
        int pwmValue;
        if (brightness > 8)
          value = pow(((brightness + 16) / 116), 3);
        else
          value = brightness / 903.3;
        value *= 255 * bri;

        pwmValue = value * 256;
        pwmValue += ditherValue;
        pwmValue = pwmValue >> 8;
        uint8_t fa = n > 0 ? finalAdjust : 0;
        gammaLUT[n + (dither << 8)] = min(pwmValue + fa, 255);
      }
    }
  } else {
    for (int dither = 0; dither < 1 << MAX_DITHER_BITS; dither++) {
      for (int n = 0; n < 256; n++) {
        uint8_t fa = n > 0 ? finalAdjust : 0;
        gammaLUT[n + (dither << 8)] = min(n * bri / 100. + fa, 255);
      }
    }
  }
}