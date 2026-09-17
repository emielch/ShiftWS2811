ShiftWS2811
===========

Drives 128 WS2811/WS2812 LED chains in parallel from a Teensy 4.1 through
8 chains of 2x74AHCT595 shift registers (the ShiftWS2811_PCB board), using
FlexIO2 and the eDMA so that the CPU only converts colour data.  The library
started as a fork of Paul Stoffregen's OctoWS2811 and keeps its API
(`setPixel`, `show`, `busy`, ...), plus gamma correction and temporal
dithering.

How it works
------------

Every WS2811 bit has three phases.  The board makes the LED data lines follow
the COMMON_WF signal (pin 12) through 1k resistors while the STORE signal
(pin 11) is low, and lets the 74HC595 outputs drive the lines while STORE is
high:

```
COMMON  ____/~~~~~~~~~~~~~~~~~~~~~~~~\_____________________________/~~~~
STORE   _____________/~~~~~~~~~~~~~~~~~~~~~~~~~~~~\_____________________
LED     ____/~~~~~~~~~<      595 data (0 or 1)   >_____________________
            <- T0H -> <-         DATA_NS        -> <-      low     ->
```

The STORE rising edge also latches the 16 bits that were shifted into the
595 chains during the previous bit period, so the shift clock (pin 10) runs
continuously with 16 shifts per bit.

FlexIO2 generates everything with cycle accuracy:

* Shifters 0..7 are chained into one 32-bit wide parallel transmitter whose
  pin range covers all data pins (D10..D29 for the QuinCube pin list).  Each
  shift clock outputs one 32-bit word; the 8 shifters hold 8 shifts and the
  eDMA refills all 8 buffers (32 bytes) in one minor loop on the shifter 0
  request, twice per WS2811 bit.  That is 2 DMA requests per bit instead of
  the 16 individually timed GPIO writes of the old QTimer/XBAR version, which
  was what limited the old version to ~554 kHz.
* Timer 0 is the shift clock.  Timers 1 and 2 are one-shot delays started
  with it; their falling edges start the COMMON (timer 6) and STORE (timer 7)
  PWM timers, so the three waveforms have a fixed phase relation.
* Timer 5 counts STORE edges and disables timers 6 and 7 after the last bit,
  timer 4 then times the reset gap and raises an interrupt that starts the
  next frame (dithering keeps re-sending the current frame, as before).
* After the last data buffer the DMA streams zero words until the frame end
  interrupt stops the shift clock, so the shifters never underrun at the end
  of a frame.  A real underrun (DMA too late) sets FlexIO's SHIFTERR flag and
  is counted: `underruns()`.

The colour to bit-stream conversion (`ShiftWS2811_fill.h`) transposes 8
chains at a time through two small lookup tables; it is hardware independent
and has a host test in `test/host_fill_test.c`.

CPU load
--------

The conversion runs in the DMA interrupt, once per LED frame.  Frames are
sent back to back (dithering re-sends the current frame), so this is a
continuous background load of roughly

    frames per second  x  conversion time per frame

i.e. 261 frames/s at 800 kHz with 125 RGB LEDs per output.  The fill core
needs about 1500 Cortex-M7 instructions per LED byte column (128 pixel
bytes); expect around 15 percent of an 816 MHz core for the QuinCube.
`cpuLoad()` returns the measured percentage (DWT cycle counts of the
library's interrupts), call it every second or so next to your fps counter.

If that is too much for an animation, raise `SHIFTWS2811_RESET_US`: a longer
gap between frames lowers the LED frame rate and the load linearly (the
dither depth chosen by `setDitherBits(255)` follows the frame time).
`SHIFTWS2811_BITDATA_DTCM 1` moves the conversion buffers from OCRAM to
DTCM, which removes the cache line fills and flushes of the destination
writes; it is untested on the board (the eDMA then reads through the core's
AHBS port), so check `underruns()` when trying it.

Timing configuration
--------------------

All timing is derived at compile time from these defines (override them with
`-D...` build flags):

| define                        | default | meaning                                              |
|-------------------------------|---------|------------------------------------------------------|
| `SHIFTWS2811_BIT_NS`          | 1250    | WS2811 bit period (800 kHz)                          |
| `SHIFTWS2811_T0H_NS`          | 300     | high time of a 0 bit                                 |
| `SHIFTWS2811_DATA_NS`         | 400     | 595 output enable time; a 1 bit is high T0H + DATA   |
| `SHIFTWS2811_RESET_US`        | 80      | low time between frames (also caps the frame rate)   |
| `SHIFTWS2811_BYTES_PER_DMA`   | 12      | LED bytes converted per DMA interrupt (x2 buffers, 6 KB each) |
| `SHIFTWS2811_SHIFT_DIV`       | 3       | shift clock period = 2*(DIV+1) FlexIO clocks         |
| `SHIFTWS2811_TRIGGER_LATENCY` | 1       | FlexIO clocks between a timer edge and the timer it starts |
| `SHIFTWS2811_DMA_BURST`       | 1       | 32-byte burst source reads (0: 32-bit reads)         |
| `SHIFTWS2811_DMA_PRIORITY`    | 1       | raise the DMA channel to the top fixed priority      |
| `SHIFTWS2811_BITDATA_DTCM`    | 0       | 1: conversion buffers in DTCM instead of OCRAM (untested) |
| `SHIFTWS2811_PIN_SHIFT_CLK`   | 10      | 595 SRCLK (FlexIO2 pin)                              |
| `SHIFTWS2811_PIN_STORE`       | 11      | 595 RCLK / output enable (FlexIO2 pin)               |
| `SHIFTWS2811_PIN_COMMON`      | 12      | COMMON_WF (FlexIO2 pin)                              |

The FlexIO clock is `16 * 2 * (SHIFT_DIV + 1) / BIT_NS` (102.4 MHz for the
defaults) and comes from the otherwise unused video PLL, so any bit period
is hit exactly.  FlexIO3 shares this clock root.  Resolution of the waveform
edges is one FlexIO clock (9.8 ns at the defaults).

Buffers: the three pixel buffers hold `numPerStrip * 3 * 16` bytes per data
pin and must be allocated for a multiple of 8 data pins (the converter reads
the pins in groups of 8).

Bringing it up on hardware
--------------------------

The library was written against the i.MX RT1060 reference manual; the first
run on the QuinCube board (September 2026) drove the LEDs correctly.
`docs/DESIGN.md` collects all the research behind it (verified board
netlist, the FlexIO/eDMA/clock facts with manual references, the timeline,
rejected alternatives, CPU load findings and a 256-channel plan).  Things
still worth a look with a scope:

1. `error()` must return 0 after `begin()`; `frames()` must count up and
   `underruns()` and `stalls()` must stay 0.  Underruns mean the DMA was late
   refilling the shifters; lengthen `SHIFTWS2811_BIT_NS` a little or check for
   other heavy DMA users.  Stalls mean the frame end interrupt never came, i.e.
   the FlexIO timer chain did not run as designed.
2. Shift clock (pin 10): 16 rising edges per bit period, 12.8 MHz at the
   defaults.  Data pins must change on the falling edges.
3. STORE (pin 11) rising edge must sit midway between the 16th and 17th shift
   clock rising edges of every bit (about 39 ns from each).  If it is off by
   a whole shift clock cycle or more, adjust `SHIFTWS2811_TRIGGER_LATENCY`
   (each unit moves STORE and COMMON by one FlexIO clock).  The 74AHCT595
   needs only 5 ns between SRCLK and RCLK.
4. COMMON (pin 12) rises `T0H` before STORE rises and falls `DATA/2` after
   it; STORE is high for `DATA`.
5. After the last bit of a frame all three signals stay low for the reset
   gap; the first bit of the next frame starts with COMMON going high one
   bit period after the shift clock restarts.

Diagnostics
-----------

```
leds.frames();       // frames sent
leds.underruns();    // frames with a late DMA refill (visible as a glitch)
leds.stalls();       // frames that never finished; show() restarted the engine
leds.error();        // 0, or an ERR_ code from ShiftWS2811.h
leds.frameTimeUs();  // frame + reset gap duration
leds.cpuLoad();      // percent of CPU spent in the library since the previous call
```
