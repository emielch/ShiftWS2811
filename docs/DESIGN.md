# ShiftWS2811 design notes

Everything that had to be researched for the FlexIO2 rewrite of this library
(September 2026), so that a re-implementation or a change of hardware does not
have to start from scratch.  Section numbers like "RM 50.3.2.4" refer to the
i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021 (IMXRT1060RM), a copy
of which PJRC hosts at <https://www.pjrc.com/teensy/IMXRT1060RM_rev3.pdf>.
The FlexIO chapter is chapter 50 (pages 2943 to 3040 of that PDF), the CCM is
chapter 14, the eDMA chapter 6, the IOMUXC chapter 11.

Status: the code runs on the QuinCube (2026-09-17: LEDs correct at 800 kHz on
the first try).  Section 6 lists what a scope should still confirm, section 10
the CPU load findings from that first run and the resulting fill core rewrite.

---------------------------------------------------------------------------

## 1. The board (ShiftWS2811_PCB), verified netlist

Derived from the KiCad 8 schematic with `docs/tools/kicad_netlist.py` and
cross-checked against KiCad's own pad-to-net assignment stored in the
`.kicad_pcb` (957 pads, 0 mismatches).

### Teensy 4.1 connections

| Teensy pin | net            | goes to                                           |
|-----------:|----------------|---------------------------------------------------|
| 6          | SR_1_DATA      | block 1 first 595 SER (pin 14), directly           |
| 7          | SR_2_DATA      | block 2                                            |
| 8          | SR_3_DATA      | block 3                                            |
| 9          | SR_4_DATA      | block 4                                            |
| 34         | SR_5_DATA      | block 5                                            |
| 35         | SR_6_DATA      | block 6                                            |
| 36         | SR_7_DATA      | block 7                                            |
| 37         | SR_8_DATA      | block 8                                            |
| 10         | SR_SHIFT_CLK   | U4 (74AHCT245) A0..A7 -> SR_SHIFT_CLK_1..8 -> SRCLK of all 16 595s |
| 11         | SR_STORE_CLK   | U5 (74AHCT245) A0..A7 -> SR_STORE_CLK_1..8 -> RCLK of all 16 595s; RV1 wiper; RV2 pin 3 |
| 12         | COMMON_WF      | U3 (74AHCT245) A0..A7 -> COMMON_WF_1..8           |
| VIN        | +5V            | board supply (J5 through Schottky D1)              |
| VBAT       | BT1            | RTC battery                                        |

The data lines are 3.3 V logic straight into the 74AHCT595 (TTL thresholds,
fine).  No other Teensy pins are connected.

### The output-enable trick (this is what makes the waveform)

* `SR_STORE_CLK` -> RV2 (1k pot) -> 47 pF (C22) -> all 8 inputs of U2
  (74AHCT240, inverting) -> `SR_OE_1..8` -> `~OE` of both 595s of each block.
  So the 595 outputs are **enabled while STORE is high** (RC delayed).
* `SR_STORE_CLK` -> RV1 (1k pot) -> 47 pF (C21) -> `~OE` of U3.  So the
  COMMON_WF drivers are **enabled while STORE is low** (RC delayed).
* Each of the 128 outputs has a 1k resistor to its block's `COMMON_WF_n`.

Result: STORE high = data phase (595 drives the LED line), STORE low = the
line follows COMMON_WF through 1k.  COMMON_WF only has to be high during the
leading part of each bit (T0H) and low during the trailing part.  The pots set
the break-before-make between the two drivers (0 to ~47 ns).

### Inside an output block (8 identical hierarchical sheets)

* U6 (first 595): SER = DATA, SRCLK = SHIFT_CLK, RCLK = STORE_CLK,
  ~OE = OE, ~SRCLR = +5V, QA..QH = OUT_1..OUT_8, QH' -> U7 SER.
* U7 (second 595): QA..QH = OUT_9..OUT_16, **QH' unconnected** (so a chain can
  be extended by wiring QH' to the next block's DATA).
* Because the first bit shifted in ends at U7.QH, shift register output `j`
  (0..15, OUT_(j+1)) is transmitted as shift number `15 - j` of the 16 shifts.
* Connectors: J1..J4 are 2x32 headers; odd pins GND, even pin `2k` = OUT_k of
  the first block on that connector, pin `32+2k` = OUT_k of the second block.
  Blocks 1..8 = J1 low, J1 high, J2 low, J2 high, J3 low, J3 high, J4 low,
  J4 high, which matches `pinList = {6, 7, 8, 9, 34, 35, 36, 37}` in
  `CubeDriver_Q25.cpp`.

### 74AHCT595 timing (TI datasheet, 5 V)

| parameter                         | value        |
|-----------------------------------|--------------|
| f_max                             | >= 115 MHz   |
| SER setup before SRCLK rising     | 3 ns         |
| SER hold after SRCLK rising       | 2 ns         |
| SRCLK rising before RCLK rising   | 5 ns         |
| minimum pulse width               | 5 ns         |

The 74AHCT245/240 buffers add about 5 ns each, equally on the clock and store
paths.

---------------------------------------------------------------------------

## 2. Why the QTimer/XBAR/GPIO-DMA version topped out at ~554 kHz

The old code wrote one 32-bit word to `GPIO2_DR` per shift clock, each write a
separate eDMA request paced by QTimer3 through XBAR (`DMA_TICS = 23` bus clocks
at 204 MHz, see `freq_calc.xlsx`): 8.87 M requests/s, 16 per WS2811 bit, hence
554 kHz.  At 800 kHz it would need 12.8 M requests/s.  Each hardware-triggered
single-word eDMA transfer costs roughly 20+ bus clocks (arbitration, TCD
fetch, OCRAM read, IPS write, TCD write-back), so ~9 M/s is the ceiling, and a
single late transfer shifts the whole frame by one stage because the request
is simply lost.  Any GPIO-based scheme keeps this problem: the data must be
paced per shift clock by the eDMA.  The fix is to let a peripheral do the
pacing (FlexIO shifters), leaving the eDMA to move blocks.

---------------------------------------------------------------------------

## 3. i.MX RT1060 facts that the design depends on

### 3.1 FlexIO instances (RM 50.6.1.3 PARAM, chapter 4 DMA mux table)

| instance | PARAM       | shifters | timers | pins | DMA requests (DMAMUX source)                  | IRQ | base        |
|----------|-------------|---------:|-------:|-----:|-----------------------------------------------|-----|-------------|
| FlexIO1  | 0x0210_0808 | 8        | 8      | 16   | shifters 0/1 -> src 0, shifters 2/3 -> src 64  | 90  | 0x401A_C000 |
| FlexIO2  | 0x0220_0808 | 8        | 8      | 32   | shifters 0/1 -> src 1, shifters 2/3 -> src 65  | 91  | 0x401B_0000 |
| FlexIO3  | 0x0220_0808 | 8        | 8      | 32   | **none**                                       | 156 | 0x4202_0000 |

Older RT1050 documents (and the app notes AN12822/AN12686) say 4 shifters and
4 timers for FlexIO1/2; that is the RT1050.  Shifters 4..7 exist on the RT1060
but have no DMA request line; that does not matter because one DMA minor loop
can write all eight SHIFTBUF registers on one request.

### 3.2 Teensy 4.1 pins on FlexIO2 (IOMUXC ALT4, RM chapter 11)

FlexIO2 pin `Dn` is the GPIO2 pad with the same bit number: `GPIO_B0_xx ->
D[xx]`, `GPIO_B1_xx -> D[16+xx]`.  In the Teensy core, a pin is on FlexIO2 iff
`portOutputRegister(pin) == &GPIO7_DR` and the FlexIO index is
`digitalPinToBit(pin)`.

| Teensy | pad      | FlexIO2 | Teensy | pad      | FlexIO2 |
|-------:|----------|---------|-------:|----------|---------|
| 10     | B0_00    | D0      | 8      | B1_00    | D16     |
| 12     | B0_01    | D1      | 7      | B1_01    | D17     |
| 11     | B0_02    | D2      | 36     | B1_02    | D18     |
| 13     | B0_03    | D3      | 37     | B1_03    | D19     |
| 6      | B0_10    | D10     | 35     | B1_12    | D28     |
| 9      | B0_11    | D11     | 34     | B1_13    | D29     |
| 32     | B0_12    | D12     |        |          |         |

That is all 13 of them.  D20..D27 (B1_04..B1_11) go to the Ethernet PHY,
D4..D9, D13..D15, D30, D31 are not on any pin.  The same pads are also
FlexIO3 pins (ALT9) with the same numbers, but FlexIO3 has no DMA.  FlexIO1
(ALT4 on GPIO_EMC_00..15) reaches Teensy pins 2 (D4), 3 (D5), 4 (D6), 33
(D7) and 5 (D8) only.

### 3.3 Parallel mode (RM 50.3.3.1, SHIFTCFG PWIDTH 50.6.1.15)

* Parallel **transmit** to pins is only possible from **shifter 0 and
  shifter 4**; parallel receive only into shifters 3 and 7.  All other
  shifters can only feed the adjacent lower shifter (chaining).
* Shift widths are 1, 4, 8, 16 or 32 bits: `PWIDTH = 0` -> 1 bit,
  `1..3` -> 4 bits, `4..7` -> 8, `8..15` -> 16, `16..31` -> 32.
* The pins driven are `FXIO_D[PINSEL + PWIDTH] : FXIO_D[PINSEL]`, and shifter
  data bit `i` goes to pin `PINSEL + i` (RM example: "SHIFTER[5:0] drives
  FXIO_D[12:7]" for PINSEL=7, PWIDTH=6).  When PWIDTH is smaller than the
  shift width the upper pins are simply not driven.  Pads in the range that
  are not muxed to FlexIO are unaffected, so a wide range may span unused
  pins.
* `PINSEL` and `PWIDTH` are 5-bit fields on FlexIO2/3 (only 4 bits on
  FlexIO1).

Consequence for this board: the eight data pins D10, D11, D16..D19, D28, D29
are not contiguous, so the only single group that covers them is a 32-bit
wide shift with `PINSEL = 10`, `PWIDTH = 19` (D10..D29).  Every shift clock
then consumes one 32-bit word of which 8 bits are used.

### 3.4 Chaining (SHIFTCFG INSRC) and stream order

With `INSRC = 1` shifter N takes its input from shifter N+1's output ("the
least significant 4/8/16/32 bits from the adjacent shifter", RM 50.3.3.1).
`INSRC = 1` is not allowed on the last shifter.  Transmit shifts LSB first
(RM 50.4.3), so for a chain 0 <- 1 <- ... <- 7 the stream order is
SHIFTBUF0 (low bits first), then SHIFTBUF1, ..., SHIFTBUF7, i.e. memory order
when the DMA writes SHIFTBUF0..7 with an incrementing address.  The RM's 8080
example (50.4.9, Table 50-17) uses exactly this: 8 shifters, 16-bit width,
`SHIFTCFG0..7 = 0x000F_0100`.

### 3.5 Transmit mode load and flag semantics (RM 50.3.1.1, 50.3.2.4, 50.3.2.5, SSF 50.6.1.6)

* Status flag SSF is set when SHIFTBUF has been transferred into the shifter
  (buffer empty) **and when the shifter is first configured for transmit
  mode**.  Any write to SHIFTBUF clears it.  This is what makes the DMA
  prefill happen: re-writing SHIFTCTL with SMOD=transmit at the start of a
  frame sets all flags and hence the request.
* On timer enable, transmit shifters with `SSTART = 0` load SHIFTBUF and
  output the first bits immediately ("loads data on enable").  On every timer
  compare (end of the configured number of shift clock edges) they load
  again.  With a never-disabled timer this gives one load per word, no double
  loads.  (Whether the compare that coincides with a `TIMDIS = on compare`
  disable also loads is not stated clearly; the design avoids the question by
  never disabling the shift clock timer inside a frame.)
* SHIFTERR (SEF) is set when a load happens while SHIFTBUF was not rewritten
  since the last load (underrun).  The data still gets sent (stale).  This is
  the DMA-late detector used by `underruns()`.
* Loading and shifting cannot happen on the same cycle (chip-specific note
  RM 50.1); irrelevant here because the compare edge replaces the shift.

### 3.6 Timers (RM 50.3.2, TIMCTL/TIMCFG/TIMCMP 50.6.1.20 to 50.6.1.22)

* **Dual 8-bit baud mode** (`TIMOD = 1`): lower byte = baud divider, output
  toggles every `(CMP[7:0]+1)` FlexIO clocks (period `2*(CMP[7:0]+1)`); upper
  byte counts output edges, compare after `CMP[15:8]+1` edges, so
  `CMP[15:8] = bits*2 - 1`.  The compare toggles the output, loads transmit
  shifters, sets TSF, and reloads the counter.  Max 128 bits per compare.
* **Dual 8-bit PWM high mode** (`TIMOD = 2`): output high for `CMP[7:0]+1`
  clocks, low for `CMP[15:8]+1`; the lower counter runs while the output is
  high, the upper while it is low, so with `TIMOUT = 1` (low at enable) the
  low phase comes first.  Compare (TSF) at the end of every low phase.
* **16-bit counter mode** (`TIMOD = 3`): toggles and compares every
  `CMP+1` decrements.  Decrement source `TIMDEC`: 0 FlexIO clock, 1 trigger
  edges, 2 **pin** edges (both edges), 3 trigger edges with shift clock =
  trigger.
* **Enable conditions** `TIMENA`: 0 always, 1 on timer N-1 enable (same
  cycle; reserved for timers 0 and 4), 2 trigger high, 3 trigger high and pin
  high, 4 pin rising, 5 pin rising and trigger high, 6 trigger rising edge,
  7 trigger both edges.  **Disable** `TIMDIS`: 0 never, 1 on timer N-1
  disable (same cycle; reserved for timers 0 and 4), 2 on compare, 3 on
  compare and trigger low, 4 pin edge, 5 pin edge and trigger high, 6
  trigger falling edge.  Level/edge refer to the signal after TRGPOL/PINPOL.
* On enable the counter loads CMP and the output takes the `TIMOUT` initial
  state (0/2 = high, 1/3 = low).  **On disable the output is cleared** (RM
  50.3.2.6), and a timer that is not enabled outputs 0.  So a PWM/one-shot
  timer with `PINPOL = 0` idles low.
* **Trigger output**: "Each FlexIO Timer generates an output trigger equal to
  the timer output" (RM 50.3.9.1), selectable by other timers with
  `TRGSEL = 4*N + 3`, `TRGSRC = 1`.  Shifter N status flag is `4*N + 1`, pin
  N input is `2*N` (RM 50.6.1.20).  `TRGPOL = 1` inverts, so "trigger rising
  edge" becomes the falling edge of the source.
* **Latencies** (RM 50.3.3.2): a pin input driven by another FlexIO timer or
  shifter is available internally after 1 FlexIO clock (0.5 to 1.5 clocks for
  an external signal).  The delay from a timer-output trigger edge to the
  enable of the timer that uses it is not stated; the library assumes 1 clock
  (`SHIFTWS2811_TRIGGER_LATENCY`) and this is the one number to confirm with a
  scope.
* TIMCFG must be written before TIMOD is set; a timer arms itself again after
  it was disabled and waits for its enable condition, so the whole chain
  re-runs on the next frame without reprogramming.
* Writing `CTRL[SWRST]` then 0 resets everything except CTRL.

### 3.7 Clocking (RM 14.7 CSCMR2/CS1CDR, 14.8.11 to 14.8.13, 14.8.19)

* FlexIO2 root: `CSCMR2[FLEXIO2_CLK_SEL]` bits 20:19 (0 = PLL4 audio,
  1 = PLL3 PFD2 508 MHz, 2 = PLL5 video, 3 = pll3_sw_clk 480 MHz),
  `CS1CDR[FLEXIO2_CLK_PRED]` bits 11:9 and `[FLEXIO2_CLK_PODF]` bits 27:25,
  both "divide by n+1" (1..8).  **FlexIO3 uses the same root and fields.**
  Clock gate `CCGR3[CG0]` (`CCM_CCGR3_FLEXIO2`).  Change the root with the gate
  off.
* The FlexIO functional clock must not exceed 120 MHz (datasheet).  FlexIO
  register writes are synchronised to the functional clock unless
  `CTRL[FASTACC]` is set, which requires FlexIO clock >= 2x bus clock (not the
  case here).
* PLL5 (video PLL) is unused by Teensyduino.  `f = 24 MHz * (DIV_SELECT +
  NUM/DENOM)` with DIV_SELECT 27..54 (VCO 650..1300 MHz), then
  `POST_DIV_SELECT` (0 = /4, 1 = /2, 2 = /1) and `MISC2[VIDEO_DIV]` (0 = /1,
  1 = /2, 3 = /4).  Register sequence that works (same as the Teensy Audio
  library uses for PLL4 and FlexIO_t4 for PLL5): write PLL_VIDEO with BYPASS,
  ENABLE, POST_DIV_SELECT and DIV_SELECT; write NUM and DENOM; clear
  POWERDOWN; wait for LOCK (bit 31); set VIDEO_DIV; clear BYPASS.  For the
  default 102.4 MHz the library picks VCO 716.8 MHz (`29 + 13/15`), total
  division 7.
* From PLL3 (480 MHz) alone the reachable FlexIO clocks near the useful range
  are 120, 96, 80, 68.6, 60 MHz, i.e. bit periods of 1.067, 1.2, 1.333 us
  with 16 shifts of `2*(DIV+1)` clocks; exactly 1.25 us needs PLL5 (or PLL4).

### 3.8 eDMA (RM chapter 6) and the Teensy DMAChannel class

* TCD `ATTR`: `SSIZE/DSIZE` 0 = 8-bit, 1 = 16, 2 = 32, 3 = 64-bit, **5 =
  32-byte burst** (4 and 6, 7 reserved).  `SMOD/DMOD` (5 bits) freeze the
  upper address bits: `DMOD = 5` keeps the destination inside a 32-byte
  window, which is how SHIFTBUF0..7 (offset 0x200, 32-byte aligned) are
  written round-robin.  With a 32-byte source burst `SOFF` must be 32 and the
  source 32-byte aligned.
* `DMAChannel::begin()` sets `DMA_CR = GRP1PRI | EMLM | EDBG`; with EMLM the
  NBYTES register is `NBYTES_MLOFFNO` (30 bits when SMLOE = DMLOE = 0) and
  CITER/BITER are 15 bits (top bit = ELINK), so 0x7FFF is the largest count.
* Scatter/gather: `CSR[ESG]` with `DLASTSGA` = address of the next TCD (must
  be 32-byte aligned; `DMASetting` objects are).  The eDMA can fetch TCDs
  from DTCM.  The reload happens at major-loop completion together with the
  `INTMAJOR` interrupt, so an ISR that runs after that may rewrite the
  memory TCD for the *following* reload.
* Channel priority registers `DCHPRI0..31` are bytes, byte-swapped inside
  each word: `DCHPRI3` at offset 0x100, `DCHPRI2` at 0x101 ... index for
  channel `c` = `(c & 0x1C) | (3 - (c & 3))` from `&DMA_DCHPRI3`.  Fixed
  priority = channel number by default, group 1 (channels 16..31) above group
  0.  `DMAChannel::begin()` allocates the lowest free channel and sets
  `ECP | DPA` (preemptable, cannot preempt).
* `DMAMEM` variables (OCRAM, RAM2) are not zeroed at start-up and are
  cached: fill, then `arm_dcache_flush_delete(addr, size)` before the DMA
  reads them.  Static/global data lives in DTCM (RAM1), which the eDMA can
  also reach.  RAM1 (512 KB) is split between ITCM code and DTCM data in
  32 KB blocks, so code just over a 32 KB boundary wastes most of a block;
  `FLASHMEM` moves cold functions out of ITCM.
* `attachInterruptVector(IRQ_FLEXIO2, fn)` + `NVIC_SET_PRIORITY` +
  `NVIC_ENABLE_IRQ` for the FlexIO interrupt; `dma.attachInterrupt(fn, prio)`
  for the channel.  Write a `dsb` after clearing a peripheral flag at the end
  of an ISR (avoids a spurious re-entry on the Cortex-M7).
* Measured/estimated throughput: single-word triggered transfers about 9 M/s
  (from the old design); a 32-byte minor loop (one burst read + eight 32-bit
  IPS writes) is estimated at roughly 250 to 300 ns.  The design needs one
  every 625 ns.  This estimate is the main thing the `underruns()` counter is
  there to confirm.

### 3.9 LED timing constraints used

WS2812B: T0H 0.4 +-0.15 us, T1H 0.8 +-0.15 us, T0L 0.85 +-0.15, T1L 0.45
+-0.15, period 1.25 +-0.6 us, reset >= 50 us (280 us for newer parts).
WS2811 high-speed: T0H 0.25 +-0.15, T1H 0.6 +-0.15.  The defaults T0H 300 ns,
T1H 700 ns, T1L 550 ns, T0L 950 ns, period 1.25 us sit inside all of them.
The old code used T0H 300 ns and T1H 900 ns at a 1.8 us period and worked on
the user's LEDs.

---------------------------------------------------------------------------

## 4. The implemented design

### 4.1 Signals per bit period

```
COMMON  ____/~~~~~~~~~~~~~~~~~~~~~~~~\_____________________________/~~~~
STORE   _____________/~~~~~~~~~~~~~~~~~~~~~~~~~~~~\_____________________
LED     ____/~~~~~~~~~<      595 data (0 or 1)   >_____________________
            <- T0H -> <-         DATA_NS        -> <-      low     ->
```

The STORE rising edge latches the 16 bits shifted during the previous bit
period, so the shift clock runs continuously (16 shifts per bit) and STORE
must fall midway between the 16th and 17th shift clock rising edges.  COMMON
stays high until DATA/2 into the data phase; while STORE is high the COMMON
drivers are tri-stated anyway, so this only guarantees the line is never
undriven at the phase-1-to-2 transition and is low before the 595 outputs
release the line.

### 4.2 Resource allocation on FlexIO2

| resource     | role                                                                     | key settings |
|--------------|--------------------------------------------------------------------------|--------------|
| shifter 0    | parallel transmitter, drives D10..D29 (`PINSEL 10`, `PWIDTH 19`)          | TIMSEL 0, TIMPOL 1 (shift on falling edge), PINCFG 3, SMOD 2, INSRC 1 |
| shifter 1..7 | chained behind shifter 0, no pin                                          | same, PINCFG 0; shifter 7 INSRC 0 |
| timer 0      | shift clock on D0 (pin 10), baud mode, 8 shifts per compare               | TIMCMP `(8*2-1)<<8 | 3`, TIMOUT 1 (low at enable), TIMENA 2 on trigger = shifter 7 flag inverted, TIMDIS 0 |
| timer 1      | one-shot, high for `128 - T0H - L` clocks                                 | 16-bit, TIMENA 1 (with timer 0), TIMDIS 2 |
| timer 2      | one-shot, high for `128 - L` clocks                                       | 16-bit, TIMENA 1 (with timer 1), TIMDIS 2 |
| timer 5      | frame end: decrements on STORE pin edges (D2), `CMP = 2*numBits - 1`     | 16-bit, TIMDEC 2, TIMENA 6 on timer 1 rising, TIMDIS 2, TSF interrupt |
| timer 6      | COMMON PWM on D1 (pin 12): high `T0H + DATA/2`, low the rest              | TIMOD 2, TIMENA 6 on timer 1 falling (TRGPOL 1), TIMDIS 1 (with timer 5) |
| timer 7      | STORE PWM on D2 (pin 11): high `DATA`, low the rest                       | TIMOD 2, TIMENA 6 on timer 2 falling, TIMDIS 1 (with timer 6) |
| timer 4      | reset gap, `CMP = RESET_US * f - 1`                                       | 16-bit, TIMENA 6 on timer 5 falling, TIMDIS 2, TSF interrupt |
| timer 3      | unused                                                                    | |
| DMA          | shifter 0 request (DMAMUX source 1) -> 32 bytes into SHIFTBUF0..7 (DMOD 5) | SSIZE 5 / SOFF 32 (or SSIZE 2 / SOFF 4), DSIZE 2, DOFF 4, ESG ping-pong |

Timers 1/2 are chained from timer 0 because timer 0 cannot be a chain target
and only chaining gives a same-cycle start; the STORE/COMMON timers are started
by trigger edges because their PWM phase lengths are fixed, so the only way to
give them an arbitrary phase offset is to delay their enable.  Timers 6/7 sit
directly after timer 5 so they can be switched off with it in the same cycle.

### 4.3 Cycle-exact timeline for the defaults

FlexIO clock 102.4 MHz (9.766 ns), `SHIFT_DIV = 3` -> 8 clocks per shift
(12.8 MHz), 128 clocks per bit (1.25 us).  `t0` = cycle in which timer 0 is
enabled (SHIFTBUF7 written by the prefill DMA).

| time (clocks from t0)      | event |
|----------------------------|-------|
| 0                          | shifters load word 0, timers 1, 2 start (high), timer 5 armed one clock later |
| 4 + 8k (k = 0..)           | shift clock rising edge k+1, 595s sample; data changes on the falling edges 8 + 8k |
| 64, 128, 192, ...          | shifter compare: next 8 words loaded, SSF set -> DMA refills within 64 clocks (625 ns) |
| 96 (+L)                    | timer 1 falls -> COMMON rises (phase 1 of bit 0) |
| 127 (+L)                   | timer 2 falls -> STORE rises: bit 0 data phase; midway between rising edges 124 and 132 (39 ns margin each side) |
| 148                        | COMMON falls (20 clocks into the data phase) |
| 169                        | STORE falls (data phase 41 clocks = 400 ns) |
| 128 (n+1), n = 0..N-1      | STORE rises for bit n |
| 128 N + 41                 | N-th STORE falling edge = 2N-th edge -> timer 5 compare -> timers 5, 6, 7 disabled, all outputs low, timer 4 starts, END interrupt |
| + 8192                     | timer 4 compare (80 us) -> GAP interrupt -> next frame or idle |

Derived constants in the source: `T0H_CYC 31`, `DATA_CYC 41`,
`COMMON_HIGH 51 / LOW 77`, `STORE_HIGH 41 / LOW 87`, `DELAY_COMMON 96`,
`DELAY_STORE 127`, `GAP 8192`, `TIMCMP[5] = 5999` for 375 bytes per output.
Frame: 3000 bits x 1.25 us + 1.25 us lead-in + 80 us = 3.83 ms (261 frames/s
of dithering) versus 5.5 ms before.

### 4.4 Frame sequence

1. `transfer()` (from `show()` when idle, else from the GAP interrupt):
   swap front/back if a new frame is pending, advance the dither phase,
   rebuild the per-output LUT pointers, convert the first two buffers, write
   the channel TCD (buffer 0) and the memory TCD `dmanext` (buffer 1 or the
   zero TCD), re-enter transmit mode on all shifters (sets SSF -> request),
   clear SHIFTERR/TIMSTAT, enable the DMA channel (prefills SHIFTBUF0..7),
   write TIMCTL0.  Timer 0 enables itself when SHIFTBUF7 is written.
2. DMA interrupt at each buffer completion: convert the next 6 LED bytes into
   the buffer that was just consumed and point `dmanext` at it; after the last
   data buffer `dmanext` is the self-linking zero TCD (SOFF 0, CITER 0x7FFF,
   ESG to itself), so the shifters keep loading zeros and never underrun at
   the frame end.
3. END interrupt (timer 5 TSF): stop timer 0 (TIMCTL0 = 0), disable the DMA
   channel, count SHIFTERR into `underruns()`, count the frame.
4. GAP interrupt (timer 4 TSF, 80 us later): `transfer()` again if dithering
   is on or a frame is pending, otherwise `transferring = false`.
5. `show()` waits for a pending frame to be picked up; if that takes longer
   than 4 frame times + 10 ms the engine is restarted and `stalls()` counts
   it (bring-up aid; a healthy system never does this).

### 4.5 Data conversion (`ShiftWS2811_fill.h`)

Stream layout: per LED byte 8 WS2811 bits (MSB first) x `wordsPerBit` words;
shift `k` (0 first) carries output `15 - k`; `shiftsPerWord = 32 /
shiftWidth` shifts share a word LSB first; chain `i` is bit `lane[i] =
FlexIOpin[i] - PINSEL` of each shift (lanes 0, 7, 6, 1, 19, 18, 8, 9 for the
QuinCube pin list).  For 32-bit shifts this is 16 words per bit, one per
shift, and 2 DMA minor loops per bit.

Algorithm (second version, 2026-09-17): loops are ordered output `j` (16),
group, column, and the 8 chains of a group are unrolled.  Per (output,
column): the 8 pixel bytes are read with a fixed stride (`chainStride =
16 * numbytes`), corrected through the 256-entry gamma/dither table of that
output (one table for even, one for odd chains), spread with a 256 x 64-bit
table so that byte `s` of the accumulator holds bit `7-s` of every chain
(chain `i` at bit `i`), and each byte is mapped with a 256 x 32-bit table
onto the lane positions and stored as the word of WS2811 bit `s`.  The two
32-bit halves of the accumulator are kept apart: a spread value shifted by
at most 7 never crosses a word boundary, so everything stays in 32-bit
registers (`orr rd, rd, rm, lsl #i`).  For 32-bit shifts with one group
every destination word is written exactly once (no memset, no
read-modify-write); other layouts OR into a zeroed buffer (generic, slower
path; specialise it before relying on it for a new board).

Cost (GCC 15, -O2, Cortex-M7): 93 instructions and about 40 memory accesses
per output and column, i.e. about 1500 instructions per LED byte column, all
in DTCM (pixels, gamma LUT, plan) except the destination writes to the OCRAM
conversion buffer.  The first version (per-chain pointer tables, 64-bit
shifts, read-modify-write, memset) compiled to 195 instructions per shift
with heavy stack spilling, 3000+ per column, about the cost of the old
`fillbits`, and caused the frame rate drop described in section 10.  On an
x86 host the second version is 3.3x faster than the first
(`gcc -O2 -DBENCH`: 68 vs 220 ns per column).

Chains beyond `numPins` in the last group are read (the pixel buffers must
be sized for a multiple of 8 chains) but contribute nothing because
`shiftws_buildExpand` leaves their lane bits out.  Up to 16 chains (two
groups) are supported.  The host test (`test/host_fill_test.c`, plain C99,
`gcc -std=c99 -O2 -I.. host_fill_test.c`, `-DBENCH` adds a timing loop)
compares it with a bit-by-bit reference for all widths, chain counts and
three chunk sizes (133 cases).

Dither phase for output `(chain i, output j)` is
`(frameCycle + i*16 + j + 1) mod 2^ditherBits`, identical to the old code
(which incremented `ditherCycle` once per frame plus once per output).
Because `16*i mod 32` is 0 for even and 16 for odd chains, this is exactly
one table per output for the even chains and one for the odd chains, which
is how the plan stores it (`lut[parity][output]`).

### 4.6 API changes

Unchanged: constructor, `begin`, `show`, `busy`, `setPixel*`, `getPixel`,
`setBrightness`, `setDitherBits`.  New: `frames()`, `underruns()`,
`stalls()`, `error()`, `bitPeriodNs()`, `frameTimeUs()`.  `numPixels()`
now includes the 16 outputs per chain (it used to return strips x pins; the
firmware does not call it).  `defaultPinList` is now the QuinCube list.

---------------------------------------------------------------------------

## 5. Alternatives considered and why they were not used

* **Stop-and-go shift bursts** (timer 0 disabled on compare and re-enabled on
  each STORE edge, fast burst of 16 shifts, idle gap): would make the STORE
  placement trivially safe, but relies on the unclear "does the disabling
  compare also load" behaviour (would skip every other word if it does).
* **TSTOP stop bit** (17 slots per bit, one shift-clock gap for STORE):
  doubles the STORE margin at the cost of a 1.275 us period at 80 MHz;
  documented as an option, not needed with 9.8 ns resolution.
* **Waveforms as shifter data** (2-bit shifter on D1/D2): the STORE edge
  would always coincide with a shift clock edge, violating 595 setup/hold.
* **PWM timers counting shift-clock edges** (`TIMDEC = 1/2`): half-shift
  resolution (39 ns) and the STORE edge lands on a clock edge plus sync
  latency; worse than the FlexIO-clock PWMs.
* **PLL3-derived FlexIO clock** (80 MHz, 1.2 us period, 833 kHz): simpler,
  no PLL programming, and within LED tolerance; PLL5 was chosen to hit exactly
  the requested 800 kHz and any other period.
* **Two 16-bit-wide groups or 4-bit groups**: with this pinout every
  partition either needs shifter 4 to drive pins that are not contiguous, or
  wastes as much as the 32-bit group; no DMA saving.  (Shifter 4 becomes
  useful with a different pinout, see section 9.)
* **eLCDIF (LCD controller) as a 6-bit parallel streamer**: pins 6, 7, 8, 9,
  36, 37, 10, 11, 12 are all LCD pins, but pins 34/35 are not, and DE/HSYNC
  cannot form the COMMON/STORE waveform.
* **FlexIO3**: has all the data pins too but no DMA request lines.
* **GPIO DMA with the clock generated by DMA writes** or bigger GPIO minor
  loops: the eDMA cannot pace 16 writes per 1.25 us with any margin.
* **Fixing the DMA priority swap while other channels run**: kept behind
  `SHIFTWS2811_DMA_PRIORITY`; disable it if another library allocates DMA
  before `begin()` and misbehaves.

---------------------------------------------------------------------------

## 6. Things assumed from the manual, to be confirmed on hardware

1. `TRIGGER_LATENCY`: the delay from a timer output edge to the enable of a
   timer triggered by it (assumed 1 clock).  Symptom of a wrong value: STORE
   not centred between shift clock rising edges; also shifts COMMON by the
   same amount (harmless).  Check STORE (pin 11) against SRCLK (pin 10).
2. Same-cycle chaining of timer 1 and 2 enables from timer 0.  A one-cycle
   stagger would move STORE by 9.8 ns; still within margin.
3. Lane order: shifter bit i -> pin PINSEL+i.  A reversed order would scramble
   which chain gets which data (blocks swapped in pairs); fix by inverting the
   `lanes[]` calculation, the fill core already supports any permutation.
4. eDMA sustaining one 32-byte minor loop per 625 ns -> `underruns()` stays 0.
   If not: `SHIFTWS2811_DMA_BURST 0` (plain 32-bit reads) is the first thing
   to try, then a longer `SHIFTWS2811_BIT_NS`.
5. SHIFTERR staying clear at the frame end thanks to the zero stream.
6. PLL5 lock and the resulting 12.8 MHz shift clock (`error()` = 6 if the
   PLL never locks).
7. The 32-byte source burst (`SSIZE = 5`) being accepted by this eDMA
   (configuration error otherwise: `DMA_ES` would show SAE/DAE; the channel
   would never complete and `stalls()` would count).

---------------------------------------------------------------------------

## 7. Tooling notes

* Reference manual text: `pdftotext -layout -f 2940 -l 3045 IMXRT1060RM.pdf`
  gives the FlexIO chapter; pages 1005..1092 the CCM; 89..200 the eDMA;
  399..720 the IOMUXC mux tables (grep for `FLEXIO2_FLEXIO10` etc.).
* NXP application notes AN12822 (8080 bus, RT1050) and AN12686 (camera) state
  the shifter 0/4 parallel restriction in plain words.
* NXP SDK `fsl_flexio.h` documents every TIMCFG/SHIFTCFG enum; `fsl_flexio_i2s.c`
  is the proven pattern for a second timer running phase-locked to a baud
  timer (`TimerEnableOnPrevTimerEnable`, FlexIO-clock decrement).
* KurtE's FlexIO_t4 library has working PLL5/PLL4 clock code
  (`setClockUsingVideoPLL`) and the pin tables.
* `docs/tools/kicad_netlist.py <top.kicad_sch> <outdir>` regenerates the
  netlist report from the KiCad files without kicad-cli and cross-checks it
  against the `.kicad_pcb`.

---------------------------------------------------------------------------

## 8. Numbers worth remembering

| item                                   | value |
|----------------------------------------|-------|
| old shift clock / bit rate             | 8.87 MHz / 554 kHz (204 MHz bus, 23 ticks) |
| new shift clock / bit rate             | 12.8 MHz / 800 kHz |
| FlexIO2 clock                          | 102.4 MHz from PLL5 (VCO 716.8 MHz) |
| DMA                                    | 32 bytes per request, 1.6 M requests/s, 51 MB/s |
| conversion buffers                     | 2 x 6 KB (12 LED bytes each) in OCRAM |
| DMA interrupt rate                     | one per 120 us (12 LED bytes = 96 bits) |
| conversion (fill core v2)              | 93 instructions per output and column, ~1500 per LED byte column |
| LED frame rate                         | 261 frames/s back to back; CPU load = frames/s x conversion time per frame |
| frame (125 RGB LEDs per output)        | 3.83 ms incl. 80 us reset |
| max bytes per output (timer 5 limit)   | 4096 (32768 bits) |
| 595 margin at STORE                    | 39 ns each side (5 ns required) |

---------------------------------------------------------------------------

## 9. A 256-channel version

### 9.1 What does not scale

* **16 chains x 16 outputs** needs 16 data pins.  FlexIO2 has 13 pins on the
  Teensy 4.1 and three are needed for SRCLK/STORE/COMMON, so at most 10
  chains (160 outputs) from FlexIO2.  FlexIO1 adds only pins 2, 3, 4, 33, 5
  (D4..D8) and would need a second module running as a clock slave (16-bit
  timer with `TIMDEC = 2` decrementing on FlexIO2's SRCLK looped into a
  FlexIO1 pin, its own DMA channel) - workable in principle (it is the FlexIO
  SPI-slave configuration) but two modules, two DMA streams and a
  synchroniser latency to verify.  Not recommended.
* **8 chains x 32 outputs with the present pinout**: the D10..D29 span forces
  32-bit shifts, so 32 shifts per bit = 128 bytes per bit = four 32-byte DMA
  minor loops per 1.25 us (one per 312 ns).  That is at or beyond the eDMA
  estimate of section 3.8.  Only viable at a lower bit rate; not recommended.

### 9.2 Recommended: 8 chains x 32 outputs on two 4-bit groups (new PCB)

Use both parallel-capable shifters, each with a contiguous 4-pin range:

| role                | FlexIO2 pins | Teensy pins   |
|---------------------|--------------|---------------|
| data group A (sh 0) | D16..D19     | 8, 7, 36, 37  |
| data group B (sh 4) | D0..D3       | 10, 12, 11, 13 |
| SRCLK               | D10          | 6             |
| STORE / OE          | D11          | 9             |
| COMMON              | D12          | 32            |
| spare               | D28, D29     | 35, 34        |

Configuration: shifters 0..3 chained into shifter 0 (`PINSEL 16, PWIDTH 3`,
shifter 3 `INSRC 0`), shifters 4..7 chained into shifter 4 (`PINSEL 0,
PWIDTH 3`, shifter 7 `INSRC 0`), all on timer 0 with `TIMCMP[15:8] = 32*2-1
= 63` (32 shifts per compare: 4 shifters x 32 bits / 4 bits per shift).
Both groups load on the same compare, so **one 32-byte DMA minor loop per bit
period** feeds all 256 channels: SHIFTBUF0..3 = group A, SHIFTBUF4..7 =
group B (a word holds 8 shifts of 4 bits, so 4 words per group per 32-shift
bit period).  That is a quarter of the DMA traffic of the current 128-channel
design.

Shift clock: 32 shifts per bit -> `SHIFT_DIV = 1` (4 clocks per shift,
25.6 MHz) at the same 102.4 MHz FlexIO clock and the same 128 clocks per
bit, so all PWM/one-shot constants stay; the STORE edge margin halves to
19.5 ns each side (still 4x the 595 requirement, but `TRIGGER_LATENCY` then
must be right to within a clock; the TSTOP stop-bit gap could buy back
margin if needed).  74AHCT595 f_max >= 115 MHz and AHCT245 buffers are fine at
25.6 MHz; keep clock and data traces short and equal-ish in length.

Timer 5 (`2*numBits - 1 <= 65535`) is per output, unchanged.  Timers 1, 2,
4..7 unchanged.  The 595 lane order: chain i -> lane `FlexIOpin - PINSEL`
inside its group (0..3).

Library changes:
* `SHIFTWS_SR_LEN` (16) becomes a parameter of the fill plan (32), with
  output `j` transmitted as shift `31 - j` and `wordsPerBit = 32 /
  shiftsPerWord`; the host test covers it by adding SR_LEN to its loops.
* Two groups need two `PINSEL/PWIDTH` values and two lane tables (one per
  parallel shifter); the DMA layout is unchanged (one 32-byte group per
  request, SHIFTBUF0..7 in order) but the fill must interleave the two
  groups' words: for each compare, words 0..3 belong to shifter chain A and
  words 4..7 to chain B.
* Pin validation: two ranges, timer pins outside both.
* Frame buffers: 256 x 125 x 3 = 96 KB each; three of them (288 KB) no longer
  fit in DTCM next to the rest of the firmware (RAM1 has ~170 KB free today).
  Put them in `DMAMEM` (RAM2, 505 KB free) or `EXTMEM` (PSRAM); the fill
  reads them sequentially so the cache copes.
* CPU: conversion cost per frame doubles (32 outputs instead of 16); with
  the second fill core (4.5) that is roughly 2 x 0.5 ms per 3.8 ms frame,
  ~25 percent, to be measured with `cpuLoad()`.  Note that 4-bit shifts use
  the generic read-modify-write path of the fill core; give it a
  specialisation (constant `wordsPerBit` = 2, combine the 8 shifts of a word
  in registers) and, since the new board has contiguous lanes, an identity
  expand (saves 8 loads per output and column).
* DMA interrupt rate halves for the same `BYTES_PER_DMA` (one minor loop per
  bit instead of two); buffers shrink to 12 x 8 x 8 words x 2 = 6 KB total
  at 12 LED bytes per half.

PCB changes: 4 x 74AHCT595 per chain (or two of the present output blocks
daisy-chained by wiring the first block's second-595 QH' to the second
block's DATA); SRCLK/STORE fan-out to 32 chips and 16 OE/COMMON lines (two
74AHCT245 per signal, two 74AHCT240 for OE, or one buffer per block pair);
keep the RC-delayed OE arrangement and the pots.  The 1k pull resistors only
load the COMMON drivers while the 595 outputs are off, so 256 of them are
fine.

### 9.3 Cheaper stepping stone: the same two-group pinout at 16 outputs

With the pinout above and the present 2 x 595 blocks, the 128-channel design
becomes 16 shifts per bit from two 4-bit groups: 2 words per group per bit in
shifters 0, 1 and 4, 5.  One 32-byte minor loop per bit (SHIFTBUF2, 3, 6, 7
carry don't-care words) instead of two, if the PCB is ever respun anyway.

### 9.4 Other routes

* Two ShiftWS2811 boards with two Teensy 4.1s sharing a frame source (e.g.
  the serial stream); trivial firmware-wise, but two controllers.
* The Teensy MicroMod (same RT1062) breaks out FlexIO2 D0..D12 contiguously
  (pins 10, 12, 11, 13, 40..45, 6, 9, 32) plus D16/D17 (pins 8, 7).  That
  allows an 8-pin group D0..D7 with the three clock signals on D8..D12: 128
  channels with one 16-byte DMA per bit, or 8 chains x 32 outputs with one
  32-byte DMA per bit, without a BGA board of one's own.
* A custom board with the bare RT1062 exposes all 32 FlexIO2 pins (and the
  16 of FlexIO1): two contiguous groups of up to 16 pins, i.e. 16 chains x 16
  outputs at the present shift rate, or more.  The price is the BGA-196
  layout, external flash and power, and either PJRC's bootloader chip (keeps
  Teensyduino) or a move to the NXP SDK; the library's pin lookup
  (`portOutputRegister`/`digitalPinToBit`) would need a pin table for it.

---------------------------------------------------------------------------

## 10. First hardware results and CPU load (2026-09-17)

The rewrite ran on the QuinCube on the first try: LEDs correct, exact
800 kHz.  Animations that are not CPU bound run at a higher frame rate than
before (an LED frame takes 3.83 ms instead of 5.5 ms).  CPU bound animations
ran slower: the Orbs animation dropped from 78 to 55 fps.

Why: the conversion runs in the DMA interrupt for every LED frame, and LED
frames are sent back to back (dithering re-sends the current frame), so it
is a continuous background load that scales with the LED frame rate: 261
frames/s now against 181 before, 1.44x more conversions per second.  The
first fill core cost about as much per frame as the old `fillbits`
(3000+ instructions per column, see 4.5), so the background load grew by
that factor.  Solving `fps = (1 - load) / T_anim` for both measurements
gives about 2.2 ms of conversion per LED frame (40 percent load before, 57
percent after) and a 7.7 ms animation frame, consistent with the assembly.

Fix (this version): the fill core rewrite (4.5) cuts the instruction count
per column by 2 to 2.5x and removes the stack spilling.  Estimated load
around 15 percent (roughly 1000 cycles per column x 375 columns x 261
frames/s at 816 MHz plus cache maintenance), which would put Orbs at about
100 fps.  `cpuLoad()` reports the measured percentage (DWT cycle counts
taken in the DMA and FlexIO interrupts and in `transfer()`); measure it.

Knobs if the load is still too high:

* `SHIFTWS2811_RESET_US`: a longer gap lowers the LED frame rate and with it
  the load, linearly (1750 us gives the old 181 frames/s); `setDitherBits`
  adapts the dither depth to the frame time automatically.
* `SHIFTWS2811_BITDATA_DTCM 1`: conversion buffers in DTCM instead of OCRAM.
  Removes the write-allocate line fills (16 per column) and the cache flush
  (16 lines per column), maybe a third of the remaining cost.  The eDMA then
  reads the buffers through the core's AHBS port (RM: TCM is accessible to
  DMA through AHBS); untested on this board, watch `underruns()`.
* `SHIFTWS2811_BYTES_PER_DMA` (now 12, 120 us per interrupt) only changes the
  interrupt rate; the fixed cost per interrupt is a few hundred cycles and
  irrelevant next to the conversion, but a larger value tolerates longer
  interrupt latency from other code.

Further fill core speedups not done yet: identity expand for boards with
contiguous lanes, loading 4 pixel columns per 32-bit read (saves 6 of 8
pixel loads per output and column, needs 4 accumulators), and converting
once per animation frame instead of once per LED frame (only possible
without dithering, which changes the corrected byte every frame).
