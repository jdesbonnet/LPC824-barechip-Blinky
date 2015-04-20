A very simple bare minimum 'Blinky' (LED blinker) running on a NXP LPC824 (LPC824M201JDH20 TSSOP20 package) mounted on a breakout board with nothing connected but a single LED and Serial Wire Debug port.

This is adapted from example project 'periph_blinky' from the NXP examples. The main modifiction was to remove the clock configuration which switches clock source from the default 12MHz internal clock to either external crystal oscillator or the internal clock through a PLL. Clock config is just commented out in this example.

Setup:

A LPC824M201JDH20 (LPC824 TSSOP20) package is mounted on a breakout board and then mounted on a breadboard. The SWD pins (RESET, SWCLK, SWDIO, GND, 3.3V) are connected to LPC Link 1.1 board (a LPCXpresso LPC812 dev board with the target part of the board disconnected).  

The chip can also be programmed with Flash Magic via FTDI cable.


