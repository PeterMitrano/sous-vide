# Info on pin numbers

Arduino ESP has macros for D0-16 that map to the GPIO #

D0 -> GPIO 16 (Red LED)
D4 -> GPIO 2  (Blue LED on ESP)

### Pins I've tested with LED output

 - D0
 - D1
 - D2
 - D3
 - D4
 - D5
 - D6
 - D7
 - D8
 - RX
 - TX (no serial if you use this)
 - GPIO10 (SD3 on board)

### Pins that FAIL the LED test

 - TX, kinda because no serial
 - GPIO9, reserved for ESP
 - GPIO8, reserved for ESP
 - GPIO7, reserved for ESP
 - GPIO6, reserved for ESP

### Pins I've tested with Button Input

 - D0
 - D1
 - D2
 - D3
 - D4
 - D5
 - D6
 - D7

### Pins that FAIL the Button test

 - D8

