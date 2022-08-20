# simultaneoussteppers
Drive 3 stepper motors simultaneously with the Arduino UNO

I needed to drive 3 stepper motors simultaneously for a project so I came up with this piece of code for the Arduino UNO. Missing many features like for example acceleration and deceleration. Also ONLY works with the UNO at this point.

Once Stepper_C is initialized, the library uses Timer0 which is responsible for the delay(), delayMicrosecond(), millis() and micros() functions. Thats why I use simple NOP delays in the demo sketch with all 3 steppers.

I also designed a shield for the library featuring the A4988 modules which can be initialized directly. The pins are preset which makes it easier to use. The PCB still has some errors but I will attach the schematic.