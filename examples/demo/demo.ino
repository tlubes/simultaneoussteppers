#include "simultaneoussteppers.h"

void setup(void){
  Stepper_A.shieldInit(200, 120);
  Stepper_A.setEnable(false);
  Stepper_A.setDirection(CLOCKWISE);
  Stepper_A.setMicrosteps(MICROSTEPS_1);
  Stepper_B.shieldInit(200, 60);
  Stepper_B.setEnable(false);
  Stepper_B.setDirection(CLOCKWISE);
  Stepper_B.setMicrosteps(MICROSTEPS_1);
  Stepper_C.shieldInit(200, 30);
  Stepper_C.setEnable(false);
  Stepper_C.setDirection(CLOCKWISE);
  Stepper_C.setMicrosteps(MICROSTEPS_1);

  // Timer1 has some issues after reset but I did not have time to debug them yet
  // We do two cycles with the drivers disabled so we have a known state

  Stepper_A.step(200);
  Stepper_B.step(200);
  Stepper_C.step(200);
  simultaneoussteppers_delay_second();
  simultaneoussteppers_delay_second();
  simultaneoussteppers_delay_second();

  Stepper_A.step(200);
  Stepper_B.step(20);
  Stepper_C.step(200);
  simultaneoussteppers_delay_second();
  simultaneoussteppers_delay_second();
  simultaneoussteppers_delay_second();

  Stepper_A.setEnable(true);
  Stepper_B.setEnable(true);
  Stepper_C.setEnable(true);
}

void simultaneoussteppers_delay_second(void){
  uint32_t i = 0;
  while(i < 2250000){
    i++; 
    __asm__("nop\n\t");
  }
}

void simultaneoussteppers_delay_milli(void){
  uint32_t i = 0;
  while(i < 2050){
    i++; 
    __asm__("nop\n\t");
  }
}

void loop(void){
  Stepper_A.step(200);
  Stepper_B.step(200);
  Stepper_C.step(200);
  simultaneoussteppers_delay_second();
  simultaneoussteppers_delay_second();
  simultaneoussteppers_delay_second();
}