#ifndef SIMULTANEOUSSTEPPERS_H
#define SIMULTANEOUSSTEPPERS_H

#include "Arduino.h"
#include <stdint.h>

#define SHIELD_LATCH_PIN    6
#define SHIELD_CLOCK_PIN    5
#define SHIELD_DATA_PIN     7

#define SHIELD_STEP_1_PIN   9
#define SHIELD_STEP_2_PIN   11
#define SHIELD_STEP_3_PIN   3

#define SHIELD_DIR_1_PIN    10
#define SHIELD_DIR_2_PIN    12
#define SHIELD_DIR_3_PIN    2

#define STEPPER_A_ENABLE    4
#define STEPPER_B_ENABLE    3
#define STEPPER_C_ENABLE    12

#define STEPPER_A_MS_1      5
#define STEPPER_B_MS_1      2
#define STEPPER_C_MS_1      13

#define STEPPER_A_MS_2      6
#define STEPPER_B_MS_2      1
#define STEPPER_C_MS_2      14

#define STEPPER_A_MS_3      7
#define STEPPER_B_MS_3      0
#define STEPPER_C_MS_3      15

typedef enum{
    ARDUINO_MODE,
    SHIELD_MODE
}initMode_t;

typedef enum{
    MICROSTEPS_1,
    MICROSTEPS_2,
    MICROSTEPS_4,
    MICROSTEPS_8,
    MICROSTEPS_16
}microSteps_t;

typedef enum{
    CLOCKWISE,
    COUNTERCLOCKWISE
}direction_t;

class SimulStepper {
    public:
        SimulStepper(uint8_t timer);
        void init(uint8_t stepPin, uint8_t directionPin);
        void init(uint8_t stepPin, uint8_t directionPin, uint8_t rpm, uint16_t stepsPerRev);
        void init(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin);
        void init(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin, uint8_t rpm, uint16_t stepsPerRev);
        void init(uint8_t stepPin, uint8_t directionPin, uint8_t microStepping1Pin, uint8_t microStepping2Pin, uint8_t microStepping3Pin);
        void init(uint8_t stepPin, uint8_t directionPin, uint8_t microStepping1Pin, uint8_t microStepping2Pin, uint8_t microStepping3Pin, uint8_t rpm, uint16_t stepsPerRev);
        void init(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin, uint8_t microStepping1Pin, uint8_t microStepping2Pin, uint8_t microStepping3Pin);
        void init(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin, uint8_t microStepping1Pin, uint8_t microStepping2Pin, uint8_t microStepping3Pin, uint8_t rpm, uint16_t stepsPerRev);
        void shieldInit(void);
        void shieldInit(uint8_t rpm, uint16_t stepsPerRev);
        void setEnable(bool state);
        bool getEnable(void);
        void setRpm(uint8_t rpm);
        uint8_t getRpm(void);
        void setStepsPerRev(uint16_t stepsPerRev);
        uint16_t getStepsPerRev(void);
        void setMicrosteps(uint8_t microSteps);
        uint8_t getMicrosteps(void);
        void setDirection(uint8_t direction);
        uint8_t getDirection();
        void step(uint16_t steps);
        uint16_t stepsRemaining(void);
        void stop(void);
        void callback(void);
    private:
        void microSteppingInit(uint8_t microStepping1Pin, uint8_t microStepping2Pin, uint8_t microStepping3Pin);
        bool        enable;
        uint8_t     timer;
        uint8_t     rpm = 0;
        uint16_t    stepsPerRev = 0;
        uint8_t     microSteps = MICROSTEPS_1;
        uint8_t     direction = CLOCKWISE;
        uint16_t    steps = 0;
        uint8_t     stepPin = 0xff;
        uint8_t     directionPin = 0xff;
        uint8_t     enablePin = 0xff;
        uint8_t     microStepping1Pin = 0xff;
        uint8_t     microStepping2Pin = 0xff;
        uint8_t     microStepping3Pin = 0xff;
        uint8_t     timerOverflows = 1;
        uint8_t     currentOverflows = 1;
        uint8_t     mode = 0;
        bool        timer_active = false;
};

extern SimulStepper Stepper_A;
extern SimulStepper Stepper_B;
extern SimulStepper Stepper_C; 

#endif // SIMULTANEOUSSTEPPERS_H