#include "Arduino.h"
#include "simultaneoussteppers.h"

#define CPU_FREQ 16000000U

static uint16_t shiftData;
static uint8_t microstepping1Bits[5] = {0, 1, 0, 1, 1};
static uint8_t microstepping2Bits[5] = {0, 0, 1, 1, 1};
static uint8_t microstepping3Bits[5] = {0, 0, 0, 0, 1};

static uint8_t enablePinsShield[3] = {STEPPER_C_ENABLE, STEPPER_B_ENABLE, STEPPER_A_ENABLE};
static uint8_t microstepping1PinsShield[3] = {STEPPER_C_MS_1, STEPPER_B_MS_1, STEPPER_A_MS_1};
static uint8_t microstepping2PinsShield[3] = {STEPPER_C_MS_2, STEPPER_B_MS_2, STEPPER_A_MS_2};
static uint8_t microstepping3PinsShield[3]  = {STEPPER_C_MS_3, STEPPER_B_MS_3, STEPPER_A_MS_3};

static uint8_t timer_0_prescaler;
static uint8_t timer_1_prescaler;
static uint8_t timer_2_prescaler;

static void shift16(uint8_t latchPin, uint8_t dataPin, uint8_t clockPin, uint16_t val)
{
    Serial.println(val, BIN);

    digitalWrite(latchPin, LOW);

    for (uint8_t i = 0; i < 16; i++)  { 
        digitalWrite(dataPin, !!(val & (1 << (15 - i))));
                  
        digitalWrite(clockPin, HIGH);
        digitalWrite(clockPin, LOW);            
    }

    digitalWrite(latchPin, HIGH);
}

static void initTimer(uint8_t timer){

    if(timer == 0){
        TCCR0A = (1<<WGM01);
        TCCR0B = 0;
        TIMSK0 = (1<<OCIE0A);
    }else if(timer == 1){
        TCCR1A = 0;
        TCCR1B = (1<<WGM12);
        TIMSK1 = (1<<OCIE1A);
    }else if(timer == 2){
        TCCR2A = (1<<WGM21);
        TCCR2B = 0;
        TIMSK2 = (1<<OCIE2A);
    }
}

static uint8_t setTimer(uint8_t timer, uint8_t rpm, uint16_t stepsPerRev, uint8_t microSteps){

    if(!rpm || !stepsPerRev){
        return;
    }

    uint32_t clock_per_minute = CPU_FREQ * 60;
    uint32_t steps_needed = clock_per_minute / stepsPerRev / rpm / (1 << microSteps);

    if(timer == 0){
        OCR0A = (uint8_t) (steps_needed / 2048);
    }else if(timer == 1){
        OCR1A = (uint16_t) (steps_needed / 2048);
    }else if(timer == 2){
        OCR2A = (uint8_t) (steps_needed / 2048);
    }
}

static void startTimer(uint8_t timer){

    if(timer == 0){
        TCCR0B |= ((1<<CS00) | (1<<CS02));
    }else if(timer == 1){
        TCCR1B |= ((1<<CS10) | (1<<CS12));
    }else if(timer == 2){
        TCCR2B |= ((1<<CS20) | (1<<CS21) | (1<<CS22));
    }
}

static void stopTimer(uint8_t timer){

    if(timer == 0){
        TCCR0B &= ~((1<<CS00) | (1<<CS01) | (1<<CS02));
    }else if(timer == 1){
        TCCR1B &= ~((1<<CS10) | (1<<CS11) | (1<<CS12));
    }else if(timer == 2){
        TCCR2B &= ~((1<<CS20) | (1<<CS21) | (1<<CS22));
    }
}

SimulStepper::SimulStepper(uint8_t timer){
    this->timer = timer;
}

void SimulStepper::init(uint8_t stepPin, uint8_t directionPin){
    this->mode = ARDUINO_MODE;
    this->stepPin = stepPin;
    this->directionPin = directionPin;
    pinMode(this->stepPin, OUTPUT);
    pinMode(this->directionPin, OUTPUT);
}

void SimulStepper::init(uint8_t stepPin, uint8_t directionPin, uint8_t rpm, uint16_t stepsPerRev){
    this->init(stepPin, directionPin);
    this->rpm = rpm;
    this->stepsPerRev = stepsPerRev;
    initTimer(this->timer);
    setTimer(this->timer, this->rpm, this->stepsPerRev, this->microSteps);
}

void SimulStepper::init(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin){
    this->init(stepPin, directionPin);
    this->enablePin = enablePin;
    pinMode(this->enablePin, OUTPUT);
}

void SimulStepper::init(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin, uint8_t rpm, uint16_t stepsPerRev){
    this->init(stepPin, directionPin, rpm, stepsPerRev);
    this->enablePin = enablePin;
    pinMode(this->enablePin, OUTPUT);
}

void SimulStepper::microSteppingInit(uint8_t microStepping1Pin, uint8_t microStepping2Pin, uint8_t microStepping3Pin){
    this->microStepping1Pin = microStepping1Pin;
    this->microStepping2Pin = microStepping2Pin;
    this->microStepping3Pin = microStepping3Pin;
    pinMode(this->microStepping1Pin, OUTPUT);
    pinMode(this->microStepping2Pin, OUTPUT);
    pinMode(this->microStepping3Pin, OUTPUT);
}

void SimulStepper::init(uint8_t stepPin, uint8_t directionPin, uint8_t microStepping1Pin, uint8_t microStepping2Pin, uint8_t microStepping3Pin){
    this->init(stepPin, directionPin);
    microSteppingInit(microStepping1Pin, microStepping2Pin, microStepping3Pin);
}

void SimulStepper::init(uint8_t stepPin, uint8_t directionPin, uint8_t microStepping1Pin, uint8_t microStepping2Pin, uint8_t microStepping3Pin, uint8_t rpm, uint16_t stepsPerRev){
    this->init(stepPin, directionPin, rpm, stepsPerRev);
    microSteppingInit(microStepping1Pin, microStepping2Pin, microStepping3Pin);
}

void SimulStepper::init(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin, uint8_t microStepping1Pin, uint8_t microStepping2Pin, uint8_t microStepping3Pin){
    this->init(stepPin, directionPin, enablePin);
    microSteppingInit(microStepping1Pin, microStepping2Pin, microStepping3Pin);
}

void SimulStepper::init(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin, uint8_t microStepping1Pin, uint8_t microStepping2Pin, uint8_t microStepping3Pin, uint8_t rpm, uint16_t stepsPerRev){
    this->init(stepPin, directionPin, enablePin, rpm, stepsPerRev);
    microSteppingInit(microStepping1Pin, microStepping2Pin, microStepping3Pin);
}

void SimulStepper::shieldInit(void){
    this->mode = SHIELD_MODE;

    pinMode(SHIELD_LATCH_PIN, OUTPUT);
    pinMode(SHIELD_CLOCK_PIN, OUTPUT);
    pinMode(SHIELD_DATA_PIN, OUTPUT);
    pinMode(SHIELD_STEP_1_PIN, OUTPUT);
    pinMode(SHIELD_STEP_2_PIN, OUTPUT);
    pinMode(SHIELD_STEP_3_PIN, OUTPUT);
    pinMode(SHIELD_DIR_1_PIN, OUTPUT);
    pinMode(SHIELD_DIR_2_PIN, OUTPUT);
    pinMode(SHIELD_DIR_3_PIN, OUTPUT);

    if(this->timer == 0){
        this->stepPin = SHIELD_STEP_3_PIN;
        this->directionPin = SHIELD_DIR_3_PIN;
    }else if(this->timer == 1){
        this->stepPin = SHIELD_STEP_2_PIN;
        this->directionPin = SHIELD_DIR_2_PIN;
    }else if(this->timer == 2){
        this->stepPin = SHIELD_STEP_1_PIN;
        this->directionPin = SHIELD_DIR_1_PIN;
    }

    digitalWrite(SHIELD_LATCH_PIN, HIGH);
}

void SimulStepper::shieldInit(uint8_t rpm, uint16_t stepsPerRev){
    this->shieldInit();
    initTimer(this->timer);
    this->rpm = rpm;
    this->stepsPerRev = stepsPerRev;
    setTimer(this->timer, this->rpm, this->stepsPerRev, this->microSteps);
}

void SimulStepper::setEnable(bool state){
    this->enable = state;
    if((this->mode == SHIELD_MODE) || (this->enablePin != 0xff)){
        if(this->mode == ARDUINO_MODE){
            digitalWrite(this->enablePin, !this->enable);
        }else{
            if(this->enable){
                shiftData &= ~(1<<enablePinsShield[this->timer]);
            }else{
                shiftData |= (1<<enablePinsShield[this->timer]);
            }
            shift16(SHIELD_LATCH_PIN, SHIELD_DATA_PIN, SHIELD_CLOCK_PIN, shiftData);
        }
    }
}

bool SimulStepper::getEnable(void){
    return this->enable;
}

void SimulStepper::setRpm(uint8_t rpm){
    this->rpm = rpm;
    setTimer(this->timer, this->rpm, this->stepsPerRev, this->microSteps);
}

uint8_t SimulStepper::getRpm(void){
    return this->rpm;
}

void SimulStepper::setStepsPerRev(uint16_t stepsPerRev){
    this->stepsPerRev;
    setTimer(this->timer, this->rpm, this->stepsPerRev, this->microSteps);
}

uint16_t SimulStepper::getStepsPerRev(void){
    return this->stepsPerRev;
}

void SimulStepper::setMicrosteps(uint8_t microSteps){
    if((this->mode == ARDUINO_MODE) && ((this->microStepping1Pin == 0xff) || (this->microStepping1Pin == 0xff) || (this->microStepping1Pin == 0xff))){
        return;
    }

    this->microSteps = microSteps;
    if(this->mode == ARDUINO_MODE){
        digitalWrite(this->microStepping1Pin, microstepping1Bits[this->microSteps]);
        digitalWrite(this->microStepping2Pin, microstepping2Bits[this->microSteps]);
        digitalWrite(this->microStepping3Pin, microstepping3Bits[this->microSteps]);
    }else{
        if(microstepping1Bits[this->microSteps]){
            shiftData |= (1<<microstepping1PinsShield[this->timer]);
        }else{
            shiftData &= ~(1<<microstepping1PinsShield[this->timer]);
        }
        if(microstepping2Bits[this->microSteps]){
            shiftData |= (1<<microstepping2PinsShield[this->timer]);
        }else{
            shiftData &= ~(1<<microstepping2PinsShield[this->timer]);
        }
        if(microstepping3Bits[this->microSteps]){
            shiftData |= (1<<microstepping3PinsShield[this->timer]);
        }else{
            shiftData &= ~(1<<microstepping3PinsShield[this->timer]);
        };     
        shift16(SHIELD_LATCH_PIN, SHIELD_DATA_PIN, SHIELD_CLOCK_PIN, shiftData);
    }

    setTimer(this->timer, this->rpm, this->stepsPerRev, this->microSteps);
}

uint8_t SimulStepper::getMicrosteps(void){
    return this->microSteps;
}

void SimulStepper::setDirection(uint8_t direction){
    this->direction = direction;
    digitalWrite(this->directionPin, this->direction);
}

uint8_t SimulStepper::getDirection(void){
    return this->direction;
}

void SimulStepper::step(uint16_t steps){
    

    this->steps += steps * 2;
    if(!timer_active){
        startTimer(this->timer);
        this->timer_active = true;
    }
}

uint16_t SimulStepper::stepsRemaining(void){
    return this->steps;
}

void SimulStepper::stop(void){
    stopTimer(this->timer);
    this->steps = 0;
}

void SimulStepper::callback(void){

    if(!(--(this->steps))){
        stopTimer(this->timer);
        this->timer_active = false;
    }

    digitalWrite(this->stepPin, !digitalRead(this->stepPin));
}

ISR(TIMER0_COMPA_vect) {
    Stepper_C.callback();
}

ISR(TIMER1_COMPA_vect) {
    Stepper_B.callback();
}

ISR(TIMER2_COMPA_vect) {
    Stepper_A.callback();
}

SimulStepper Stepper_A(2);
SimulStepper Stepper_B(1);
SimulStepper Stepper_C(0); 