#include <Arduino.h>

const uint8_t STEPPER_STEP_PIN = 10;
const uint8_t STEPPER_DIR_PIN = 11;
const long STEPPER_STEPS_PER_REV = 200L * 8;
const unsigned int STEPPER_INTERVAL_US = 1000;
const float STEPPER_DEG_PER_STEP = 360.0 / STEPPER_STEPS_PER_REV;

long stepperPos = 0;
long stepperTarget = 0;
unsigned long stepperLastUs = 0;

void stepperSetup(){
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
}

void stepperUpdate(){
    if (stepperPos == stepperTarget) return;
    if (micros() - stepperLastUs < STEPPER_INTERVAL_US) return;

    stepperLastUs = micros();

    bool forward = stepperTarget > stepperPos;
    digitalWrite(STEPPER_DIR_PIN, forward ? HIGH : LOW); // send high if forward is true
    digitalWrite(STEPPER_STEP_PIN, HIGH); // EasyDriver stepper driver steps on rising edge
    delayMicroseconds(2);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    stepperPos += forward ? 1 : -1;
}

void stepperMoveToDeg(float deg){
    stepperTarget = lround(deg / STEPPER_DEG_PER_STEP);
}

// GUI use
float stepperMoveRelDeg(float deg){
    long steps = lround(deg / STEPPER_DEG_PER_STEP);
    stepperTarget = stepperPos + steps;
    return steps * STEPPER_DEG_PER_STEP;
}

float stepperPositionDeg(){
    return stepperPos * STEPPER_DEG_PER_STEP;
}