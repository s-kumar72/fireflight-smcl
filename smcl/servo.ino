#include <Servo.h>

#define SERVO_PIN 9

const int   SERVO_MIN_US  = 1000;
const int   SERVO_MAX_US  = 2000;
const float SERVO_MIN_DEG = 0.0;
const float SERVO_MAX_DEG = 180.0;

const float SERVO_TEMP_MIN_C = 20.0;
const float SERVO_TEMP_MAX_C = 35.0;
const unsigned long SERVO_SENSOR_PERIOD_MS = 100;
const float SERVO_DEADBAND_DEG = 2.0;

Servo servo;
float servoDeg = 90.0;
bool servoSensorMode = true;
unsigned long servoLastMs = 0;

void servoSetDeg(float deg) {
  deg = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  int us = SERVO_MIN_US + (deg - SERVO_MIN_DEG) * (SERVO_MAX_US - SERVO_MIN_US) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
  servo.writeMicroseconds(us);
  servoDeg = deg;
}

void servoSetup() {
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoSetDeg(90.0);
}

float servoGetDeg() {
  return servoDeg;
}

void servoSetSensorMode(bool on) {
  servoSensorMode = on;
}

float servoTempToDeg(float tempC) {
  float frac = (tempC - SERVO_TEMP_MIN_C) / (SERVO_TEMP_MAX_C - SERVO_TEMP_MIN_C);
  frac = constrain(frac, 0.0, 1.0);
  return SERVO_MIN_DEG + frac * (SERVO_MAX_DEG - SERVO_MIN_DEG);
}

void servoUpdate() {
  if (!servoSensorMode) return;
  if (millis() - servoLastMs < SERVO_SENSOR_PERIOD_MS) return;
  servoLastMs = millis();
  float target = servoTempToDeg(thermReadC());
  if (fabs(target - servoDeg) >= SERVO_DEADBAND_DEG) {
    servoSetDeg(target);
  }
}

void servoHandleCommand(const char* arg) {
  servoSensorMode = false;
  servoSetDeg(atof(arg));
}
