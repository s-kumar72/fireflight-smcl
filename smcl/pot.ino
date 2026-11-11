#include <Arduino.h>

const uint8_t POT_PIN = A0;
const float POT_TRAVEL_DEG = 280.0; // from datasheet for Bourns 3386 trimpot

int potRawValue = 0;
float potDegValue = 0.0;

void potSetup(){
  // no pin node for analog pins
}

void potUpdate(){
  potRawValue = analogRead(POT_PIN);
  potDegValue = potRawValue * (POT_TRAVEL_DEG / 1023.0);
}

// to interface with smcl.ino
int potRaw(){
  return potRawValue;
}

float potDegrees(){
  return potDegValue;
}
