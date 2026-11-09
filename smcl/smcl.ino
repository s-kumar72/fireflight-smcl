#include <cstdio>
#include <Arduino.h>

#define POT A0

char buffer[60];


void potControl(){
  int potValue = analogRead(POT);
  snprintf(buffer, sizeof(buffer), "Pot value: %d", potValue);
  Serial.println(buffer);

  float motorSpeed = potValue;

}

void setup() {
  // put your setup code here, to run once:


}

void loop() {
  // put your main code here, to run repeatedly:

}
