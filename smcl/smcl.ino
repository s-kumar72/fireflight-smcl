#include <Arduino.h>

const uint8_t CMD_MAX = 32;
char cmdBuf[CMD_MAX];
uint8_t cmdLen = 0;

void handleRequest(const char* cmd){
  if (strcmp(cmd, "REQ:POT") == 0){
    Serial.print("POT: ");
    Serial.print(potRaw());
    Serial.print('\n');
  }
}

void serialUpdate(){
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0){
        handleRequest(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < CMD_MAX - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  potSetup();
  stepperSetup();

}

void loop() {
  // put your main code here, to run repeatedly:
  potUpdate();
  serialUpdate();

  stepperMoveToDeg(potDegrees());

  stepperUpdate();
}
