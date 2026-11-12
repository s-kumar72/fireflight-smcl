#include <Arduino.h>
#include <string.h>

// Standalone sensor simulator for ros2_ws/src/comms/comms/comms_node.py.
// Send a newline-terminated request at 115200 baud to receive one raw reading:
// REQ:POT -> POT:<ADC>, REQ:THERM -> TRM:<ADC>, REQ:ULTR -> ULT:<ADC>,
// REQ:SLOT -> SLT:<0 or 1>, REQ:TOF -> TOF:<mm>, REQ:DCE -> DCE:<counts>.
// Motor commands receive a brief description; no motors are driven:
// SER:<angle>, STP:<+|->:<angle>, DCM:<P|V>:<+|->:<value>.
// Unknown or invalid commands are ignored. No unsolicited output.

const size_t COMMAND_CAPACITY = 32;
char commandBuffer[COMMAND_CAPACITY];
size_t commandLength = 0;
bool commandOverflow = false;

void sendReading(const char* header, long value) {
  Serial.print(header);
  Serial.print(':');
  Serial.print(value);
  Serial.print('\n');
}

bool parseMagnitude(const char* text, long maximum, long& value) {
  if (*text == '\0') {
    return false;
  }
  value = 0;
  for (; *text != '\0'; ++text) {
    if (*text < '0' || *text > '9') {
      return false;
    }
    const long digit = *text - '0';
    if (value > (maximum - digit) / 10) {
      return false;
    }
    value = value * 10 + digit;
  }
  return value <= maximum;
}

void handleMotorCommand(const char* command) {
  long value;
  if (strncmp(command, "SER:", 4) == 0) {
    if (!parseMagnitude(command + 4, 180, value)) {
      return;
    }
    Serial.print("Set servo position to ");
    Serial.print(value);
    Serial.print(" degrees.\n");
  } else if (strncmp(command, "STP:", 4) == 0) {
    const char direction = command[4];
    if ((direction != '+' && direction != '-') || command[5] != ':' ||
        !parseMagnitude(command + 6, 32767, value)) {
      return;
    }
    Serial.print("Set stepper position to ");
    Serial.print(direction);
    Serial.print(value);
    Serial.print(" degrees.\n");
  } else if (strncmp(command, "DCM:", 4) == 0) {
    const char mode = command[4];
    if ((mode != 'P' && mode != 'V') || command[5] != ':' ||
        (command[6] != '+' && command[6] != '-') || command[7] != ':' ||
        !parseMagnitude(command + 8, mode == 'P' ? 32767 : 60, value)) {
      return;
    }
    Serial.print(mode == 'P' ? "Set DC motor position to " : "Set DC motor velocity to ");
    Serial.print(command[6]);
    Serial.print(value);
    Serial.print(mode == 'P' ? " degrees.\n" : " RPM.\n");
  }
}

void handleRequest(const char* command) {
  // random() excludes the upper bound.
  if (strcmp(command, "REQ:POT") == 0) {
    sendReading("POT", random(0, 1024));
  } else if (strcmp(command, "REQ:THERM") == 0) {
    // Exclude divider endpoints, where temperature is undefined.
    sendReading("TRM", random(1, 1023));
  } else if (strcmp(command, "REQ:ULTR") == 0) {
    // MaxSonar analog counts: two counts/inch, approximately 6–254 inches.
    sendReading("ULT", random(12, 509));
  } else if (strcmp(command, "REQ:SLOT") == 0) {
    sendReading("SLT", random(0, 2));
  } else if (strcmp(command, "REQ:TOF") == 0) {
    sendReading("TOF", random(40, 4001));
  } else if (strcmp(command, "REQ:DCE") == 0) {
    sendReading("DCE", random(0, 4097));
  } else {
    handleMotorCommand(command);
  }
}

void setup() {
  Serial.begin(115200);
  // An unconnected A0 and startup timing vary the sequence between resets.
  randomSeed(analogRead(A0) ^ micros());
}

void loop() {
  while (Serial.available() > 0) {
    const char character = Serial.read();
    if (character == '\r') {
      continue;
    }
    if (character == '\n') {
      if (!commandOverflow && commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        handleRequest(commandBuffer);
      }
      commandLength = 0;
      commandOverflow = false;
    } else if (!commandOverflow) {
      if (commandLength < COMMAND_CAPACITY - 1) {
        commandBuffer[commandLength++] = character;
      } else {
        // Discard the entire oversized line and resume at the next newline.
        commandOverflow = true;
      }
    }
  }
}
