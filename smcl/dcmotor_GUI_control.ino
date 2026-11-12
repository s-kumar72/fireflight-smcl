/*
  DC motor control: POSITION (angle) or VELOCITY (RPM) using PID

  Controller: Arduino Uno
  Feedback:   Quadrature encoder (A on pin 2 interrupt, B on pin 3)
  Driver:     H-bridge (PWM on 6, direction on 7 / 8)

  Serial commands (115200 baud, end each line with Newline):

    P <deg>   Rotate to an absolute angle in degrees,  e.g.  P 90   or  P -180
    V <rpm>   Run at a constant speed in RPM,          e.g.  V 40   or  V -25

  Negative angle / RPM = opposite direction.
  Stopping the motor:  V 0
  Returning to the start position:  P 0
*/

// -------- PINS --------

const int encoderA = 2;
const int encoderB = 3;

const int motorPWM = 6;
const int motorIN1 = 7;
const int motorIN2 = 8;

// -------- ENCODER --------

// Encoder counts per output-shaft revolution,
// using the selected interrupt edge (RISING on channel A)
const float ENCODER_CPR = 420.0;

volatile long encoderCount = 0;

// -------- LIMITS --------

const float MAX_RPM = 60.0;     // maximum commanded speed (+/-)
const int   MAX_PWM = 255;

// Minimum PWM needed to overcome motor friction (tune for your motor).
// Applied only while the position controller is actively driving the motor.
const int   MIN_PWM_POS = 35;

// Position is considered "reached" inside this band
const float POSITION_DEADBAND_DEG = 1.0;

// -------- CONTROL MODE --------

enum Mode { MODE_IDLE, MODE_POSITION, MODE_VELOCITY };
Mode mode = MODE_IDLE;

float targetAngleDeg = 0;
float targetRPM = 0;

// -------- PID --------

struct PID {
  float kp, ki, kd;
  float integral;
  float lastMeasurement;
  float integralLimit;
};

//                kp    ki     kd     I   lastMeas  Ilimit
PID posPID = {   3.0,  0.10,  0.05,   0,  0,        100.0 };
PID velPID = {  10.0,  0.50,  0.001,  0,  0,        200.0 };

// Derivative is taken on the measurement to avoid "kick"
// when the target changes suddenly.
float computePID(PID &pid, float target, float measurement, float dt) {

  float error = target - measurement;

  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -pid.integralLimit, pid.integralLimit);

  float derivative = -(measurement - pid.lastMeasurement) / dt;
  pid.lastMeasurement = measurement;

  return pid.kp * error +
         pid.ki * pid.integral +
         pid.kd * derivative;
}

void resetPID(PID &pid, float measurement) {
  pid.integral = 0;
  pid.lastMeasurement = measurement;
}

// -------- TIMING --------

const unsigned long POS_INTERVAL = 20;    // ms, position loop
const unsigned long VEL_INTERVAL = 100;   // ms, velocity loop
const unsigned long PRINT_INTERVAL = 100; // ms

unsigned long lastPIDTime = 0;
unsigned long lastPrintTime = 0;

long previousEncoderCount = 0;

float actualAngleDeg = 0;
float actualRPM = 0;
float lastPWMSigned = 0;

// -------- SERIAL INPUT --------

char inputBuffer[24];
byte inputLen = 0;

// -------- ENCODER INTERRUPT --------

void encoderISR() {

  if (digitalRead(encoderB))
    encoderCount++;
  else
    encoderCount--;
}

long readEncoder() {

  long c;
  noInterrupts();
  c = encoderCount;
  interrupts();
  return c;
}

// -------- MOTOR DRIVE --------
// Input: signed PWM (-255 .. 255). Sign sets direction.

void driveMotor(int pwm) {

  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0) {
    digitalWrite(motorIN1, HIGH);
    digitalWrite(motorIN2, LOW);
  } else if (pwm < 0) {
    digitalWrite(motorIN1, LOW);
    digitalWrite(motorIN2, HIGH);
  } else {
    digitalWrite(motorIN1, LOW);
    digitalWrite(motorIN2, LOW);
  }

  analogWrite(motorPWM, abs(pwm));
  lastPWMSigned = pwm;
}

// -------- COMMANDS --------

void startPositionMode(float angleDeg) {

  targetAngleDeg = angleDeg;
  mode = MODE_POSITION;

  resetPID(posPID, actualAngleDeg);
  previousEncoderCount = readEncoder();
  lastPIDTime = millis();

  Serial.print(F("-> POSITION mode, target = "));
  Serial.print(targetAngleDeg);
  Serial.println(F(" deg"));
}

void startVelocityMode(float rpm) {

  targetRPM = constrain(rpm, -MAX_RPM, MAX_RPM);
  mode = MODE_VELOCITY;

  resetPID(velPID, actualRPM);
  previousEncoderCount = readEncoder();
  lastPIDTime = millis();

  Serial.print(F("-> VELOCITY mode, target = "));
  Serial.print(targetRPM);
  Serial.println(F(" RPM"));
}

void processCommand(char *line) {

  // Skip leading spaces
  while (*line == ' ') line++;

  char cmd = toupper(line[0]);

  if (cmd == 'P') {

    startPositionMode(atof(line + 1));

  } else if (cmd == 'V') {

    startVelocityMode(atof(line + 1));

  } else if (cmd != '\0') {

    Serial.println(F("Unknown command. Use: P <deg>  or  V <rpm>"));
  }
}

void readSerial() {

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n' || c == '\r') {

      if (inputLen > 0) {
        inputBuffer[inputLen] = '\0';
        processCommand(inputBuffer);
        inputLen = 0;
      }

    } else if (inputLen < sizeof(inputBuffer) - 1) {

      inputBuffer[inputLen++] = c;
    }
  }
}

// -------- SETUP --------

void setup() {

  Serial.begin(115200);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  pinMode(motorPWM, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);

  attachInterrupt(
    digitalPinToInterrupt(encoderA),
    encoderISR,
    RISING
  );

  driveMotor(0);

  lastPIDTime = millis();
  lastPrintTime = millis();

  Serial.println(F("Ready. Enter  P <deg>  for position or  V <rpm>  for velocity."));
}

// -------- MAIN LOOP --------

void loop() {

  readSerial();

  unsigned long now = millis();

  // ---------- CONTROL LOOP ----------

  unsigned long interval =
    (mode == MODE_VELOCITY) ? VEL_INTERVAL : POS_INTERVAL;

  if (now - lastPIDTime >= interval) {

    float dt = (now - lastPIDTime) / 1000.0;
    lastPIDTime = now;

    long currentCount = readEncoder();
    long deltaCount = currentCount - previousEncoderCount;
    previousEncoderCount = currentCount;

    // Position (deg) and signed velocity (RPM) from the encoder
    actualAngleDeg = currentCount * 360.0 / ENCODER_CPR;
    actualRPM = deltaCount * 60.0 / (ENCODER_CPR * dt);

    if (mode == MODE_POSITION) {

      float error = targetAngleDeg - actualAngleDeg;

      if (abs(error) <= POSITION_DEADBAND_DEG) {

        // Target reached: stop and clear integral to avoid hunting
        posPID.integral = 0;
        posPID.lastMeasurement = actualAngleDeg;
        driveMotor(0);

      } else {

        float output = computePID(posPID, targetAngleDeg, actualAngleDeg, dt);

        output = constrain(output, -MAX_PWM, MAX_PWM);

        // Add minimum PWM to overcome static friction
        if (output > 0)      output = max(output,  (float)MIN_PWM_POS);
        else if (output < 0) output = min(output, -(float)MIN_PWM_POS);

        driveMotor((int)output);
      }

    } else if (mode == MODE_VELOCITY) {

      if (targetRPM == 0) {

        // V 0 = stop
        velPID.integral = 0;
        driveMotor(0);

      } else {

        float output = computePID(velPID, targetRPM, actualRPM, dt);
        driveMotor((int)constrain(output, -MAX_PWM, MAX_PWM));
      }

    } else {

      // MODE_IDLE (before the first command)
      driveMotor(0);
    }
  }

  // ---------- MONITORING ----------

  if (now - lastPrintTime >= PRINT_INTERVAL) {

    lastPrintTime = now;

    if (mode == MODE_POSITION) {

      Serial.print(F("Mode: POS | Target: "));
      Serial.print(targetAngleDeg);
      Serial.print(F(" deg | Actual: "));
      Serial.print(actualAngleDeg);
      Serial.print(F(" deg | Error: "));
      Serial.print(targetAngleDeg - actualAngleDeg);

    } else if (mode == MODE_VELOCITY) {

      Serial.print(F("Mode: VEL | Target: "));
      Serial.print(targetRPM);
      Serial.print(F(" RPM | Actual: "));
      Serial.print(actualRPM);
      Serial.print(F(" RPM | Angle: "));
      Serial.print(actualAngleDeg);

    } else {

      Serial.print(F("Mode: IDLE | Angle: "));
      Serial.print(actualAngleDeg);
    }

    Serial.print(F(" | PWM: "));
    Serial.println(lastPWMSigned);
  }
}
