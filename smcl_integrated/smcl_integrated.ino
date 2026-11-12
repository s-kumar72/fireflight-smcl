/*
  Standalone Arduino Uno sketch. Open this file in its own sketch folder;
  do not add it as a tab to smcl (which has its own setup/loop/controllers).
  Libraries: Servo, SparkFun VL53L1X 4m Laser Distance Sensor.

  D13 -> push-button -> GND, INPUT_PULLUP. A debounced falling edge toggles
  SENSOR_CONTROL (initial mode) / UI_CONTROL. No delays in the main loop.
  Sensor mode: pot -> stepper, thermistor -> servo, ultrasonic -> DC RPM.
  UI mode (115200 baud, newline terminated):
    SER:0..180             absolute servo angle in degrees
    STP:+:0..32767         relative stepper move in degrees (also STP:-:...)
    DCM:P:+:0..32767       absolute DC shaft angle from boot (also -)
    DCM:V:+:0..60          signed DC shaft RPM (also -); V with 0 stops
  Motor commands are ignored in sensor mode. Switching modes cancels pending
  stepper travel and stops the DC motor; the servo holds until its next target.
  Sensor requests work in BOTH modes: REQ:POT/THERM/ULTR/SLOT/TOF/DCE.
  SLOT reports integer RPM, 0..4095, using a local three-blade counter.
  DCE is encoder shaft angle wrapped to 0..4095 per revolution; TOF is -1
  when unavailable/invalid. Only protocol replies are written to Serial.

  Wiring retained from smcl: A0 pot, A1 sonar, A3 thermistor, A4/A5 I2C,
  D2/D3 encoder, D6 PWM, D7/D8 bridge direction, D9 servo,
  D10/D11 stepper step/direction, D12 slot sensor.
*/
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFun_VL53L1X.h>
#include <math.h>
#include <string.h>

// Local slot RPM counter, based on the original smcl implementation.
int slotReading();
#define SLOT_SENSOR_PIN 12

const unsigned long SLOT_WINDOW_MS = 1000;

int slotPrevState = 0;                
unsigned long slotCount = 0;
unsigned long slotWindowStartMs = 0;  
int slotRPM = 0;       
const int SLOT_BLADES = 3; 

void slotSetup() {
  pinMode(SLOT_SENSOR_PIN, INPUT_PULLUP);
  slotPrevState = slotReading();  // avoid a false count on the first loop
  slotCount = 0;
  slotRPM = 0;
  slotWindowStartMs = millis();
}

int slotReading() {
  if (digitalRead(SLOT_SENSOR_PIN) == HIGH) {
    return 1; // blocked means 1 (detected a reading)
  } else {
    return 0; // open means that its not reading anything 0
  }
}

// call every loop() 
void slotUpdate() {
  int current = slotReading();

  if (slotPrevState == 0 && current == 1) {
    slotCount++;
  }
  slotPrevState = current;

  unsigned long now = millis();
  unsigned long elapsed = now - slotWindowStartMs;
  if (elapsed >= SLOT_WINDOW_MS) {
    // Three rising edges per revolution; account for the actual window length.
    float rpm = slotCount * 60000.0f / (SLOT_BLADES * (float)elapsed);
    slotRPM = (int)lround(constrain(rpm, 0.0f, 4095.0f));
    slotCount = 0;
    slotWindowStartMs = now;
  }
}

int slotHandleRequest() {
  return slotRPM;
}

const uint8_t BUTTON_PIN = 13;
const unsigned long DEBOUNCE_MS = 30;
enum ControlMode { SENSOR_CONTROL, UI_CONTROL };
ControlMode controlMode = SENSOR_CONTROL;
int buttonRaw = HIGH;
int buttonStable = HIGH;
unsigned long buttonChangedMs = 0;

const uint8_t POT_PIN = A0, SONAR_PIN = A1, THERM_PIN = A3;
const uint8_t ENCODER_A = 2, ENCODER_B = 3;
const uint8_t MOTOR_PWM = 6, MOTOR_IN1 = 7, MOTOR_IN2 = 8;
const uint8_t SERVO_PIN = 9, STEP_PIN = 10, DIR_PIN = 11;
const float ENCODER_CPR = 420.0f;
const float DEG_PER_STEP = 360.0f / (200L * 8);
const unsigned long STEP_INTERVAL_US = 1000;

Servo servo;
SFEVL53L1X tof;
bool tofOk = false, tofValid = false;
int tofRawMm = -1;
float tofFilteredMm = -1;
int potRaw = 0, thermRaw = 0;
float sonarFilteredMm = 0;
float servoDeg = 90;
unsigned long lastSensorMs = 0, lastServoMs = 0, lastStepUs = 0;
long stepperPos = 0, stepperTarget = 0;
volatile long encoderCount = 0;
long previousEncoderCount = 0;
float actualAngleDeg = 0, actualRPM = 0;

// Same informational scene classification as FusionUltrasonicPID.
enum Scene { SCENE_CLEAR, SCENE_SOLID, SCENE_TRANSPARENT, SCENE_SMOKE };
Scene candidate = SCENE_CLEAR, confirmed = SCENE_CLEAR;
uint8_t streak = 0;
bool nearLatched = false;

enum DcMode { DC_IDLE, DC_POSITION, DC_VELOCITY };
DcMode dcMode = DC_VELOCITY;
float targetAngleDeg = 0, targetRPM = 0;
struct PID {
  float kp, ki, kd, integral, lastMeasurement, integralLimit;
};
// Keep Arduino's generated prototypes from referring to PID before its definition.
float computePID(PID &pid, float target, float measurement, float dt);
PID posPID = {3.0f, 0.10f, 0.05f, 0, 0, 100};
PID velPID = {10.0f, 0.50f, 0.001f, 0, 0, 200};
unsigned long lastPIDMs = 0;

const uint8_t CMD_MAX = 32;
char cmdBuf[CMD_MAX];
uint8_t cmdLen = 0;
bool cmdOverflow = false;

void encoderISR() {
  encoderCount += digitalRead(ENCODER_B) ? 1 : -1;
}

long readEncoder() {
  noInterrupts();
  long count = encoderCount;
  interrupts();
  return count;
}

void driveMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(MOTOR_IN1, pwm > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_IN2, pwm < 0 ? HIGH : LOW);
  analogWrite(MOTOR_PWM, abs(pwm));
}

void resetControllers() {
  long count = readEncoder();
  actualAngleDeg = count * 360.0f / ENCODER_CPR;
  previousEncoderCount = count;
  posPID.integral = velPID.integral = 0;
  posPID.lastMeasurement = actualAngleDeg;
  velPID.lastMeasurement = actualRPM;
  lastPIDMs = millis();
}

float computePID(PID &pid, float target, float measurement, float dt) {
  pid.integral += (target - measurement) * dt;
  pid.integral = constrain(pid.integral, -pid.integralLimit, pid.integralLimit);
  float derivative = -(measurement - pid.lastMeasurement) / dt;
  pid.lastMeasurement = measurement;
  return pid.kp * (target - measurement) + pid.ki * pid.integral + pid.kd * derivative;
}

void buttonUpdate() {
  unsigned long now = millis();
  int reading = digitalRead(BUTTON_PIN);
  if (reading != buttonRaw) {
    buttonRaw = reading;
    buttonChangedMs = now;
  }
  if (buttonRaw == buttonStable || now - buttonChangedMs < DEBOUNCE_MS) return;
  int previous = buttonStable;
  buttonStable = buttonRaw;
  if (previous != HIGH || buttonStable != LOW) return;

  controlMode = controlMode == SENSOR_CONTROL ? UI_CONTROL : SENSOR_CONTROL;
  stepperTarget = stepperPos;
  driveMotor(0);
  targetRPM = 0;
  dcMode = controlMode == UI_CONTROL ? DC_IDLE : DC_VELOCITY;
  resetControllers();
}

void servoSetDeg(float degrees) {
  servoDeg = constrain(degrees, 0.0f, 180.0f);
  servo.writeMicroseconds(1000 + (int)(servoDeg * 1000.0f / 180.0f));
}

float thermCelsius(int adc) {
  adc = constrain(adc, 1, 1022);
  float resistance = 10000.0f * adc / (1023.0f - adc);
  return 1.0f / (1.0f / 298.15f + log(resistance / 10000.0f) / 3950.0f) - 273.15f;
}

float distanceRPM(float mm) {
  if (mm >= 1000) return 60;
  if (mm <= 150) return 5;
  return 5 + (mm - 150) * 55.0f / 850.0f;
}

void sensorsUpdate() {
  if (tofOk && tof.checkForDataReady()) {
    tofRawMm = tof.getDistance();
    tofValid = tof.getRangeStatus() == 0;
    tof.clearInterrupt();
    if (tofValid) {
      tofFilteredMm = tofFilteredMm < 0 ? tofRawMm
          : 0.4f * tofRawMm + 0.6f * tofFilteredMm;
    }
  }
  unsigned long now = millis();
  if (now - lastSensorMs < 50) return;
  lastSensorMs = now;
  potRaw = analogRead(POT_PIN);
  long sum = 0;
  for (uint8_t i = 0; i < 16; ++i) sum += analogRead(THERM_PIN);
  thermRaw = sum / 16;
  sonarFilteredMm = 0.3f * analogRead(SONAR_PIN) * 12.7f + 0.7f * sonarFilteredMm;

  int threshold = nearLatched ? 400 : 300;
  bool usNear = sonarFilteredMm < threshold;
  bool tofNear = tofValid && tofFilteredMm >= 0 && tofFilteredMm < threshold;
  Scene scene = usNear ? (tofNear ? SCENE_SOLID : SCENE_TRANSPARENT)
                       : (tofNear ? SCENE_SMOKE : SCENE_CLEAR);
  if (scene != candidate) { candidate = scene; streak = 1; }
  else if (streak < 3) ++streak;
  if (streak >= 3) confirmed = candidate;
  nearLatched = confirmed == SCENE_SOLID || confirmed == SCENE_TRANSPARENT;
}

void sensorControlUpdate() {
  if (controlMode != SENSOR_CONTROL) return;
  stepperTarget = lround(potRaw * (280.0f / 1023.0f) / DEG_PER_STEP);
  targetRPM = distanceRPM(sonarFilteredMm);
  dcMode = DC_VELOCITY;
  if (millis() - lastServoMs >= 100) {
    lastServoMs = millis();
    float target = constrain((thermCelsius(thermRaw) - 20.0f) * 12.0f, 0.0f, 180.0f);
    if (fabs(target - servoDeg) >= 2.0f) servoSetDeg(target);
  }
}

void stepperUpdate() {
  if (stepperPos == stepperTarget || micros() - lastStepUs < STEP_INTERVAL_US) return;
  lastStepUs = micros();
  bool forward = stepperTarget > stepperPos;
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  delayMicroseconds(2); // driver direction setup time
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN, LOW);
  stepperPos += forward ? 1 : -1;
}

void dcUpdate() {
  unsigned long now = millis();
  unsigned long interval = dcMode == DC_POSITION ? 20 : 100;
  if (now - lastPIDMs < interval) return;
  float dt = (now - lastPIDMs) / 1000.0f;
  lastPIDMs = now;
  long count = readEncoder();
  actualAngleDeg = count * 360.0f / ENCODER_CPR;
  actualRPM = (count - previousEncoderCount) * 60.0f / (ENCODER_CPR * dt);
  previousEncoderCount = count;
  float output = 0;
  if (dcMode == DC_POSITION) {
    if (fabs(targetAngleDeg - actualAngleDeg) <= 1.0f) {
      posPID.integral = 0;
      posPID.lastMeasurement = actualAngleDeg;
    } else {
      output = computePID(posPID, targetAngleDeg, actualAngleDeg, dt);
      if (output > 0) output = max(output, 35.0f);
      else if (output < 0) output = min(output, -35.0f);
    }
  } else if (dcMode == DC_VELOCITY && targetRPM != 0) {
    output = computePID(velPID, targetRPM, actualRPM, dt);
    // The original distance controller only drives forward.
    if (controlMode == SENSOR_CONTROL) output = max(output, 0.0f);
  } else {
    velPID.integral = 0;
    velPID.lastMeasurement = actualRPM;
  }
  driveMotor((int)constrain(output, -255.0f, 255.0f));
}

// Strictly parse unsigned decimal payloads; reject partial or out-of-range input.
bool parseValue(const char *arg, unsigned int limit, unsigned int &value) {
  if (*arg < '0' || *arg > '9') return false;
  unsigned long result = 0;
  while (*arg) {
    if (*arg < '0' || *arg > '9') return false;
    result = result * 10 + (*arg++ - '0');
    if (result > limit) return false;
  }
  value = (unsigned int)result;
  return true;
}

void reply(const char *header, long value) {
  Serial.print(header);
  Serial.println(value);
}

void handleRequest(const char *cmd) {
  if (strcmp(cmd, "REQ:POT") == 0) reply("POT:", potRaw);
  else if (strcmp(cmd, "REQ:THERM") == 0) reply("TRM:", thermRaw);
  else if (strcmp(cmd, "REQ:ULTR") == 0) reply("ULT:", lround(sonarFilteredMm));
  else if (strcmp(cmd, "REQ:SLOT") == 0) reply("SLT:", slotHandleRequest());
  else if (strcmp(cmd, "REQ:TOF") == 0) reply("TOF:", tofValid ? tofRawMm : -1);
  else if (strcmp(cmd, "REQ:DCE") == 0) {
    long wrapped = readEncoder() % (long)ENCODER_CPR;
    if (wrapped < 0) wrapped += (long)ENCODER_CPR;
    reply("DCE:", wrapped * 4096L / (long)ENCODER_CPR);
  } else if (controlMode == UI_CONTROL) {
    unsigned int value;
    size_t length = strlen(cmd);
    if (strncmp(cmd, "SER:", 4) == 0 && parseValue(cmd + 4, 180, value)) {
      servoSetDeg(value);
    } else if (length >= 7 && strncmp(cmd, "STP:", 4) == 0 &&
               (cmd[4] == '+' || cmd[4] == '-') && cmd[5] == ':' &&
               parseValue(cmd + 6, 32767, value)) {
      // Matches stepperMoveRelDeg: relative to the current physical position.
      float degrees = cmd[4] == '-' ? -(float)value : (float)value;
      stepperTarget = stepperPos + lround(degrees / DEG_PER_STEP);
    } else if (length >= 9 && strncmp(cmd, "DCM:", 4) == 0 &&
               (cmd[4] == 'P' || cmd[4] == 'V') && cmd[5] == ':' &&
               (cmd[6] == '+' || cmd[6] == '-') && cmd[7] == ':' &&
               parseValue(cmd + 8, cmd[4] == 'V' ? 60 : 32767, value)) {
      float signedValue = cmd[6] == '-' ? -(float)value : (float)value;
      resetControllers();
      if (cmd[4] == 'P') {
        targetAngleDeg = signedValue;
        dcMode = DC_POSITION;
      } else {
        targetRPM = signedValue;
        dcMode = DC_VELOCITY;
      }
      if (dcMode == DC_VELOCITY && targetRPM == 0) driveMotor(0);
    }
  }
}

void serialUpdate() {
  // Bound each pass so sustained serial input cannot starve stepping/button polls.
  for (uint8_t budget = 0; budget < 64 && Serial.available(); ++budget) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (!cmdOverflow && cmdLen > 0) handleRequest(cmdBuf);
      cmdLen = 0;
      cmdOverflow = false;
    } else if (!cmdOverflow) {
      if (cmdLen < CMD_MAX - 1) cmdBuf[cmdLen++] = c;
      else cmdOverflow = true; // discard the entire line, never execute a prefix
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  buttonRaw = buttonStable = digitalRead(BUTTON_PIN);
  buttonChangedMs = millis();
  pinMode(SONAR_PIN, INPUT);
  slotSetup();
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  driveMotor(0);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
  servo.attach(SERVO_PIN, 1000, 2000);
  servoSetDeg(90);

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  tofOk = tof.begin() == 0;
  if (tofOk) {
    tof.setDistanceModeLong();
    tof.setTimingBudgetInMs(50);
    tof.setIntermeasurementPeriod(50);
    tof.startRanging();
  }
  // Sonar's power-on calibration can finish while the loop services serial.
  sonarFilteredMm = analogRead(SONAR_PIN) * 12.7f;
  lastSensorMs = millis() - 50;
  sensorsUpdate();
  resetControllers();
}

void loop() {
  buttonUpdate();
  slotUpdate();
  serialUpdate();
  sensorsUpdate();
  sensorControlUpdate();
  stepperUpdate();
  dcUpdate();
}
