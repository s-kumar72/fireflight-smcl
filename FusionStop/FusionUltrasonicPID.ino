/*
  FusionUltrasonicPID.ino
  Ultrasonic-driven motor speed control (PID + encoder)  +  ToF/ultrasonic scene classification

  - MOTOR: commanded ONLY by the ultrasonic distance (same behaviour as the ultrasonic sketch):
           the closer the object, the lower the desired RPM; a PID loop on the encoder tracks it.
  - TOF  : VL53L1X is an additional sensor used ONLY to classify what is in front
           (logic from FusionStop):

      Ultrasonic | ToF            | Scene
      -----------|----------------|------------------------------------------------
      far        | far            | CLEAR
      near       | near           | SOLID
      near       | far / no echo  | TRANSPARENT (light passed through, sound bounced)
      far        | near           | SMOKE       (light scattered, sound passed through)

  Hardware (Arduino Uno):
    Sonar (Pololu 723 / MaxBotix EZ0)  AN -> A1, +5 -> 5V, GND -> GND
    ToF (SparkFun VL53L1X)             SDA -> A4, SCL -> A5, 3V3 -> 3.3V, GND -> GND
    Encoder                            A -> D2 (interrupt), B -> D3, power + GND
    Motor driver (L298)                ENA/PWM -> D6, IN1 -> D7, IN2 -> D8, driver GND -> Arduino GND
    Motor supply                       12 V bench supply into the driver's +/-

  Library: "SparkFun VL53L1X 4m Laser Distance Sensor" (Library Manager)
  Serial Monitor: 115200 baud
*/

#include <Wire.h>
#include <SparkFun_VL53L1X.h>

// ======================= PINS (unchanged from ultrasonic sketch) =======================
const int sonarPin = A1;

const int encoderA = 2;
const int encoderB = 3;

const int motorPWM = 6;
const int motorIN1 = 7;
const int motorIN2 = 8;

// ======================= DISTANCE / SPEED SETTINGS (motor control) =======================
const float START_DISTANCE = 1000.0; // mm - full speed at or beyond this
const float MIN_DISTANCE   = 150.0;  // mm - minimum speed at or below this

const float MAX_RPM = 60.0;
const float MIN_RPM = 5.0;

const float ENCODER_CPR = 420.0;     // counts per rev with the selected interrupt edge

// ======================= PID =======================
float Kp = 10.0;
float Ki = 0.5;
float Kd = 0.001;

float integral = 0;
float previousError = 0;

float desiredRPM = MAX_RPM;
float actualRPM  = 0;

// ======================= ULTRASONIC FILTER =======================
float filteredDistance = 0;
const float alpha = 0.3;

// ======================= TOF / CLASSIFICATION SETTINGS =======================
const int   NEAR_MM    = 300;    // closer than this -> "near" for classification
const int   RELEASE_MM = 400;    // once SOLID/TRANSPARENT, must clear past this (hysteresis)
const int   CONFIRM_N  = 3;      // readings in a row before a new scene is accepted
const float TOF_ALPHA  = 0.4;    // ToF EMA filter weight

enum Scene { SCENE_CLEAR = 0, SCENE_SOLID = 1, SCENE_TRANSPARENT = 2, SCENE_SMOKE = 3 };

SFEVL53L1X tof;
bool  tofOk     = false;
bool  tofValid  = false;
int   tofStatus = -1;   // 0 = good; 1 sigma fail; 2 signal fail; 4 out of bounds; 7 wrap-around
int   tofRawMm  = -1;
float tofFiltMm = -1;

Scene candidate  = SCENE_CLEAR;
Scene confirmed  = SCENE_CLEAR;
int   streak     = 0;
bool  nearLatched = false;  // true while a SOLID/TRANSPARENT object is held (for hysteresis)

// ======================= TIMING =======================
unsigned long lastSensorTime = 0;
unsigned long lastPIDTime    = 0;

const unsigned long SENSOR_INTERVAL = 50;   // ms
const unsigned long PID_INTERVAL    = 100;  // ms

// ======================= ENCODER =======================
volatile long encoderCount = 0;
long previousEncoderCount  = 0;

void encoderISR() {
  if (digitalRead(encoderB))
    encoderCount++;
  else
    encoderCount--;
}

// ======================= ULTRASONIC =======================
// Input: sonar pin reading. Output: distance in mm (12.7 mm per ADC count at 5 V).
float readDistance() {
  int adc = analogRead(sonarPin);
  return adc * 12.7;
}

// ======================= TOF =======================
bool tofBegin() {
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);      // don't freeze forever if I2C wiring is wrong
#endif
  if (tof.begin() != 0) return false;   // 0 = success
  tof.setDistanceModeLong();            // up to ~4 m
  tof.setTimingBudgetInMs(50);
  tof.setIntermeasurementPeriod(50);
  tof.startRanging();
  return true;
}

// Non-blocking: returns immediately if no new measurement is ready
void tofUpdate() {
  if (!tofOk || !tof.checkForDataReady()) return;
  tofRawMm  = tof.getDistance();
  tofStatus = tof.getRangeStatus();
  tofValid  = (tofStatus == 0);
  tof.clearInterrupt();
  if (tofValid) {
    tofFiltMm = (tofFiltMm < 0) ? tofRawMm : TOF_ALPHA * tofRawMm + (1.0 - TOF_ALPHA) * tofFiltMm;
  }
}

// ======================= CLASSIFICATION (does NOT command the motor) =======================
Scene classify(int usMm, int tofMm, bool valid, int threshMm) {
  bool usNear  = usMm < threshMm;
  bool tofNear = valid && tofMm >= 0 && tofMm < threshMm;

  if (usNear && tofNear)  return SCENE_SOLID;        // both see it
  if (usNear && !tofNear) return SCENE_TRANSPARENT;  // sound bounced, light passed through
  if (!usNear && tofNear) return SCENE_SMOKE;        // light scattered, sound passed through
  return SCENE_CLEAR;
}

void classificationUpdate(int usMm, int tofMm, bool valid) {
  // larger threshold while an object is latched, so the label doesn't flicker at the edge
  int thresh = nearLatched ? RELEASE_MM : NEAR_MM;
  Scene s = classify(usMm, tofMm, valid, thresh);

  // temporal consistency: accept a new scene only after CONFIRM_N readings in a row
  if (s == candidate) { if (streak < CONFIRM_N) streak++; }
  else                { candidate = s; streak = 1; }
  if (streak >= CONFIRM_N) confirmed = candidate;

  // smoke is not a physical obstacle
  nearLatched = (confirmed == SCENE_SOLID || confirmed == SCENE_TRANSPARENT);
}

const char* sceneName(Scene s) {
  switch (s) {
    case SCENE_SOLID:       return "SOLID";
    case SCENE_TRANSPARENT: return "TRANSPARENT";
    case SCENE_SMOKE:       return "SMOKE";
    default:                return "CLEAR";
  }
}

// ======================= SPEED REFERENCE (ultrasonic only) =======================
float calculateDesiredRPM(float distance) {
  if (distance >= START_DISTANCE) return MAX_RPM;
  if (distance <= MIN_DISTANCE)   return MIN_RPM;

  float ratio = (distance - MIN_DISTANCE) / (START_DISTANCE - MIN_DISTANCE);
  return MIN_RPM + ratio * (MAX_RPM - MIN_RPM);
}

// ======================= MOTOR =======================
void driveMotor(int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(motorIN1, HIGH);
  digitalWrite(motorIN2, LOW);
  analogWrite(motorPWM, pwm);
}

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  Serial.println("BOOT,FusionUltrasonicPID starting");

  pinMode(sonarPin, INPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  pinMode(motorPWM, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  analogWrite(motorPWM, 0);

  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, RISING);

  // MaxSonar calibrates itself in the first ~250 ms after power-up; keep the area in front clear
  delay(500);
  filteredDistance = readDistance();

  Serial.println("BOOT,starting ToF...");
  tofOk = tofBegin();
  Serial.println(tofOk ? "BOOT,ToF OK" : "ERR,VL53L1X not found - check SDA/SCL/3.3V/GND");

  lastSensorTime = millis();
  lastPIDTime    = millis();
  Serial.println("BOOT,running");
}

// ======================= MAIN LOOP =======================
void loop() {
  unsigned long now = millis();

  // 0. TOF update (non-blocking, polled every loop)
  tofUpdate();

  // 1. ULTRASONIC UPDATE + SPEED REFERENCE + CLASSIFICATION  (every 50 ms)
  if (now - lastSensorTime >= SENSOR_INTERVAL) {
    lastSensorTime = now;

    float distance = readDistance();
    filteredDistance = alpha * distance + (1.0 - alpha) * filteredDistance;

    // Motor speed reference comes from the ultrasonic distance only
    desiredRPM = calculateDesiredRPM(filteredDistance);

    // ToF + ultrasonic -> scene label (informational only)
    classificationUpdate((int)filteredDistance, (int)tofFiltMm, tofValid);
  }

  // 2. ENCODER SPEED MEASUREMENT + PID  (every 100 ms)
  if (now - lastPIDTime >= PID_INTERVAL) {
    float elapsed = (now - lastPIDTime) / 1000.0;
    lastPIDTime = now;

    long currentCount;
    noInterrupts();
    currentCount = encoderCount;
    interrupts();

    long deltaCount = currentCount - previousEncoderCount;
    previousEncoderCount = currentCount;

    actualRPM = abs(deltaCount) * 60.0 / (ENCODER_CPR * elapsed);

    float error = desiredRPM - actualRPM;
    integral += error * elapsed;
    integral = constrain(integral, -200, 200);
    float derivative = (error - previousError) / elapsed;

    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    static float pwm = 0;
    pwm = constrain(output, 0, 255);
    driveMotor((int)pwm);

    // 3. MONITORING (ultrasonic/motor output identical to the ultrasonic sketch,
    //    ToF + scene fields appended at the end of the same line)

    Serial.print("Distance (Ultrasonic Sensor): ");
    Serial.print(filteredDistance);

    Serial.print(" mm | Desired RPM: ");
    Serial.print(desiredRPM);

    Serial.print(" | Actual RPM: ");
    Serial.print(actualRPM);

    Serial.print(" | PWM: ");
    Serial.print(pwm);

    Serial.print(" | TOF Distance: ");
    Serial.print(tofRawMm);

    Serial.print(" mm (status ");
    Serial.print(tofStatus);

    Serial.print(") | Scene: ");
    Serial.println(sceneName(confirmed));
  }
}
