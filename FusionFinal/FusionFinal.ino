/*
  FusionFinal.ino — Team G, Sensors & Motor Control Lab
  Ultrasonic-driven DC motor speed control (PID + encoder)
  + ToF/ultrasonic scene classification (Clear / Solid / Transparent / Smoke)
  + plot-ready output (raw vs filtered, live error against a known distance)

  Scene idea (Hopkins et al., arXiv:2510.06518 — glass only; smoke is our extension):
      Ultrasonic | ToF            | Scene
      -----------|----------------|-------------
      far        | far            | CLEAR
      near       | near           | SOLID
      near       | far / no echo  | TRANSPARENT  (light passed through, sound bounced)
      far        | near           | SMOKE        (light scattered, sound passed through)

  Hardware (Arduino Uno):
    Sonar  Pololu 723 / MaxBotix LV-MaxSonar-EZ0   AN -> A1, +5 -> 5V, GND
    ToF    SparkFun VL53L1X                        SDA -> A4, SCL -> A5, 3V3 -> 3.3V, GND
    Encoder (Cytron SPG30E-60K)                    A -> D2, B -> D3, VCC -> 5V, GND
    L298N  ENA -> D6 (remove ENA jumper), IN1 -> D7, IN2 -> D8, GND -> Arduino GND
    Motor supply: 12 V bench supply into L298N +12V / GND

  Library: "SparkFun VL53L1X 4m Laser Distance Sensor"
  Serial: 115200 baud, line ending "Newline"

  OUTPUT MODES (type the letter + Enter):
    p  = PLOT  -> open Tools > Serial Plotter: us_raw, us_filt, tof_raw, tof_filt
                  (+ us_err, tof_err when a true distance is set)
    c  = CSV   -> one CSV row per cycle; log it with plot_sensors.py for report figures
    h  = TEXT  -> human-readable line (default)
  OTHER COMMANDS:
    t<cm>  set the true target distance, e.g. t50  (enables live error output)
    t0     clear the true distance
*/

#include <Wire.h>
#include <SparkFun_VL53L1X.h>

// ======================= PINS =======================
const int sonarPin = A1;
const int encoderA = 2;
const int encoderB = 3;
const int motorPWM = 6;
const int motorIN1 = 7;
const int motorIN2 = 8;

// ======================= CALIBRATION =======================
// Paste the constants from UltrasonicCal / TofCal here.
// Ultrasonic: distance_mm = US_A * ADC + US_B   (datasheet: 12.7 mm per count, 0 offset)
const float US_A  = 12.7;
const float US_B  = 0.0;
// ToF: corrected_mm = TOF_A * raw_mm + TOF_B      (1, 0 = no correction)
const float TOF_A = 1.0;
const float TOF_B = 0.0;

// ======================= FILTERS =======================
const float US_ALPHA  = 0.3;    // EMA weight for ultrasonic (lower = smoother)
const float TOF_ALPHA = 0.4;    // EMA weight for ToF

// ======================= MOTOR SPEED REFERENCE =======================
const float START_DISTANCE = 1000.0; // mm: full speed at or beyond this
const float MIN_DISTANCE   = 150.0;  // mm: minimum speed at or below this
const float MAX_RPM = 60.0;
const float MIN_RPM = 5.0;
const float ENCODER_CPR = 420.0;     // counts per output-shaft revolution (measured)

// ======================= PID =======================
float Kp = 10.0, Ki = 0.5, Kd = 0.001;
float integral = 0, previousError = 0;
float desiredRPM = MAX_RPM, actualRPM = 0, pwmOut = 0;

// ======================= CLASSIFICATION =======================
const int NEAR_MM    = 300;   // closer than this = "near"
const int RELEASE_MM = 400;   // hysteresis: must clear past this once latched
const int CONFIRM_N  = 3;     // consecutive readings to accept a new scene

enum Scene { SCENE_CLEAR = 0, SCENE_SOLID = 1, SCENE_TRANSPARENT = 2, SCENE_SMOKE = 3 };
Scene candidate = SCENE_CLEAR, confirmed = SCENE_CLEAR;
int   streak = 0;
bool  nearLatched = false;

// ======================= SENSOR STATE =======================
SFEVL53L1X tof;
bool  tofOk = false, tofValid = false;
int   tofStatus = -1;
float tofRawMm = -1, tofFiltMm = -1;

int   usAdc = 0;
float usRawMm = 0, usFiltMm = 0;

float trueMm = 0;             // 0 = no ground truth set

// ======================= OUTPUT =======================
enum OutMode { OUT_TEXT, OUT_PLOT, OUT_CSV };
OutMode outMode = OUT_TEXT;

// ======================= TIMING =======================
unsigned long lastSensorTime = 0, lastPIDTime = 0, lastPrintTime = 0;
const unsigned long SENSOR_INTERVAL = 50;   // ms (sonar refresh ~49 ms)
const unsigned long PID_INTERVAL    = 100;  // ms
const unsigned long PRINT_INTERVAL  = 50;   // ms

// ======================= ENCODER =======================
volatile long encoderCount = 0;
long previousEncoderCount = 0;

void encoderISR() {
  if (digitalRead(encoderB)) encoderCount++;
  else                       encoderCount--;
}

// ======================= ULTRASONIC =======================
void usUpdate() {
  usAdc   = analogRead(sonarPin);
  usRawMm = US_A * usAdc + US_B;
  usFiltMm = US_ALPHA * usRawMm + (1.0 - US_ALPHA) * usFiltMm;
}

// ======================= TOF =======================
bool tofBegin() {
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  if (tof.begin() != 0) return false;
  tof.setDistanceModeLong();
  tof.setTimingBudgetInMs(50);
  tof.setIntermeasurementPeriod(50);
  tof.startRanging();
  return true;
}

void tofUpdate() {
  if (!tofOk || !tof.checkForDataReady()) return;
  int d     = tof.getDistance();
  tofStatus = tof.getRangeStatus();
  tofValid  = (tofStatus == 0);
  tof.clearInterrupt();
  if (tofValid) {
    tofRawMm  = TOF_A * d + TOF_B;
    tofFiltMm = (tofFiltMm < 0) ? tofRawMm : TOF_ALPHA * tofRawMm + (1.0 - TOF_ALPHA) * tofFiltMm;
  }
}

// ======================= CLASSIFICATION =======================
Scene classify(int usMm, int tofMm, bool valid, int threshMm) {
  bool usNear  = usMm < threshMm;
  bool tofNear = valid && tofMm >= 0 && tofMm < threshMm;
  if (usNear && tofNear)  return SCENE_SOLID;
  if (usNear && !tofNear) return SCENE_TRANSPARENT;
  if (!usNear && tofNear) return SCENE_SMOKE;
  return SCENE_CLEAR;
}

void classificationUpdate() {
  int thresh = nearLatched ? RELEASE_MM : NEAR_MM;
  Scene s = classify((int)usFiltMm, (int)tofFiltMm, tofValid, thresh);
  if (s == candidate) { if (streak < CONFIRM_N) streak++; }
  else                { candidate = s; streak = 1; }
  if (streak >= CONFIRM_N) confirmed = candidate;
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

// ======================= MOTOR =======================
float calculateDesiredRPM(float distance) {
  if (distance >= START_DISTANCE) return MAX_RPM;
  if (distance <= MIN_DISTANCE)   return MIN_RPM;
  float ratio = (distance - MIN_DISTANCE) / (START_DISTANCE - MIN_DISTANCE);
  return MIN_RPM + ratio * (MAX_RPM - MIN_RPM);
}

void driveMotor(int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(motorIN1, HIGH);
  digitalWrite(motorIN2, LOW);
  analogWrite(motorPWM, pwm);
}

void pidUpdate(float elapsed) {
  long currentCount;
  noInterrupts();
  currentCount = encoderCount;
  interrupts();
  long deltaCount = currentCount - previousEncoderCount;
  previousEncoderCount = currentCount;
  actualRPM = abs(deltaCount) * 60.0 / (ENCODER_CPR * elapsed);

  float error = desiredRPM - actualRPM;
  integral += error * elapsed;
  integral = constrain(integral, -200.0, 200.0);
  float derivative = (error - previousError) / elapsed;
  previousError = error;

  pwmOut = constrain(Kp * error + Ki * integral + Kd * derivative, 0.0, 255.0);
  driveMotor((int)pwmOut);
}

// ======================= OUTPUT =======================
void printCsvHeader() {
  Serial.println("ms,us_adc,us_raw_mm,us_filt_mm,tof_raw_mm,tof_filt_mm,tof_status,scene,"
                 "desired_rpm,actual_rpm,pwm,true_mm");
}

void printOutput() {
  bool hasTruth = trueMm > 0;

  if (outMode == OUT_PLOT) {
    // Arduino IDE Serial Plotter format: label:value pairs
    Serial.print("us_raw:");    Serial.print(usRawMm);
    Serial.print(",us_filt:");  Serial.print(usFiltMm);
    Serial.print(",tof_raw:");  Serial.print(tofRawMm);
    Serial.print(",tof_filt:"); Serial.print(tofFiltMm);
    if (hasTruth) {
      Serial.print(",true:");    Serial.print(trueMm);
      Serial.print(",us_err:");  Serial.print(usFiltMm - trueMm);
      Serial.print(",tof_err:"); Serial.print(tofValid ? tofFiltMm - trueMm : 0);
    }
    Serial.println();
  }
  else if (outMode == OUT_CSV) {
    Serial.print(millis());          Serial.print(",");
    Serial.print(usAdc);             Serial.print(",");
    Serial.print(usRawMm, 1);        Serial.print(",");
    Serial.print(usFiltMm, 1);       Serial.print(",");
    Serial.print(tofRawMm, 1);       Serial.print(",");
    Serial.print(tofFiltMm, 1);      Serial.print(",");
    Serial.print(tofStatus);         Serial.print(",");
    Serial.print(sceneName(confirmed)); Serial.print(",");
    Serial.print(desiredRPM, 2);     Serial.print(",");
    Serial.print(actualRPM, 2);      Serial.print(",");
    Serial.print(pwmOut, 0);         Serial.print(",");
    Serial.println(trueMm, 0);
  }
  else {
    Serial.print("US raw/filt: ");   Serial.print(usRawMm, 0); Serial.print("/"); Serial.print(usFiltMm, 0);
    Serial.print(" mm | ToF raw/filt: "); Serial.print(tofRawMm, 0); Serial.print("/"); Serial.print(tofFiltMm, 0);
    Serial.print(" mm (st "); Serial.print(tofStatus);
    Serial.print(") | Scene: "); Serial.print(sceneName(confirmed));
    Serial.print(" | RPM des/act: "); Serial.print(desiredRPM, 1); Serial.print("/"); Serial.print(actualRPM, 1);
    Serial.print(" | PWM: "); Serial.print(pwmOut, 0);
    if (hasTruth) {
      Serial.print(" | err US/ToF: "); Serial.print(usFiltMm - trueMm, 0);
      Serial.print("/"); Serial.print(tofValid ? tofFiltMm - trueMm : 0, 0);
    }
    Serial.println();
  }
}

// ======================= COMMANDS =======================
void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;
  char c = s.charAt(0);
  if (c == 'p') outMode = OUT_PLOT;
  else if (c == 'h') outMode = OUT_TEXT;
  else if (c == 'c') { outMode = OUT_CSV; printCsvHeader(); }
  else if (c == 't') trueMm = s.substring(1).toFloat() * 10.0;   // cm -> mm
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  Serial.println("BOOT,FusionFinal starting");

  pinMode(sonarPin, INPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  driveMotor(0);
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, RISING);

  delay(500);                           // sonar power-up self-calibration: keep area clear
  usAdc = analogRead(sonarPin);
  usRawMm = usFiltMm = US_A * usAdc + US_B;

  Serial.println("BOOT,starting ToF...");
  tofOk = tofBegin();
  Serial.println(tofOk ? "BOOT,ToF OK" : "ERR,VL53L1X not found - check SDA/SCL/3.3V/GND");
  Serial.println("BOOT,commands: p=plot c=csv h=text t<cm>=true distance");

  lastSensorTime = lastPIDTime = lastPrintTime = millis();
}

void loop() {
  unsigned long now = millis();

  handleSerial();
  tofUpdate();                                    // non-blocking

  if (now - lastSensorTime >= SENSOR_INTERVAL) {  // 20 Hz
    lastSensorTime = now;
    usUpdate();
    desiredRPM = calculateDesiredRPM(usFiltMm);   // motor reference: ultrasonic only
    classificationUpdate();                       // scene label: ultrasonic + ToF
  }

  if (now - lastPIDTime >= PID_INTERVAL) {        // 10 Hz
    float elapsed = (now - lastPIDTime) / 1000.0;
    lastPIDTime = now;
    pidUpdate(elapsed);
  }

  if (now - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = now;
    printOutput();
  }
}
