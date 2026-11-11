// FusionStop.ino — Ultrasonic + ToF fusion -> detect solid / transparent / smoke -> DC motor start/stop
// Team G, Sensors & Motor Control Lab
//
// Idea (Hopkins et al., arXiv:2510.06518): sound reflects off glass, light mostly passes through.
// When the two sensors DISAGREE, that tells us what's in front:
//
//   Ultrasonic | ToF          | Scene         | Motor
//   -----------|--------------|---------------|------
//   far        | far          | CLEAR         | RUN
//   near       | near         | SOLID         | STOP
//   near       | far/no echo  | TRANSPARENT   | STOP  (light passed through, sound bounced)
//   far        | near         | SMOKE         | RUN   (light scattered, sound passed through)
//
// Library: Library Manager -> "SparkFun VL53L1X 4m Laser Distance Sensor"
// Wiring:
//   VL53L1X  SDA->A4, SCL->A5, 3.3V, GND
//   Ultrasonic (Pololu 723) AN->A1, +5V, GND
//   L298     ENA->5, IN1->7, IN2->8, motor supply from bench PSU, common GND
// Mount both sensors side by side, facing the same direction.

#include <Wire.h>
#include <SparkFun_VL53L1X.h>

// ======================= PINS =======================
#define PIN_US_AN   A1
#define PIN_DC_ENA  5
#define PIN_DC_IN1  7
#define PIN_DC_IN2  8

// ======================= TUNABLES =======================
const int   RUN_PWM    = 180;   // forward speed when running (0..255)
const int   STOP_MM    = 300;   // obstacle closer than this -> stop
const int   RESTART_MM = 400;   // must clear past this to run again (hysteresis)
const int   CONFIRM_N  = 3;     // readings in a row before a new scene is accepted
const float US_ALPHA   = 0.3;   // ultrasonic EMA filter weight (lower = smoother)
const float TOF_ALPHA  = 0.4;   // ToF EMA filter weight

// ======================= STATE =======================
enum Scene { SCENE_CLEAR = 0, SCENE_SOLID = 1, SCENE_TRANSPARENT = 2, SCENE_SMOKE = 3 };

SFEVL53L1X tof;
bool  tofOk     = false;
bool  tofValid  = false;
int   tofRawMm  = -1;
float tofFiltMm = -1;

int   usAdc     = 0;
float usRawMm   = 0;
float usFiltMm  = -1;

Scene candidate = SCENE_CLEAR;
Scene confirmed = SCENE_CLEAR;
int   streak    = 0;
bool  stopped   = false;

int   dcPwm     = 0;

unsigned long lastMs = 0;
char buf[120];

// ======================= ULTRASONIC =======================
// LV-MaxSonar-EZ analog out = Vcc/512 per inch -> ~2 ADC counts per inch at 5 V.
// Check against a tape measure (transfer-function plot). Blind below ~15 cm.
void usUpdate() {
  usAdc   = analogRead(PIN_US_AN);
  usRawMm = (usAdc / 2.0) * 25.4;
  usFiltMm = (usFiltMm < 0) ? usRawMm : US_ALPHA * usRawMm + (1.0 - US_ALPHA) * usFiltMm;
}

// ======================= TOF =======================
bool tofBegin() {
  Wire.begin();
  if (tof.begin() != 0) return false;   // 0 = success
  tof.setDistanceModeLong();            // up to ~4 m
  tof.setTimingBudgetInMs(50);
  tof.setIntermeasurementPeriod(50);
  tof.startRanging();
  return true;
}

void tofUpdate() {
  if (!tofOk || !tof.checkForDataReady()) return;
  tofRawMm = tof.getDistance();
  tofValid = (tof.getRangeStatus() == 0);   // 0 = good measurement
  tof.clearInterrupt();
  if (tofValid) {
    tofFiltMm = (tofFiltMm < 0) ? tofRawMm : TOF_ALPHA * tofRawMm + (1.0 - TOF_ALPHA) * tofFiltMm;
  }
}

// ======================= DC MOTOR =======================
void dcSetPwm(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0)      { digitalWrite(PIN_DC_IN1, HIGH); digitalWrite(PIN_DC_IN2, LOW);  }
  else if (pwm < 0) { digitalWrite(PIN_DC_IN1, LOW);  digitalWrite(PIN_DC_IN2, HIGH); }
  else              { digitalWrite(PIN_DC_IN1, LOW);  digitalWrite(PIN_DC_IN2, LOW);  }
  analogWrite(PIN_DC_ENA, abs(pwm));
  dcPwm = pwm;
}

// ======================= FUSION =======================
Scene classify(int usMm, int tofMm, bool valid, int threshMm) {
  bool usNear  = usMm < threshMm;
  bool tofNear = valid && tofMm >= 0 && tofMm < threshMm;

  if (usNear && tofNear)  return SCENE_SOLID;        // both see it
  if (usNear && !tofNear) return SCENE_TRANSPARENT;  // sound bounced, light passed through
  if (!usNear && tofNear) return SCENE_SMOKE;        // light scattered, sound passed through
  return SCENE_CLEAR;
}

void fusionUpdate(int usMm, int tofMm, bool valid) {
  // while stopped, use the larger threshold so the motor doesn't flicker at the edge
  int thresh = stopped ? RESTART_MM : STOP_MM;
  Scene s = classify(usMm, tofMm, valid, thresh);

  // temporal consistency: accept a new scene only after CONFIRM_N readings in a row
  if (s == candidate) { if (streak < CONFIRM_N) streak++; }
  else                { candidate = s; streak = 1; }
  if (streak >= CONFIRM_N) confirmed = candidate;

  // smoke is not a physical obstacle -> keep going
  stopped = (confirmed == SCENE_SOLID || confirmed == SCENE_TRANSPARENT);
}

const char* sceneName(Scene s) {
  switch (s) {
    case SCENE_SOLID:       return "SOLID";
    case SCENE_TRANSPARENT: return "TRANSPARENT";
    case SCENE_SMOKE:       return "SMOKE";
    default:                return "CLEAR";
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(PIN_US_AN, INPUT);
  pinMode(PIN_DC_ENA, OUTPUT);
  pinMode(PIN_DC_IN1, OUTPUT);
  pinMode(PIN_DC_IN2, OUTPUT);
  dcSetPwm(0);

  tofOk = tofBegin();
  if (!tofOk) Serial.println("ERR,VL53L1X not found - check wiring");
}

void loop() {
  tofUpdate();                         

  if (millis() - lastMs >= 50) {       // 20 Hz
    lastMs = millis();
    usUpdate();

    fusionUpdate((int)usFiltMm, (int)tofFiltMm, tofValid);
    dcSetPwm(stopped ? 0 : RUN_PWM);


    snprintf(buf, sizeof(buf),
             "US_RAW_MM,%d,US_MM,%d,TOF_RAW_MM,%d,TOF_MM,%d,TOF_VALID,%d,SCENE,%s,MOTOR,%s",
             (int)usRawMm, (int)usFiltMm, tofRawMm, (int)tofFiltMm, tofValid ? 1 : 0,
             sceneName(confirmed), stopped ? "STOP" : "RUN");
    Serial.println(buf);
  }
}
