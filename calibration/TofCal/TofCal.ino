// TofCal.ino — calibration + error measurement for SparkFun VL53L1X (time-of-flight)
// Team G, Sensors & Motor Control Lab
//
// Wiring: SDA -> A4, SCL -> A5, 3V3 -> 3.3V, GND -> GND
// Library: "SparkFun VL53L1X 4m Laser Distance Sensor"
// Serial Monitor: 115200 baud, line ending = "Newline"
//
// HOW TO USE
//   1. Use a flat, matte, light-coloured target (white card/paper), square to the sensor.
//   2. Measure from the sensor's FRONT FACE (the little black window) to the target.
//   3. Type that distance in cm (e.g. 30) and press Enter. The sketch averages N_SAMPLES valid readings.
//   4. Repeat at 8-12 distances (e.g. 5, 10, 20, 30, 50, 75, 100, 150, 200 cm).
//   5. Type  f  -> prints the best-fit correction and the error at every point.
//
// Optional built-in offset calibration (ST's method):
//   Put the white target at exactly 14.0 cm and type  o . The sensor stores an offset
//   for this power cycle; the sketch prints the value so you can hard-code it with setOffset().
//
// Commands:  <number> = record a point at that true distance (cm)
//            f = fit + report      p = print points      u = undo last point
//            c = clear all         l = live readings on/off      o = sensor offset calibration at 14 cm
//
// Note: the VL53L1X already outputs mm, so this is a correction (true = a*measured + b),
// not a transfer function. Use the ultrasonic for the transfer-function plot.

#include <Wire.h>
#include <SparkFun_VL53L1X.h>

SFEVL53L1X tof;

const int N_SAMPLES  = 30;
const int MAX_POINTS = 15;

float trueMm[MAX_POINTS];
float meanMm[MAX_POINTS];
float stdMm[MAX_POINTS];
int   validCnt[MAX_POINTS];
int   nPoints = 0;

bool live = true;
unsigned long lastLive = 0;

// Blocks until the next measurement; returns distance and fills status (0 = good)
int readOnce(int& status) {
  unsigned long t0 = millis();
  while (!tof.checkForDataReady()) {
    if (millis() - t0 > 500) { status = 255; return -1; }   // timeout
    delay(1);
  }
  int d = tof.getDistance();
  status = tof.getRangeStatus();
  tof.clearInterrupt();
  return d;
}

void recordPoint(float cm) {
  if (nPoints >= MAX_POINTS) { Serial.println("Max points reached. Type f to fit or c to clear."); return; }

  Serial.print("Measuring at "); Serial.print(cm, 1); Serial.println(" cm ... hold still");
  float sum = 0, sumSq = 0;
  int good = 0, bad = 0;
  while (good < N_SAMPLES && bad < N_SAMPLES * 2) {
    int st;
    int d = readOnce(st);
    if (st == 0 && d >= 0) { sum += d; sumSq += (float)d * d; good++; }
    else bad++;
  }
  if (good < 5) {
    Serial.print("  Too few valid readings ("); Serial.print(good);
    Serial.println("). Target too far / too dark / not square? Point NOT recorded.");
    return;
  }
  float m = sum / good;
  float var = sumSq / good - m * m;
  float s = (var > 0) ? sqrt(var) : 0;

  trueMm[nPoints] = cm * 10.0;
  meanMm[nPoints] = m;
  stdMm[nPoints] = s;
  validCnt[nPoints] = good;
  nPoints++;

  Serial.print("  point "); Serial.print(nPoints);
  Serial.print(": mean = "); Serial.print(m, 1);
  Serial.print(" mm (std "); Serial.print(s, 1);
  Serial.print(" mm), error = "); Serial.print(m - cm * 10.0, 1);
  Serial.print(" mm, valid "); Serial.print(good); Serial.print("/"); Serial.println(good + bad);
}

void printPoints() {
  Serial.println();
  Serial.println("true_mm,measured_mm,std_mm,valid_readings");
  for (int i = 0; i < nPoints; i++) {
    Serial.print(trueMm[i], 0); Serial.print(",");
    Serial.print(meanMm[i], 1); Serial.print(",");
    Serial.print(stdMm[i], 1);  Serial.print(",");
    Serial.println(validCnt[i]);
  }
}

void fitLine(const float* x, const float* y, int n, float& a, float& b) {
  float sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int i = 0; i < n; i++) { sx += x[i]; sy += y[i]; sxx += x[i] * x[i]; sxy += x[i] * y[i]; }
  float d = n * sxx - sx * sx;
  a = (n * sxy - sx * sy) / d;
  b = (sy - a * sx) / n;
}

void fitAndReport() {
  if (nPoints < 3) { Serial.println("Need at least 3 points."); return; }
  printPoints();

  // Correction: true_mm = a * measured_mm + b
  float a, b;
  fitLine(meanMm, trueMm, nPoints, a, b);

  Serial.println();
  Serial.println("ERROR AT EACH POINT");
  Serial.println("true_mm,raw_mm,raw_err_mm,corrected_mm,corrected_err_mm");
  float sqR = 0, sqC = 0, maxR = 0, maxC = 0;
  for (int i = 0; i < nPoints; i++) {
    float corr = a * meanMm[i] + b;
    float eR = meanMm[i] - trueMm[i];
    float eC = corr - trueMm[i];
    sqR += eR * eR;  sqC += eC * eC;
    if (fabs(eR) > maxR) maxR = fabs(eR);
    if (fabs(eC) > maxC) maxC = fabs(eC);
    Serial.print(trueMm[i], 0); Serial.print(",");
    Serial.print(meanMm[i], 1); Serial.print(",");
    Serial.print(eR, 1);        Serial.print(",");
    Serial.print(corr, 1);      Serial.print(",");
    Serial.println(eC, 1);
  }
  Serial.println();
  Serial.print("Raw sensor:   RMS error = "); Serial.print(sqrt(sqR / nPoints), 1);
  Serial.print(" mm, max error = "); Serial.print(maxR, 1); Serial.println(" mm");
  Serial.print("Corrected:    RMS error = "); Serial.print(sqrt(sqC / nPoints), 1);
  Serial.print(" mm, max error = "); Serial.print(maxC, 1); Serial.println(" mm");

  Serial.println();
  Serial.println("PASTE INTO FusionStop.ino (in tofUpdate(), right after tofRawMm = tof.getDistance();):");
  Serial.print("  tofRawMm = (int)("); Serial.print(a, 5); Serial.print(" * tofRawMm + ");
  Serial.print(b, 2); Serial.println(");");
  Serial.println();
}

void offsetCal() {
  Serial.println("Offset calibration: white target at exactly 140 mm, square to the sensor...");
  tof.stopRanging();
  tof.calibrateOffset(140);
  int16_t off = tof.getOffset();
  tof.startRanging();
  Serial.print("Offset stored for this session: "); Serial.print(off); Serial.println(" mm");
  Serial.print("To make it permanent, add after startRanging() in FusionStop:  tof.setOffset(");
  Serial.print(off); Serial.println(");");
}

void handleCommand(String s) {
  s.trim();
  if (s.length() == 0) return;
  char ch = s.charAt(0);
  if (ch == 'f') fitAndReport();
  else if (ch == 'p') printPoints();
  else if (ch == 'o') offsetCal();
  else if (ch == 'l') { live = !live; Serial.println(live ? "live ON" : "live OFF"); }
  else if (ch == 'u') { if (nPoints > 0) nPoints--; Serial.print("points: "); Serial.println(nPoints); }
  else if (ch == 'c') { nPoints = 0; Serial.println("cleared"); }
  else {
    float cm = s.toFloat();
    if (cm > 0) recordPoint(cm);
    else Serial.println("Type a distance in cm, or f / p / u / c / l / o");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  Serial.println();
  Serial.println("=== ToF calibration (VL53L1X) ===");
  if (tof.begin() != 0) {
    Serial.println("ERR: VL53L1X not found - check SDA->A4, SCL->A5, 3.3V, GND");
    while (true) {}
  }
  tof.setDistanceModeLong();        // same settings as FusionStop, so the calibration matches
  tof.setTimingBudgetInMs(50);
  tof.setIntermeasurementPeriod(50);
  tof.startRanging();
  Serial.println("Type the true distance in cm and press Enter. f = fit, p = points, u = undo, c = clear, l = live, o = offset cal");
}

void loop() {
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));

  if (live && millis() - lastLive >= 500) {
    lastLive = millis();
    int st;
    int d = readOnce(st);
    Serial.print("live: "); Serial.print(d); Serial.print(" mm  status ");
    Serial.print(st); Serial.println(st == 0 ? " (good)" : " (bad - ignored when recording)");
  }
}
