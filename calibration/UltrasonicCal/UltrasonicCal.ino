// UltrasonicCal.ino — calibration + transfer function for Pololu 723 (MaxBotix LV-MaxSonar-EZ0, MB1000)
// Team G, Sensors & Motor Control Lab
//
// Wiring: AN -> A1, +5 -> 5V, GND -> GND
// Serial Monitor: 115200 baud, line ending = "Newline"
//
// HOW TO USE
//   1. Put a flat target (a book or cardboard, at least 20x20 cm) square to the sensor.
//   2. Measure from the FRONT FACE of the sensor to the target with a tape measure.
//   3. Type that distance in cm (e.g. 30) and press Enter. The sketch averages N_SAMPLES readings.
//   4. Repeat at 8-12 distances across the range (e.g. 20, 30, 40, 60, 80, 100, 130, 160, 200 cm).
//   5. Type  f  -> prints the transfer-function table, the best-fit line, and error at every point.
//
// Commands:  <number> = record a point at that true distance (cm)
//            f = fit + report      p = print points      u = undo last point
//            c = clear all         l = live readings on/off
//
// Notes: objects closer than ~15 cm read as ~15 cm, so start at 20 cm.
//        EZ0 has a wide beam — keep the table edge and your body out of its cone.
//        Keep nothing in front of the sensor for the first second after power-up (it self-calibrates).

#define PIN_US_AN A1

const int   N_SAMPLES  = 20;      // readings averaged per point (sensor updates every ~49 ms)
const int   MAX_POINTS = 15;
const float VREF       = 5.0;

float trueCm[MAX_POINTS];
float meanAdc[MAX_POINTS];
float stdAdc[MAX_POINTS];
int   nPoints = 0;

bool live = true;
unsigned long lastLive = 0;

// Datasheet scaling: Vcc/512 per inch -> ~2 ADC counts per inch at 5 V
float datasheetCm(float adc) { return (adc / 2.0) * 2.54; }

void recordPoint(float cm) {
  if (nPoints >= MAX_POINTS) { Serial.println("Max points reached. Type f to fit or c to clear."); return; }

  Serial.print("Measuring at "); Serial.print(cm, 1); Serial.println(" cm ... hold still");
  float sum = 0, sumSq = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    int a = analogRead(PIN_US_AN);
    sum += a;
    sumSq += (float)a * a;
    delay(50);                              // wait for the next sonar update
  }
  float m = sum / N_SAMPLES;
  float var = sumSq / N_SAMPLES - m * m;
  float s = (var > 0) ? sqrt(var) : 0;

  trueCm[nPoints] = cm;
  meanAdc[nPoints] = m;
  stdAdc[nPoints] = s;
  nPoints++;

  Serial.print("  point "); Serial.print(nPoints);
  Serial.print(": mean ADC = "); Serial.print(m, 2);
  Serial.print("  (std "); Serial.print(s, 2);
  Serial.print(")  -> datasheet says "); Serial.print(datasheetCm(m), 1); Serial.println(" cm");
}

void printPoints() {
  Serial.println();
  Serial.println("TRANSFER FUNCTION DATA (x = physical distance, y = raw output)");
  Serial.println("true_cm,mean_adc,std_adc,volts");
  for (int i = 0; i < nPoints; i++) {
    Serial.print(trueCm[i], 1);           Serial.print(",");
    Serial.print(meanAdc[i], 2);          Serial.print(",");
    Serial.print(stdAdc[i], 2);           Serial.print(",");
    Serial.println(meanAdc[i] * VREF / 1023.0, 4);
  }
  Serial.println("(copy these rows into Excel / Sheets / Python to plot the transfer function)");
}

// least squares y = a*x + b
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

  // Transfer function: ADC = m * distance + c
  float m, c;
  fitLine(trueCm, meanAdc, nPoints, m, c);

  // Calibration (inverse): distance_cm = a * ADC + b
  float a, b;
  fitLine(meanAdc, trueCm, nPoints, a, b);

  Serial.println();
  Serial.println("TRANSFER FUNCTION FIT");
  Serial.print("  ADC = "); Serial.print(m, 4); Serial.print(" * distance_cm + "); Serial.println(c, 3);
  Serial.print("  (datasheet ideal: 0.7874 counts per cm, i.e. 2 per inch)"); Serial.println();

  Serial.println();
  Serial.println("ERROR AT EACH POINT");
  Serial.println("true_cm,datasheet_cm,datasheet_err_cm,calibrated_cm,calibrated_err_cm");
  float sqD = 0, sqC = 0, maxD = 0, maxC = 0;
  for (int i = 0; i < nPoints; i++) {
    float dCm = datasheetCm(meanAdc[i]);
    float cCm = a * meanAdc[i] + b;
    float eD = dCm - trueCm[i];
    float eC = cCm - trueCm[i];
    sqD += eD * eD;  sqC += eC * eC;
    if (fabs(eD) > maxD) maxD = fabs(eD);
    if (fabs(eC) > maxC) maxC = fabs(eC);
    Serial.print(trueCm[i], 1); Serial.print(",");
    Serial.print(dCm, 2);       Serial.print(",");
    Serial.print(eD, 2);        Serial.print(",");
    Serial.print(cCm, 2);       Serial.print(",");
    Serial.println(eC, 2);
  }
  Serial.println();
  Serial.print("Datasheet scaling:  RMS error = "); Serial.print(sqrt(sqD / nPoints), 2);
  Serial.print(" cm, max error = "); Serial.print(maxD, 2); Serial.println(" cm");
  Serial.print("Your calibration:   RMS error = "); Serial.print(sqrt(sqC / nPoints), 2);
  Serial.print(" cm, max error = "); Serial.print(maxC, 2); Serial.println(" cm");

  Serial.println();
  Serial.println("PASTE INTO FusionStop.ino (replace the usRawMm line in usUpdate()):");
  Serial.print("  usRawMm = ("); Serial.print(a, 5); Serial.print(" * usAdc + ");
  Serial.print(b, 3); Serial.println(") * 10.0;");
  Serial.println();
}

void handleCommand(String s) {
  s.trim();
  if (s.length() == 0) return;
  char ch = s.charAt(0);
  if (ch == 'f') fitAndReport();
  else if (ch == 'p') printPoints();
  else if (ch == 'l') { live = !live; Serial.println(live ? "live ON" : "live OFF"); }
  else if (ch == 'u') { if (nPoints > 0) nPoints--; Serial.print("points: "); Serial.println(nPoints); }
  else if (ch == 'c') { nPoints = 0; Serial.println("cleared"); }
  else {
    float cm = s.toFloat();
    if (cm > 0) recordPoint(cm);
    else Serial.println("Type a distance in cm, or f / p / u / c / l");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_US_AN, INPUT);
  delay(1000);   // sonar power-up self-calibration — keep the area in front clear
  Serial.println();
  Serial.println("=== Ultrasonic calibration (LV-MaxSonar-EZ0) ===");
  Serial.println("Type the true distance in cm and press Enter. f = fit, p = points, u = undo, c = clear, l = live");
}

void loop() {
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));

  if (live && millis() - lastLive >= 500) {
    lastLive = millis();
    int adc = analogRead(PIN_US_AN);
    Serial.print("live: ADC "); Serial.print(adc);
    Serial.print("  ~"); Serial.print(datasheetCm(adc), 1); Serial.println(" cm (datasheet)");
  }
}
