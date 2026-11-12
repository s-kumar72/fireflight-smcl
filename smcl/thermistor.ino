#define THERM_PIN A3

const float R_FIXED   = 10000.0;
const float R0        = 10000.0;
const float T0_K      = 298.15;
const float BETA      = 3220.0;
const int   N_SAMPLES = 16;

int thermReadAdc() {
  long sum = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    sum += analogRead(THERM_PIN);
  }
  return (int)(sum / N_SAMPLES);
}

float thermAdcToC(int adc) {
  if (adc <= 0)    adc = 1;
  if (adc >= 1023) adc = 1022;
  float rTherm = R_FIXED * adc / (1023.0 - adc);
  float invT   = 1.0 / T0_K + log(rTherm / R0) / BETA;
  return 1.0 / invT - 273.15;
}

float thermReadC() {
  return thermAdcToC(thermReadAdc());
}

void thermHandleRequest() {
  Serial.print("TRM:");
  Serial.println(thermReadAdc());
}
