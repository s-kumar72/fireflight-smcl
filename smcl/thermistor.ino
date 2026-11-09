// thermistor.ino — 10k NTC thermistor on A1
// Wiring (voltage divider): 5V -> 10k fixed resistor -> A1 -> thermistor -> GND
//
// Serial protocol (handled by smcl.ino):
//   laptop sends  "REQ:THERM\n"
//   Arduino sends "TRM:<raw ADC 0-1023>\n"
// The °C conversion for the GUI happens in the ROS comms node.
// thermAdcToC() is kept here for sensor-mode control of the servo.

#define THERM_PIN A1

const float R_FIXED   = 10000.0;  // fixed divider resistor (ohms)
const float R0        = 10000.0;  // thermistor resistance at T0 (ohms)
const float T0_K      = 298.15;   // 25 °C in kelvin
const float BETA      = 3950.0;   // placeholder: replace with calibrated value
const int   N_SAMPLES = 16;       // ADC readings averaged per sample

// Averaged raw ADC reading (0-1023)
int thermReadAdc() {
  long sum = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    sum += analogRead(THERM_PIN);
  }
  return (int)(sum / N_SAMPLES);
}

// Raw ADC -> thermistor resistance -> temperature (°C) via the beta equation
float thermAdcToC(int adc) {
  if (adc <= 0)    adc = 1;       // avoid divide-by-zero / log(0)
  if (adc >= 1023) adc = 1022;
  float rTherm = R_FIXED * adc / (1023.0 - adc);
  float invT   = 1.0 / T0_K + log(rTherm / R0) / BETA;
  return 1.0 / invT - 273.15;
}

// Convenience: current temperature in °C
float thermReadC() {
  return thermAdcToC(thermReadAdc());
}

// Reply to "REQ:THERM" with the raw ADC value
void thermHandleRequest() {
  Serial.print("TRM:");
  Serial.println(thermReadAdc());
}
