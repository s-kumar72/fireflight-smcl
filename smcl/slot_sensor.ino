#define SLOT_SENSOR_PIN 12

void slotSetup() {
  pinMode(SLOT_SENSOR_PIN, INPUT_PULLUP);
}

int slotReading() {
  if (digitalRead(SLOT_SENSOR_PIN) == HIGH) {
    return 1; // blocked means 1 (detected a reading)
  } else {
    return 0; // open means that its not reading anything 0
  }
}