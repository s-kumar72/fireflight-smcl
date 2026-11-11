const int SENSOR_PIN = 12;

void setup() {
  Serial.begin(9600);
  pinMode(SENSOR_PIN, INPUT_PULLUP);
}

void loop() {
  if (digitalRead(SENSOR_PIN) == HIGH) {
    Serial.println("BLOCKED");
  } else {
    Serial.println("OPEN");
  }
}