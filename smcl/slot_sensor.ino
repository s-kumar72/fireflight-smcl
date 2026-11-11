#define SLOT_SENSOR_PIN 12

const unsigned long SLOT_WINDOW_MS = 1000;

int slotPrevState = 0;                
int slotCount = 0;                    
unsigned long slotWindowStartMs = 0;  
int slotRPM = 0;       
const int SLOT_BLADES = 3; 

void slotSetup() {
  pinMode(SLOT_SENSOR_PIN, INPUT_PULLUP);
  slotPrevState = slotReading();  // avoid a false count on the first loop
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

  if (millis() - slotWindowStartMs >= SLOT_WINDOW_MS) {
    slotRPM = slotCount * 60 / SLOT_BLADES;
    slotCount = 0;
    slotWindowStartMs = millis();
  }
}

int slotHandleRequest() {
  return slotRPM;
}
