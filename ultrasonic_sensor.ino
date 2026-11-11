
/*
  Distance-dependent DC motor speed control

  Sensor: Pololu 723 / MaxBotix analog output
  Controller: Arduino Uno
  Feedback: Quadrature encoder
  Driver: H-bridge

  Motor slows down as an object approaches.
*/

const int sonarPin = A1;

const int encoderA = 2;
const int encoderB = 3;

const int motorPWM = 6;
const int motorIN1 = 7;
const int motorIN2 = 8;

// -------- DISTANCE SETTINGS --------

const float START_DISTANCE = 1000.0; // mm
const float MIN_DISTANCE = 150.0;    // mm

// -------- MOTOR SPEED SETTINGS --------

const float MAX_RPM = 60.0;
const float MIN_RPM = 5.0;

// Encoder counts per revolution,
// using the selected interrupt edge
const float ENCODER_CPR = 420.0;

// -------- PID SETTINGS --------

float Kp = 10.0;
float Ki = 0.5;
float Kd = 0.001;

float integral = 0;
float previousError = 0;

float desiredRPM = MAX_RPM;
float actualRPM = 0;

// -------- SENSOR FILTER --------

float filteredDistance = 0;
const float alpha = 0.3;

// -------- TIMING --------

unsigned long lastSensorTime = 0;
unsigned long lastPIDTime = 0;

const unsigned long SENSOR_INTERVAL = 50;
const unsigned long PID_INTERVAL = 100;

volatile long encoderCount = 0;

long previousEncoderCount = 0;

// -------- ENCODER INTERRUPT --------

void encoderISR() {

  if (digitalRead(encoderB))
    encoderCount++;
  else
    encoderCount--;
}

// -------- READ DISTANCE (Ultrasonic) --------
// Input: Sonar Pin Reading
// Output: Distance in mm

float readDistance() {

  int adc = analogRead(sonarPin);

  // Approximate MaxBotix analog conversion
  // At 5 V: approximately 12.7 mm per ADC count

  return adc * 12.7;
}

// -------- SPEED REFERENCE --------
// Controlling the speed of the DC motor based on the distance from the object

float calculateDesiredRPM(float distance) {

  if (distance >= START_DISTANCE) {
    return MAX_RPM;
  }

  if (distance <= MIN_DISTANCE) {
    return MIN_RPM;
  }

  float ratio =
    (distance - MIN_DISTANCE) /
    (START_DISTANCE - MIN_DISTANCE);

  return MIN_RPM +
         ratio * (MAX_RPM - MIN_RPM);
}

// -------- MOTOR CONTROL --------

void driveMotor(int pwm) {

  pwm = constrain(pwm, 0, 255);

  digitalWrite(motorIN1, HIGH);
  digitalWrite(motorIN2, LOW);

  analogWrite(motorPWM, pwm);
}

// -------- SETUP --------

void setup() {

  Serial.begin(115200);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  pinMode(motorPWM, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);

  attachInterrupt(
    digitalPinToInterrupt(encoderA),
    encoderISR,
    RISING
  );

  filteredDistance = readDistance();

  lastSensorTime = millis();
  lastPIDTime = millis();
}

// -------- MAIN LOOP --------

void loop() {

  unsigned long now = millis();

  // 1. ULTRASONIC SENSOR UPDATE

  if (now - lastSensorTime >= SENSOR_INTERVAL) {

    lastSensorTime = now;

    float distance = readDistance();

    // Low-pass filtering
    filteredDistance =
      alpha * distance +
      (1.0 - alpha) * filteredDistance;

    // Calculate motor speed reference
    desiredRPM =
      calculateDesiredRPM(filteredDistance);
  }

  // 2. ENCODER SPEED MEASUREMENT + PID

  if (now - lastPIDTime >= PID_INTERVAL) {

    float elapsed =
      (now - lastPIDTime) / 1000.0;

    lastPIDTime = now;

    long currentCount;

    noInterrupts();
    currentCount = encoderCount;
    interrupts();

    long deltaCount =
      currentCount - previousEncoderCount;

    previousEncoderCount = currentCount;

    actualRPM =
      abs(deltaCount) * 60.0 /
      (ENCODER_CPR * elapsed);

    // 3. PID CONTROL

    float error = desiredRPM - actualRPM;

    integral += error * elapsed;

    integral = constrain(integral, -200, 200);

    float derivative =
      (error - previousError) / elapsed;

    float output =
      Kp * error +
      Ki * integral +
      Kd * derivative;

    previousError = error;

    // PWM control
    static float pwm = 0;

    pwm = constrain(output, 0, 255);

    driveMotor((int)pwm);

    // 4. MONITORING

    Serial.print("Distance: ");
    Serial.print(filteredDistance);

    Serial.print(" mm | Desired RPM: ");
    Serial.print(desiredRPM);

    Serial.print(" | Actual RPM: ");
    Serial.print(actualRPM);

    Serial.print(" | PWM: ");
    Serial.println(pwm);
  }
}
