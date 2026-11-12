# fireflight-smcl
FireFlight Sensors &amp; Motor Control Lab

*SENSORS:*

**Potentiometer:**
- `smcl/pot.ino`: reads from A0, converts ADC counts to degrees
- `smcl/stepper.ino`: STEP/DIR driver with absolute moves (when controlled by sensor) and relative moves (when controlled by GUI)

_Transfer function_:
Raw counts were measured by placing a folded paper dial (45 degree increments) under the knob and recording the reading at each mark. The best-fit line was calculated from these points as ADC = 3.649 × angle + 0.3 (R² = 0.99991). 

- Measured electrical travel: 280.2° (datasheet: 280°)
- Measured error: max 1.47°, RMS 0.90° (most of this is probably measurement error)
- Since a pot is a resistive voltage divider with no dynamics, its transfer function is just a gain (3.649 counts / degree - same as slope from best fit line)

![Potentiometer transfer function](pot_tf_plot.png)

**Thermistor:**

**TOF & ultrasonic sensors:**
Scene classification (Clear, Smoke, Transparent, Solid) using Ultrasonic and TOF sensor (based on https://arxiv.org/pdf/2510.06518 )

**Slot sensor:**

**Ultrasonic sensor:**
- .
- DC Motor RPM control (PID) based on distance from object using ultrasonic sensor.
- Thermistor controls servo so that it can moves the thermal camera towards the higher temp region & transfer function
- Potentiometer controls stepper to adjust the motor speed for spinning radar

