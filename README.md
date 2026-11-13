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

| Ultrasonic | ToF | Scene |
|---|---|---|
| far | far | CLEAR |
| near | near | SOLID |
| near | far/no valid echo | TRANSPARENT |
| far | near | SMOKE |

- **"Near"** = closer than 300 mm. A ToF reading only counts if the sensor marks it valid (range status 0).
- **Filtering:** exponential moving average on both sensors.

**Slot sensor:**

**Ultrasonic sensor:**

The UI's bottom-right row shows smoke detection on the left and **Control Mode**
on the right. In `smcl_integrated`, the debounced button on digital 13 toggles
between **Sensor Control** (startup) and **UI Control**. The `UIMode` sensor
accepts `REQ:UIMODE\n` in either mode and replies with `UIMODE:0\n` for sensor
control or `UIMODE:1\n` for UI control. Comms publishes `std_msgs/Bool` on
`ui_mode` (`true` means UI control); the display shows `—` until a reply arrives.

Ultrasonic telemetry uses raw 10-bit ADC counts: `REQ:ULTR\n` replies with
`ULT:<ADC>\n`. With the MaxSonar scaling assumed by this project (Vcc/512 volts
per inch and ADC reference equal to Vcc), distance is `ADC / 2` inches or
`ADC × 0.0127` meters. For example, ADC 100 displays 1.27 m. The Arduino keeps
a separate filtered millimeter value for its sensor control logic.

### INDIVIDUAL CONTRIBUTIONS & CHALLENGES:
#### Sadhana Kumar 
- _Contributions_: Implemented potentiometer reading and ADC-to-angle conversion and integrated the serial request handler for the comms node. Implemented a non-blocking STEP/DIR stepper driver for the EasyDriver so the potentiometer angle and the stepper angle are linked (pot can be used to change the angle of the stepper). Also measured the pot's transfer function with a paper angle dial at 45 degree increments and plotted this along with the measurement error. Wired and tested the pot, EasyDriver, and stepper on the bench supply. Added reporting of the stepper's actual angle moved for GUI commands.
- _Challenges_: Calibration errors from a slipping pointer (initial attempts gave readings at only 70% of the dial angles because the paper pointer was twisting on the knob. This was fixed by taping the pointer all the way around the knob and by checking for ~165 counts per 45 degrees while measuring. Also, the EasyDriver's LED stayed off even while other parts of the circuit had power / LEDs on; I traced the 12V supply voltage step by step using the multimeter and found that the other driver's lit LED had been lit by the Arduino's USB power, not the 12V rail as assumed. This helped deduce that the wires clipped to the banana plugs had come undone. 


#### Joel Milla
- I worked on wiring and integrating the slot sensor (Omron EE-SX1042), which was my first time working with a breadboard and electrical components, so a significant part of my contribution was learning how to read a sensor datasheet and translate it into a correct circuit. Although the slot sensor only required four connections (power, ground, signal output, and a pull-up/current-limiting resistor), and understanding why each connection was needed.I connected the sensor to the Arduino’s digital input pin, and converted the raw digital signal into a derived physical quantity (RPM/event count) as required by the lab spec.

Challenges
- I spent about three hours debugging the board because the arduino was not reading whether the slot was open or blocked. The symptom was that the sensor output never changed state regardless of whether the beam was broken. I used a multimeter to check for continuity and voltage drop across the circuit and was able to find that the resistor being used had too high resistance whcih limited the current causing the output to never reached the arduino. This taught me that resistor selection isn’t just about preventing damage from excess current, it also has to supply enough current for the sensor’s output stage to function correctly.
