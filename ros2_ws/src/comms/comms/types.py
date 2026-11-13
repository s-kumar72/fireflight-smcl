"""Device identifiers shared by the communications and UI packages."""

from enum import Enum


class SensorType(Enum):
    POTENTIOMETER = 0
    THERMISTOR = 1
    ULTRASONIC = 2
    SLOT = 3
    TOF = 4
    DC_ENCODER = 5
    DC_RPM = 6
    UI_MODE = 7
    STEPPER_MOVED = 8
    DES_RPM = 9


class MotorType(Enum):
    SERVO = 0
    STEPPER = 1
    DC = 2
