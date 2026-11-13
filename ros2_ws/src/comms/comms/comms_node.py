# Start the comms node: ros2 run comms comms_node
"""ROS 2 node communication between the Arduino and the UI"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, UInt8, String
import serial
from itertools import cycle
from comms.types import SensorType, MotorType
import time

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
TIMEOUT = 1.0
SENSOR_REQ_FREQ = 5  # Hz per sensor; requests are spaced evenly across a cycle.

REQUEST_MSGS = {
    SensorType.POTENTIOMETER: b'REQ:POT\n',
    SensorType.THERMISTOR: b'REQ:THERM\n',
    SensorType.ULTRASONIC: b'REQ:ULTR\n',
    SensorType.SLOT: b'REQ:SLOT\n',
    SensorType.TOF: b'REQ:TOF\n',
    SensorType.DC_ENCODER: b'REQ:DCE\n',
    SensorType.DC_RPM: b'REQ:DCR\n',
    SensorType.DES_RPM: b'REQ:DES_RPM\n',
    SensorType.UI_MODE: b'REQ:UIMODE\n',
    SensorType.STEPPER_MOVED: b'REQ:STA\n',
}

RESPONSE_HEADERS = {
    'POT': SensorType.POTENTIOMETER,
    'TRM': SensorType.THERMISTOR,
    'ULT': SensorType.ULTRASONIC,
    'SLT': SensorType.SLOT,
    'TOF': SensorType.TOF,
    'DCE': SensorType.DC_ENCODER,
    'DCR': SensorType.DC_RPM,
    'DES_RPM': SensorType.DES_RPM,
    'UIMODE': SensorType.UI_MODE,
    'STA': SensorType.STEPPER_MOVED,
}



class CommsNode(Node):
    """Handle communication between ROS 2 and external devices."""

    def __init__(self):
        """Initialize the node and configure communications here."""
        super().__init__('comms')

        # Arduino goes 0-1023 for adc

        self.get_logger().info('Comms node started')

        self.pot_pub = self.create_publisher(Int32, 'potentiometer', 10) # 0-1023
        self.thermistor_pub = self.create_publisher(Int32, 'thermistor', 10) # 0-1023
        self.ultrasonic_pub = self.create_publisher(Int32, 'ultrasonic', 10) # 0-4000
        self.slot_pub = self.create_publisher(Int32, 'slot', 10) # RPM, 0-4095
        self.tof_pub = self.create_publisher(Int32, 'tof', 10) # 0~4000
        self.dc_encoder_pub = self.create_publisher(Int32, 'dc_encoder', 10) # Signed counts, 420/rev
        self.dc_rpm_pub = self.create_publisher(Float32, 'dc_rpm', 10)
        self.des_rpm_pub = self.create_publisher(Float32, 'des_rpm', 10)
        self.ui_mode_pub = self.create_publisher(Bool, 'ui_mode', 10)
        self.stepper_moved_pub = self.create_publisher(Float32, 'stepper_moved', 10)

        # Was thinking don't request from UI, just request here repeatedly
        # self.read_request_sub = self.create_subscription(
        #     UInt8, 'sensor_read_requests', self.sensorRequestCallback, 100)

        self.motor_command_sub = self.create_subscription(
            String, 'motor_commands', self.motorCommandCallback, 100)


        self.sensor_values = {
            SensorType.POTENTIOMETER: 0,
            SensorType.THERMISTOR: 0,
            SensorType.ULTRASONIC: 0,
            SensorType.SLOT: 0,
            SensorType.TOF: 0,
            SensorType.DC_ENCODER: 0,
            SensorType.DC_RPM: 0.0,
            SensorType.DES_RPM: 0.0,
            SensorType.UI_MODE: None,
            SensorType.STEPPER_MOVED: 0.0,
        }
        
        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        except serial.SerialException as exc:
            self.get_logger().error(f'Failed to open {SERIAL_PORT} at {BAUD_RATE} baud: {exc}')
            raise
        self.get_logger().info(f'Connected to {self.serial.port}')
        self._sensor_requests = cycle(REQUEST_MSGS)
        self.sensor_request_timer = self.create_timer(
            1.0 / (SENSOR_REQ_FREQ * len(REQUEST_MSGS)), self.request_next_sensor)
    
    # MOTOR CONTROL PROTOCOL NOTE:
    # SERVO goes SER:<ANGLE>\n WHERE ANGLE IS STR(INT) [0, 90] (degrees)
    # STEPPER goes STP:<DIR>:<ANGLE>\n WHERE DIR IS "+" OR "-" AND ANGLE IS STR(INT) [0, 32767] (degrees)
    # DC goes DCM:<MODE>:<DIR>:<VALUE>\n WHERE MODE IS "P" for POSITION or "V" for VELOCITY
    # AND DIR IS "+" OR "-" AND FOR RPM is [0, 60] and for POSITION is [0, 32767] (degrees)
    # Each DC command resets the encoder origin; POSITION is a relative rotation.
    # DCE reports signed counts since that reset; DCR reports measured RPM.
    # REQ:DES_RPM returns DES_RPM:<commanded RPM> in either control mode.
    # REQ:UIMODE returns UIMODE:1 for UI control, UIMODE:0 for sensor control.

    def motorCommandCallback(self, msg: String):
        self.serial.write(msg.data.encode('utf-8'))

    # def sensorRequestCallback(self, msg: UInt8):
    #     self.request_data(SensorType(msg.data))

    def read_serial(self, wait_time: float = 0.1):
        now = time.time()
        while time.time() - now < wait_time:
            if self.serial.in_waiting > 0:
                msg = self.serial.readline().decode('utf-8').strip()
                break
        
        msgSplit = msg.split(':')

        if msgSplit[0] in RESPONSE_HEADERS and len(msgSplit) > 1:
            sensorType = RESPONSE_HEADERS[msgSplit[0]]
            match sensorType:
                case SensorType.POTENTIOMETER:
                    self.updatePotValue(msgSplit[1])
                case SensorType.THERMISTOR:
                    self.updateThermistorValue(msgSplit[1])
                case SensorType.ULTRASONIC:
                    self.updateUltrasonicValue(msgSplit[1])
                case SensorType.SLOT:
                    self.updateSlotValue(msgSplit[1])
                case SensorType.TOF:
                    self.updateTofValue(msgSplit[1])
                case SensorType.DC_ENCODER:
                    self.updateDcEncoderValue(msgSplit[1])
                case SensorType.DC_RPM:
                    self.updateDcRpmValue(msgSplit[1])
                case SensorType.DES_RPM:
                    self.updateDesRpmValue(msgSplit[1])
                case SensorType.UI_MODE:
                    self.updateUIModeValue(msgSplit[1])
                case SensorType.STEPPER_MOVED:
                    self.updateStepperMovedValue(msgSplit[1])
        else:
            self.get_logger().warning(f'Response not recognized: {msg}')        

    # region Sensor Value Update Functions
    def request_next_sensor(self):
        # All requests together exceed the Uno's 64-byte RX buffer, dropping
        # the trailing UIMODE request if loop() is busy. Send one per tick.
        self.request_data(next(self._sensor_requests))

    def request_data(self, sensor_type: SensorType):
        # Sends a request to the Arduino to get some sensor reading
        self.serial.write(REQUEST_MSGS[sensor_type])
    
    def updatePotValue(self, value: str):
        self.sensor_values[SensorType.POTENTIOMETER] = int(value)
        self.pot_pub.publish(Int32(data=self.sensor_values[SensorType.POTENTIOMETER]))
    
    def updateThermistorValue(self, value: str):
        self.sensor_values[SensorType.THERMISTOR] = int(value)
        self.thermistor_pub.publish(Int32(data=self.sensor_values[SensorType.THERMISTOR]))
    
    def updateUltrasonicValue(self, value: str):
        self.sensor_values[SensorType.ULTRASONIC] = int(value)
        self.ultrasonic_pub.publish(Int32(data=self.sensor_values[SensorType.ULTRASONIC]))
    
    def updateSlotValue(self, value: str):
        self.sensor_values[SensorType.SLOT] = max(0, min(4095, int(value)))
        self.slot_pub.publish(Int32(data=self.sensor_values[SensorType.SLOT]))
    
    def updateTofValue(self, value: str):
        self.sensor_values[SensorType.TOF] = int(value)
        self.tof_pub.publish(Int32(data=self.sensor_values[SensorType.TOF]))

    def updateDcEncoderValue(self, value: str):
        self.sensor_values[SensorType.DC_ENCODER] = int(value)
        self.dc_encoder_pub.publish(Int32(data=self.sensor_values[SensorType.DC_ENCODER]))

    def updateDcRpmValue(self, value: str):
        self.sensor_values[SensorType.DC_RPM] = float(value)
        self.dc_rpm_pub.publish(Float32(data=self.sensor_values[SensorType.DC_RPM]))

    def updateDesRpmValue(self, value: str):
        self.sensor_values[SensorType.DES_RPM] = float(value)
        self.des_rpm_pub.publish(Float32(data=self.sensor_values[SensorType.DES_RPM]))

    def updateUIModeValue(self, value: str):
        value = value.strip()
        if value not in ('0', '1'):
            self.get_logger().warning(f'Invalid UIMode boolean: {value}')
            return
        self.sensor_values[SensorType.UI_MODE] = value == '1'
        self.ui_mode_pub.publish(Bool(data=self.sensor_values[SensorType.UI_MODE]))
    
    def updateStepperMovedValue(self, value: str):
        self.sensor_values[SensorType.STEPPER_MOVED] = float(value)
        self.stepper_moved_pub.publish(Float32(data=self.sensor_values[SensorType.STEPPER_MOVED]))
    # endregion Sensor Value Update Functions

def main(args=None):
    """Run the communications node until shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = CommsNode()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.serial.in_waiting > 0:
                node.read_serial()
    except serial.SerialException:
        pass
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # TODO: Close device connections here before destroying the node.
        if node is not None and hasattr(node, 'serial') and node.serial.is_open:
            node.serial.close()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
