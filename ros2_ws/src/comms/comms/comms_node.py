# Start the comms node: ros2 run comms comms_node
"""ROS 2 node communication between the Arduino and the UI"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, UInt8, String
import serial
from comms.types import SensorType, MotorType
import time

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
TIMEOUT = 1.0
SENSOR_REQ_FREQ = 5  # Hz; each cycle requests every sensor in order.

REQUEST_MSGS = {
    SensorType.POTENTIOMETER: b'REQ:POT\n',
    SensorType.THERMISTOR: b'REQ:THERM\n',
    SensorType.ULTRASONIC: b'REQ:ULTR\n',
    SensorType.SLOT: b'REQ:SLOT\n',
    SensorType.TOF: b'REQ:TOF\n',
    SensorType.DC_ENCODER: b'REQ:DCE\n',
}

RESPONSE_HEADERS = {
    'POT': SensorType.POTENTIOMETER,
    'TRM': SensorType.THERMISTOR,
    'ULT': SensorType.ULTRASONIC,
    'SLT': SensorType.SLOT,
    'TOF': SensorType.TOF,
    'DCE': SensorType.DC_ENCODER,
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
        self.dc_encoder_pub = self.create_publisher(Int32, 'dc_encoder', 10) # 0-4096, mapping tbd

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
        }
        
        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        except serial.SerialException as exc:
            self.get_logger().error(f'Failed to open {SERIAL_PORT} at {BAUD_RATE} baud: {exc}')
            raise
        self.get_logger().info(f'Connected to {self.serial.port}')
        self.sensor_request_timer = self.create_timer(
            1.0 / SENSOR_REQ_FREQ, self.request_all_sensors)
    
    # MOTOR CONTROL PROTOCOL NOTE:
    # SERVO goes SER:<ANGLE>\n WHERE ANGLE IS STR(INT) [0, 180] (degrees, may need to be remapped, check arduino code)
    # STEPPER goes STP:<DIR>:<ANGLE>\n WHERE DIR IS "+" OR "-" AND ANGLE IS STR(INT) [0, 32767] (degrees)
    # DC goes DCM:<MODE>:<DIR>:<VALUE>\n WHERE MODE IS "P" for POSITION or "V" for VELOCITY
    # AND DIR IS "+" OR "-" AND FOR RPM is [0, 60] and for POSITION is [0, 32767] (degrees)

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
        else:
            self.get_logger().warning(f'Response not recognized: {msg}')        

    # region Sensor Value Update Functions
    def request_all_sensors(self):
        for sensor_type in REQUEST_MSGS:
            self.request_data(sensor_type)

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
