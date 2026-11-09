"""Starter ROS 2 node for external device communications."""

import rclpy
from rclpy.node import Node
import serial
import enum

class SensorType(enum.Enum):
    POTENTIOMETER = 0
    THERMISTOR = 1
    ULTRASONIC = 2
    SLOT = 3
    TOF = 4

SERIAL_PORT = "COM3"
BAUD_RATE = 115200

REQUEST_MSGS = {
    SensorType.POTENTIOMETER: b'REQ:POT\n',
    SensorType.THERMISTOR: b'REQ:THERM\n',
    SensorType.ULTRASONIC: b'REQ:ULTR\n',
    SensorType.SLOT: b'REQ:SLOT\n',
    SensorType.TOF: b'REQ:TOF\n',
}

class CommsNode(Node):
    """Handle communication between ROS 2 and external devices."""

    def __init__(self):
        """Initialize the node and configure communications here."""
        super().__init__('comms')

        # TODO: Set up device connections, publishers, and subscriptions.
        # Use timer or subscription callbacks for communication logic.

        # Arduino goes 0-1023 for adc

        self.get_logger().info('Comms node started')

        self.pot_pub = self.create_publisher(int, 'potentiometer', 10) # 0-1023
        self.thermistor_pub = self.create_publisher(int, 'thermistor', 10) # 0-1023
        self.ultrasonic_pub = self.create_publisher(int, 'ultrasonic', 10) # 0-4000
        self.slot_pub = self.create_publisher(bool, 'slot', 10) # True if slot is blocked
        self.tof_pub = self.create_publisher(int, 'tof', 10) # 0~4000

        self.pot_value = 0
        self.thermistor_value = 0
        self.ultrasonic_value = 0
        self.slot_value = False
        self.tof_value = 0
        
        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        except serial.SerialException as exc:
            self.get_logger().error(f'Failed to open {SERIAL_PORT} at {BAUD_RATE} baud: {exc}')
            raise
        self.get_logger().info(f'Connected to {self.serial.port}')

    def read_serial(self):
        # Checks if any data on serial, if so, update the values
        pass

    def request_data(self, sensor_type: SensorType):
        # Sends a request to the Arduino to get some sensor reading
        self.serial.write(REQUEST_MSGS[sensor_type])

def main(args=None):
    """Run the communications node until shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = CommsNode()
        rclpy.spin(node)
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
