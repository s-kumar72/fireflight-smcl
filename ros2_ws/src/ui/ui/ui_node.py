# Start the UI: ros2 run ui ui_node
"""FireFlight rqt panel with ROS sensor and motor topics."""

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QLineEdit, QPushButton, QSizePolicy,
    QSlider, QVBoxLayout, QWidget,
)
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import String, Int32, Bool

from comms.types import SensorType, MotorType


class UiWidget(QWidget):
    """Display sensor controls, chart placeholders, and motor cells."""

    def __init__(self, node=None):
        super().__init__()
        self._node = node
        self._subscriptions = []
        self.motor_command_pub = None

        # region Values
        self.sensor_values = {
            SensorType.POTENTIOMETER: 0,
            SensorType.THERMISTOR: 0,
            SensorType.ULTRASONIC: 0,
            SensorType.SLOT: False,
            SensorType.TOF: 0,
            SensorType.DC_ENCODER: 0,
        }

        if node is not None:
            # region Publishers and Subscribers
            self.motor_command_pub = node.create_publisher(String, 'motor_commands', 10)

            self.pot_sub = node.create_subscription(Int32, 'potentiometer', self.pot_callback, 10)
            self.thermistor_sub = node.create_subscription(Int32, 'thermistor', self.thermistor_callback, 10)
            self.ultrasonic_sub = node.create_subscription(Int32, 'ultrasonic', self.ultrasonic_callback, 10)
            self.slot_sub = node.create_subscription(Bool, 'slot', self.slot_callback, 10)
            self.tof_sub = node.create_subscription(Int32, 'tof', self.tof_callback, 10)

            self.dc_encoder_sub = node.create_subscription(Int32, 'dc_encoder', self.dc_encoder_callback, 10)
            # Stepper and servo are technically open-loop, so no subs for these
            # endregion Publishers and Subscribers

            self._subscriptions = [self.pot_sub, self.thermistor_sub,
                self.ultrasonic_sub, self.slot_sub, self.tof_sub, self.dc_encoder_sub]

        # region UI Setup
        self.setObjectName('FireFlightUi')
        self.setWindowTitle('FireFlight')
        self.resize(1000, 750)
        self.setStyleSheet('''
            QWidget#FireFlightUi { background-color: #f1f3f5; }
            QFrame[uiCell="true"] {
                background-color: #ffffff;
                border: 1px solid #89939e;
                border-radius: 4px;
            }
            QLabel {
                color: #263238;
                font-size: 15px;
                font-weight: bold;
                background: transparent;
                border: none;
            }
            QFrame[chartPlaceholder="true"] {
                background-color: #f8fafc;
                border: 1px dashed #aab4bf;
                border-radius: 3px;
            }
            QLabel[valuePlaceholder="true"] {
                font-size: 18px;
            }
        ''')
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)
        layout.addWidget(self._column([
            'Potentiometer', 'Thermistor', 'Ultrasonic', 'Slot',
            'Time-of-Flight',
        ], sensors=True), 1)
        layout.addWidget(self._column(['Servo', 'Stepper', 'DC']), 1)
        # endregion UI Setup
    # region Publisher and Subscriber Functions
    def send_motor_command(self, motor_type: MotorType, command: str):
        if self.motor_command_pub is None:
            raise RuntimeError("Motor commands require a ROS node")
        msg = String()
        msg.data = f"{motor_type.value}:{command}"
        self.motor_command_pub.publish(msg)

    def pot_callback(self, msg: Int32):
        self.sensor_values[SensorType.POTENTIOMETER] = msg.data

    def thermistor_callback(self, msg: Int32):
        self.sensor_values[SensorType.THERMISTOR] = msg.data

    def ultrasonic_callback(self, msg: Int32):
        self.sensor_values[SensorType.ULTRASONIC] = msg.data

    def slot_callback(self, msg: Bool):
        self.sensor_values[SensorType.SLOT] = msg.data

    def tof_callback(self, msg: Int32):
        self.sensor_values[SensorType.TOF] = msg.data

    def dc_encoder_callback(self, msg: Int32):
        self.sensor_values[SensorType.DC_ENCODER] = msg.data

    # endregion Subscriber Callbacks

    # region Update Functions
    def update_ui(self):
        self.update_pot_ui()
        self.update_thermistor_ui()
        self.update_ultrasonic_ui()
        self.update_slot_ui()
        self.update_tof_ui()
        self.update_dc_encoder_ui()
        self.update_servo_ui()
        self.update_stepper_ui()
        self.update_dc_ui()
    
    def update_pot_ui(self):
        pass

    def update_thermistor_ui(self):
        pass

    def update_ultrasonic_ui(self):
        pass

    def update_slot_ui(self):
        pass

    def update_tof_ui(self):
        pass

    def update_dc_encoder_ui(self):
        pass

    def update_servo_ui(self):
        pass

    def update_stepper_ui(self):
        pass

    def update_dc_ui(self):
        pass

    # endregion Update Functions

    def shutdown(self):
        """Release this panel's ROS entities; rqt owns the node."""
        if self._node is not None:
            for subscription in self._subscriptions:
                self._node.destroy_subscription(subscription)
            self._subscriptions.clear()
            if self.motor_command_pub is not None:
                self._node.destroy_publisher(self.motor_command_pub)
                self.motor_command_pub = None

    # region UI Functions
    @staticmethod
    def _column(labels, sensors=False):
        column = QWidget()
        column.setMinimumWidth(0)
        column.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        layout = QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        for text in labels:
            cell = QFrame()
            cell.setProperty('uiCell', True)
            cell.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Ignored)
            contents = QHBoxLayout(cell)
            contents.setContentsMargins(12, 10, 12, 10)
            contents.setSpacing(12)
            if sensors:
                UiWidget._sensor_contents(contents, text)
            else:
                UiWidget._motor_contents(contents, text)
            layout.addWidget(cell, 1)
        return column

    @staticmethod
    def _sensor_contents(layout, name):
        controls = QWidget()
        controls.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        controls_layout = QVBoxLayout(controls)
        controls_layout.setContentsMargins(0, 0, 0, 0)
        controls_layout.setSpacing(6)
        title = QLabel(name)
        title.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        header = QHBoxLayout()
        header.addWidget(title)
        header.addStretch(1)
        controls_layout.addLayout(header)
        controls_layout.addStretch(1)

        if name != 'Slot':
            value = QLabel('Value: —')
            value.setProperty('valuePlaceholder', True)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value.setAlignment(Qt.AlignTop | Qt.AlignRight)
            header.addWidget(value)
            slider = QSlider(Qt.Horizontal)
            slider.setAccessibleName(f'{name} slider')
            slider.setRange(0, 100)
            slider.setValue(0)
            slider.setEnabled(False)
            controls_layout.addWidget(slider)
            endpoints = QHBoxLayout()
            for text, alignment in [('a', Qt.AlignLeft), ('b', Qt.AlignRight)]:
                endpoint = QLabel(text)
                endpoint.setAlignment(alignment)
                endpoint.setStyleSheet('font-size: 15px; font-weight: normal;')
                endpoints.addWidget(endpoint)
            controls_layout.addLayout(endpoints)

        controls_layout.addStretch(1)
        layout.addWidget(controls, 2)

        layout.addWidget(UiWidget._chart_placeholder(), 3)

    @staticmethod
    def _chart_placeholder():
        chart = QFrame()
        chart.setProperty('chartPlaceholder', True)
        chart.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        chart_layout = QVBoxLayout(chart)
        chart_label = QLabel('Chart placeholder')
        chart_label.setAlignment(Qt.AlignCenter)
        chart_layout.addWidget(chart_label)
        return chart

    @staticmethod
    def _motor_contents(layout, name):
        controls = QWidget()
        controls.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        contents = QVBoxLayout(controls)
        contents.setContentsMargins(0, 0, 0, 0)
        contents.setSpacing(6)
        title = QLabel(name)
        title.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        contents.addWidget(title)
        contents.addStretch(1)

        if name == 'DC':
            position_row = QHBoxLayout()
            position = QLabel('POS: —')
            position.setProperty('valuePlaceholder', True)
            position_row.addWidget(position)
            entry = QLineEdit()
            entry.setAccessibleName('DC position command')
            entry.setPlaceholderText('Position')
            entry.setMinimumWidth(0)
            position_row.addWidget(entry, 1)
            position_row.addWidget(QPushButton('Send'))
            contents.addLayout(position_row)
            contents.addStretch(1)

        value = QLabel('POS: —' if name == 'Servo' else 'RPM: —')
        value.setProperty('valuePlaceholder', True)
        value.setAlignment(Qt.AlignCenter)
        value.setTextInteractionFlags(Qt.TextSelectableByMouse)
        contents.addWidget(value)

        slider = QSlider(Qt.Horizontal)
        slider.setAccessibleName(f'{name} slider')
        slider.setRange(0, 180 if name == 'Servo' else 100)
        contents.addWidget(slider)
        endpoints = QHBoxLayout()
        bounds = ('0', '180°') if name == 'Servo' else ('a', 'b')
        for text, alignment in zip(bounds, (Qt.AlignLeft, Qt.AlignRight)):
            endpoint = QLabel(text)
            endpoint.setAlignment(alignment)
            endpoint.setStyleSheet('font-size: 15px; font-weight: normal;')
            endpoints.addWidget(endpoint)
        contents.addLayout(endpoints)
        if name != 'DC':
            contents.addStretch(1)

        layout.addWidget(controls, 1)
        layout.addWidget(UiWidget._chart_placeholder(), 1)
    # endregion UI Functions


class UiPlugin(Plugin):
    """Expose the panel using rqt's managed ROS node."""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('FireFlightUiPlugin')
        self.widget = UiWidget(context.node)
        context.add_widget(self.widget)
        # Wait until rqt has attached the panel to its main window.
        QTimer.singleShot(0, lambda: self.widget.window().showMaximized())


    def shutdown_plugin(self):
        self.widget.shutdown()

def main(args=None):
    """Launch this panel in its own rqt window."""
    import sys

    from rqt_gui.main import Main

    argv = sys.argv if args is None else [sys.argv[0], *args]
    return Main().main(argv=argv, standalone='ui.ui_node.UiPlugin')


if __name__ == '__main__':
    raise SystemExit(main())
