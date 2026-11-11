# Start the UI: ros2 run ui ui_node
"""FireFlight rqt panel with ROS sensor and motor topics."""

from collections import deque
import math
from threading import Lock
import time

from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QLineEdit, QPushButton, QSizePolicy,
    QSlider, QVBoxLayout, QWidget,
)
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import String, Int32, Bool
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from comms.types import SensorType, MotorType

UPDATE_PERIOD = 1.0  # Seconds between UI refreshes.
EST_DATA_FREQ = 5.0  # Hz
EST_DATA_PERIOD = 1.0 / EST_DATA_FREQ
HISTORY_NOMINAL_LEN = round(60 * EST_DATA_FREQ)  # One minute of samples.

ADC_FULL_SCALE = 1023.0

# NTCLE100E3103: 10 kΩ at 25 °C, B25/85 = 3977 K.
# Vishay 29049 formula (2), material A. The 10 kΩ divider is the pull-up;
# the thermistor is from the ADC pin to ground.
THERMISTOR_R25 = 10000.0
THERMISTOR_SERIES = 10000.0
THERMISTOR_A1 = 3.354016e-3
THERMISTOR_B1 = 2.569850e-4
THERMISTOR_C1 = 2.620131e-6
THERMISTOR_D1 = 6.383091e-8

# LV-MaxSonar-EZ analog output is (Vcc/512) volts per inch. A 10-bit ADC
# using that same Vcc therefore has 1024/512 = 2 counts per inch.
ULTRASONIC_COUNTS_PER_INCH = 1024.0 / 512.0
INCH_TO_METER = 0.0254
ULTRASONIC_MAX_METERS = 254.0 * INCH_TO_METER

# VL53L1X reports ranging distance in millimeters. Long mode reaches 4 m.
TOF_MAX_METERS = 4.0

def thermistor_celsius(raw):
    """Convert a 10-bit ADC count to Celsius for the 10 kΩ NTCLE100."""
    if raw <= 0 or raw >= ADC_FULL_SCALE:
        return None
    resistance = THERMISTOR_SERIES * raw / (ADC_FULL_SCALE - raw)
    log_ratio = math.log(resistance / THERMISTOR_R25)
    kelvin = 1.0 / (
        THERMISTOR_A1
        + THERMISTOR_B1 * log_ratio
        + THERMISTOR_C1 * log_ratio ** 2
        + THERMISTOR_D1 * log_ratio ** 3
    )
    return kelvin - 273.15

def ultrasonic_meters(raw):
    """Convert a 10-bit MaxBotix analog count to meters."""
    inches = raw / ULTRASONIC_COUNTS_PER_INCH
    return inches * INCH_TO_METER

def tof_meters(millimeters):
    """Convert a VL53L1X ranging result, in millimeters, to meters."""
    return millimeters / 1000.0

class SensorChart(FigureCanvasQTAgg):
    """Plot a history snapshot with derived and raw values on separate axes."""

    def __init__(self, derived_label=None, raw_label=None, binary=False):
        figure = Figure(figsize=(3, 1.2), dpi=100)
        super().__init__(figure)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        self.setMinimumSize(0, 90)
        figure.subplots_adjust(left=0.23, right=0.77, bottom=0.32, top=0.94)
        self.axes = figure.add_subplot(111)
        self.axes.set_xlabel('Time relative to latest sample (s)', fontsize=6)
        self.axes.tick_params(labelsize=6)
        self.axes.grid(True, alpha=0.25)
        self.derived_line = None
        self.raw_axes = None
        self.raw_line = None
        if derived_label:
            self.axes.set_ylabel(derived_label, fontsize=7, color='tab:blue')
            self.axes.tick_params(axis='y', colors='tab:blue')
            self.derived_line, = self.axes.plot(
                [], [], color='tab:blue', marker='.', markersize=3,
                drawstyle='steps-post' if binary else 'default')
        else:
            self.axes.yaxis.set_visible(False)
        if binary:
            self.axes.set_yticks([0, 1], labels=['Clear', 'Blocked'])
            self.axes.set_ylim(-0.1, 1.1)
        self.binary = binary
        if raw_label:
            self.raw_axes = self.axes.twinx()
            self.raw_axes.set_ylabel(raw_label, fontsize=7, color='tab:orange')
            self.raw_axes.tick_params(labelsize=6, axis='y', colors='tab:orange')
            self.raw_line, = self.raw_axes.plot(
                [], [], color='tab:orange', linestyle='--', marker='.', markersize=3)
        self.axes.set_xlim(-1, 0)

    def set_history(self, history):
        if not history:
            return
        times = [sample[0] - history[-1][0] for sample in history]
        if self.derived_line is not None:
            index = 2 if self.raw_line is not None else 1
            self.derived_line.set_data(times, [
                float('nan') if sample[index] is None else sample[index]
                for sample in history])
            if not self.binary:
                self.axes.relim()
                self.axes.autoscale_view(scalex=False)
        if self.raw_line is not None:
            self.raw_line.set_data(times, [sample[1] for sample in history])
            self.raw_axes.relim()
            self.raw_axes.autoscale_view(scalex=False)
        self.axes.set_xlim(min(-1.0, times[0]), 0)
        self.draw_idle()


class UiWidget(QWidget):
    """Display sensor controls, history graphs, and motor cells."""

    sensor_history_updated = Signal(object, object)

    def __init__(self, node=None):
        super().__init__()
        self._node = node
        self._subscriptions = []
        self.motor_command_pub = None
        self.sensor_labels = {}
        self.sensor_sliders = {}
        self.sensor_charts = {}
        self.motor_charts = {}
        self._history_lock = Lock()

        # region Values
        self.sensor_values = {
            SensorType.POTENTIOMETER: None,
            SensorType.THERMISTOR: None,
            SensorType.ULTRASONIC: None,
            SensorType.SLOT: None,
            SensorType.TOF: None,
            SensorType.DC_ENCODER: 0,
        }

        self.data_history = {
            SensorType.POTENTIOMETER: self._history(), # [timestamp, rawValue, percent]
            SensorType.THERMISTOR: self._history(), # [timestamp, raw ADC, celsius]
            SensorType.ULTRASONIC: self._history(), # [timestamp, raw ADC, meters]
            SensorType.SLOT: self._history(), # [timestamp, isBlocked]
            SensorType.TOF: self._history(), # [timestamp, millimeters, meters]
            MotorType.SERVO: self._history(), # [timestamp, position]
            MotorType.STEPPER: self._history(), # [timestamp, rpm]
            SensorType.DC_ENCODER: self._history(), # [timestamp, position]
        }

        #endregion Values

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
        layout.addWidget(self._column([
            'Servo', 'Stepper', 'DC', 'Smoke detection placeholder',
        ]), 1)
        self.sensor_history_updated.connect(self._refresh_sensor_chart, Qt.QueuedConnection)
        # endregion UI Setup
        # rqt receives messages on worker threads; update widgets on Qt's thread.
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self.update_ui)
        self._update_timer.start(round(UPDATE_PERIOD * 1000))
    # region Publisher and Subscriber Functions
    def send_motor_command(self, motor_type: MotorType, command: str):
        if self.motor_command_pub is None:
            raise RuntimeError("Motor commands require a ROS node")
        msg = String()
        msg.data = f"{motor_type.value}:{command}"
        self.motor_command_pub.publish(msg)

    @staticmethod
    def _history():
        """Fixed-length buffer; appending past the end drops the oldest sample."""
        return deque(maxlen=HISTORY_NOMINAL_LEN)

    def _record(self, key, *fields):
        # Read the full history for every callback; only touch plots on Qt's thread.
        with self._history_lock:
            self.data_history[key].append((time.time(), *fields))
            history = list(self.data_history[key])
        self.sensor_history_updated.emit(key, history)

    def _refresh_sensor_chart(self, sensor, history):
        self.sensor_charts[sensor].set_history(history)

    def pot_callback(self, msg: Int32):
        raw = msg.data
        self.sensor_values[SensorType.POTENTIOMETER] = raw
        self._record(SensorType.POTENTIOMETER, raw, raw * 100.0 / 1023.0)

    def thermistor_callback(self, msg: Int32):
        raw = msg.data
        self.sensor_values[SensorType.THERMISTOR] = raw
        celsius = thermistor_celsius(raw)
        self._record(SensorType.THERMISTOR, raw, celsius)

    def ultrasonic_callback(self, msg: Int32):
        raw = msg.data
        self.sensor_values[SensorType.ULTRASONIC] = raw
        self._record(SensorType.ULTRASONIC, raw, ultrasonic_meters(raw))

    def slot_callback(self, msg: Bool):
        self.sensor_values[SensorType.SLOT] = msg.data
        self._record(SensorType.SLOT, msg.data)

    def tof_callback(self, msg: Int32):
        millimeters = msg.data
        self.sensor_values[SensorType.TOF] = millimeters
        self._record(SensorType.TOF, millimeters, tof_meters(millimeters))

    def dc_encoder_callback(self, msg: Int32):
        position = msg.data
        self.sensor_values[SensorType.DC_ENCODER] = position
        self._record(SensorType.DC_ENCODER, position)

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
        raw = self.sensor_values[SensorType.POTENTIOMETER]
        if raw is not None:
            percent = raw * 100.0 / 1023.0
            self._update_sensor_display(SensorType.POTENTIOMETER, percent,
                                        f'{percent:.1f}%', 100.0)

    def update_thermistor_ui(self):
        raw = self.sensor_values[SensorType.THERMISTOR]
        if raw is not None:
            celsius = thermistor_celsius(raw)
            if celsius is not None:
                self._update_sensor_display(SensorType.THERMISTOR, celsius,
                                            f'{celsius:.1f}°C', 50.0)

    def update_ultrasonic_ui(self):
        raw = self.sensor_values[SensorType.ULTRASONIC]
        if raw is not None:
            meters = ultrasonic_meters(raw)
            self._update_sensor_display(SensorType.ULTRASONIC, meters,
                                        f'{meters:.2f} m', ULTRASONIC_MAX_METERS)

    def update_slot_ui(self):
        blocked = self.sensor_values[SensorType.SLOT]
        if blocked is not None:
            self.sensor_labels[SensorType.SLOT].setText(
                'STATE: Blocked' if blocked else 'STATE: Clear')

    def update_tof_ui(self):
        millimeters = self.sensor_values[SensorType.TOF]
        if millimeters is not None:
            meters = tof_meters(millimeters)
            self._update_sensor_display(SensorType.TOF, meters,
                                        f'{meters:.2f} m', TOF_MAX_METERS)

    def _update_sensor_display(self, sensor, value, text, maximum):
        self.sensor_labels[sensor].setText(f'Value: {text}')
        slider = self.sensor_sliders[sensor]
        fraction = max(0.0, min(1.0, value / maximum))
        slider.setValue(round(fraction * slider.maximum()))

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
        self._update_timer.stop()
        if self._node is not None:
            for subscription in self._subscriptions:
                self._node.destroy_subscription(subscription)
            self._subscriptions.clear()
            if self.motor_command_pub is not None:
                self._node.destroy_publisher(self.motor_command_pub)
                self.motor_command_pub = None

    # region UI Functions
    def _column(self, labels, sensors=False):
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
                self._sensor_contents(contents, text)
            elif text == 'Smoke detection placeholder':
                label = QLabel(text)
                label.setAlignment(Qt.AlignCenter)
                contents.addWidget(label)
            else:
                self._motor_contents(contents, text)
            layout.addWidget(cell, 1 if text == 'Smoke detection placeholder' else 2)
        return column

    def _sensor_contents(self, layout, name):
        sensor = {
            'Potentiometer': SensorType.POTENTIOMETER,
            'Thermistor': SensorType.THERMISTOR,
            'Ultrasonic': SensorType.ULTRASONIC,
            'Slot': SensorType.SLOT,
            'Time-of-Flight': SensorType.TOF,
        }[name]
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

        if name == 'Slot':
            state = QLabel('STATE: —')
            state.setProperty('valuePlaceholder', True)
            state.setAlignment(Qt.AlignCenter)
            controls_layout.addWidget(state)
            self.sensor_labels[sensor] = state
        else:
            value = QLabel('Value: —')
            value.setProperty('valuePlaceholder', True)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value.setAlignment(Qt.AlignTop | Qt.AlignRight)
            header.addWidget(value)
            self.sensor_labels[sensor] = value
            slider = QSlider(Qt.Horizontal)
            slider.setAccessibleName(f'{name} slider')
            slider.setRange(0, 1000)
            slider.setValue(0)
            slider.setEnabled(False)
            self.sensor_sliders[sensor] = slider
            controls_layout.addWidget(slider)
            endpoints = QHBoxLayout()
            bounds = {
                'Potentiometer': ('0%', '100%'),
                'Thermistor': ('0°C', '50°C'),
                'Ultrasonic': ('0 m', '6.45 m'),
                'Time-of-Flight': ('0 m', '4 m'),
            }[name]
            for text, alignment in zip(bounds, (Qt.AlignLeft, Qt.AlignRight)):
                endpoint = QLabel(text)
                endpoint.setAlignment(alignment)
                endpoint.setStyleSheet('font-size: 15px; font-weight: normal;')
                endpoints.addWidget(endpoint)
            controls_layout.addLayout(endpoints)

        controls_layout.addStretch(1)
        layout.addWidget(controls, 2)

        chart = {
            SensorType.POTENTIOMETER: ('Position (%)', 'Raw ADC'),
            SensorType.THERMISTOR: ('Temperature (°C)', 'Raw ADC'),
            SensorType.ULTRASONIC: ('Distance (m)', 'Raw ADC'),
            SensorType.SLOT: ('Slot state', None),
            SensorType.TOF: ('Distance (m)', 'Raw (mm)'),
        }[sensor]
        self.sensor_charts[sensor] = SensorChart(
            *chart, binary=sensor == SensorType.SLOT)
        self.sensor_charts[sensor].setAccessibleName(f'{name} history graph')
        layout.addWidget(self.sensor_charts[sensor], 3)

    def _motor_contents(self, layout, name):
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
        bounds = ('0°', '180°') if name == 'Servo' else ('a', 'b')
        for text, alignment in zip(bounds, (Qt.AlignLeft, Qt.AlignRight)):
            endpoint = QLabel(text)
            endpoint.setAlignment(alignment)
            endpoint.setStyleSheet('font-size: 15px; font-weight: normal;')
            endpoints.addWidget(endpoint)
        contents.addLayout(endpoints)
        if name != 'DC':
            contents.addStretch(1)

        layout.addWidget(controls, 1)
        if name == 'DC':
            chart = SensorChart(raw_label='Encoder position (counts)')
            chart.setAccessibleName('DC encoder history graph')
            self.sensor_charts[SensorType.DC_ENCODER] = chart
            layout.addWidget(chart, 1)
        else:
            motor = MotorType.SERVO if name == 'Servo' else MotorType.STEPPER
            chart = SensorChart(
                derived_label='Position (°)' if name == 'Servo' else 'Speed (RPM)')
            if name == 'Servo':
                chart.axes.set_ylim(0, 180)
            chart.setAccessibleName(f'{name} history graph')
            self.motor_charts[motor] = chart
            layout.addWidget(chart, 1)
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
