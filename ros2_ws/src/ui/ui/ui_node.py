# Start the UI: ros2 run ui ui_node
"""FireFlight rqt panel with ROS sensor and motor topics."""

from collections import deque
import math
from threading import Lock
import time

from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtGui import QIntValidator
from python_qt_binding.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QLineEdit, QPushButton, QSizePolicy,
    QSlider, QVBoxLayout, QWidget,
)
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import Bool, String, Int32, Float32
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from comms.types import SensorType, MotorType

UPDATE_PERIOD = 1.0  # Seconds between UI refreshes.
EST_DATA_FREQ = 5.0  # Hz
EST_DATA_PERIOD = 1.0 / EST_DATA_FREQ
HISTORY_NOMINAL_LEN = round(60 * EST_DATA_FREQ)  # One minute of samples.
MIN_CMD_DELAY = 0.2 # Seconds before sending a new command to the motor
SLOT_RPM_MAX = 4095
DC_ENCODER_CPR = 90.0  # Matches smcl_integrated's rising-edge encoder counter.

ADC_FULL_SCALE = 1023.0

# Match the beta-model conversion in smcl/thermistor.ino: 10 kΩ at 25 °C
# with a 10 kΩ pull-up and the thermistor from the ADC pin to ground.
THERMISTOR_R25 = 10000.0
THERMISTOR_SERIES = 10000.0
THERMISTOR_T0_K = 298.15
THERMISTOR_BETA = 3220.0

# LV-MaxSonar-EZ analog output is (Vcc/512) volts per inch. A 10-bit ADC
# using that same Vcc therefore has 1024/512 = 2 counts per inch.
ULTRASONIC_COUNTS_PER_INCH = 1024.0 / 512.0
INCH_TO_METER = 0.0254
# The sensor cannot range closer than 6 in or farther than 254 in; it holds
# its output at those limits instead, so a reading AT a limit is out of range.
ULTRASONIC_MIN_INCHES = 6.0
ULTRASONIC_MAX_INCHES = 254.0
ULTRASONIC_MIN_METERS = ULTRASONIC_MIN_INCHES * INCH_TO_METER
ULTRASONIC_MAX_METERS = ULTRASONIC_MAX_INCHES * INCH_TO_METER
ULTRASONIC_EMA_ALPHA = 2.0 / (10 + 1)  # 10-sample EMA of converted meters only.

# VL53L1X reports ranging distance in millimeters. Long mode reaches 4 m, and
# the datasheet only ensures ranging down to 4 cm.
TOF_MIN_MM = 40
TOF_MAX_MM = 4000
TOF_MIN_METERS = TOF_MIN_MM / 1000.0
TOF_MAX_METERS = TOF_MAX_MM / 1000.0

def thermistor_celsius(raw):
    """Convert a 10-bit ADC count using the Arduino thermistor beta model."""
    if raw <= 0 or raw >= ADC_FULL_SCALE:
        return None
    resistance = THERMISTOR_SERIES * raw / (ADC_FULL_SCALE - raw)
    log_ratio = math.log(resistance / THERMISTOR_R25)
    kelvin = 1.0 / (
        1.0 / THERMISTOR_T0_K + log_ratio / THERMISTOR_BETA
    )
    return kelvin - 273.15

def ultrasonic_invalid_reason(raw):
    """Return why a MaxBotix count is out of range, or None when it is valid."""
    inches = raw / ULTRASONIC_COUNTS_PER_INCH
    if inches <= ULTRASONIC_MIN_INCHES:
        return f'< {ULTRASONIC_MIN_METERS:.2f} m'
    if inches >= ULTRASONIC_MAX_INCHES:
        return f'> {ULTRASONIC_MAX_METERS:.2f} m'
    return None

def ultrasonic_meters(raw):
    """Convert a 10-bit MaxBotix analog count to meters; None if out of range."""
    if ultrasonic_invalid_reason(raw) is not None:
        return None
    inches = raw / ULTRASONIC_COUNTS_PER_INCH
    return inches * INCH_TO_METER

def tof_invalid_reason(millimeters):
    """Return why a VL53L1X result is out of range, or None when it is valid."""
    if millimeters < 0:
        return 'no reading'  # smcl_integrated sends -1 for a bad range status.
    if millimeters < TOF_MIN_MM:
        return f'< {TOF_MIN_METERS:.2f} m'
    if millimeters > TOF_MAX_MM:
        return f'> {TOF_MAX_METERS:.2f} m'
    return None

def tof_meters(millimeters):
    """Convert a VL53L1X result, in millimeters, to meters; None if out of range."""
    if tof_invalid_reason(millimeters) is not None:
        return None
    return millimeters / 1000.0


class SmokeFusion:
    """Mirror FusionStop's filters, scene confirmation, and threshold hysteresis."""

    STOP_MM = 300
    RESTART_MM = 400
    CONFIRM_N = 3
    US_ALPHA = 0.3
    TOF_ALPHA = 0.4

    def __init__(self):
        self.us_filtered_mm = None
        self.tof_filtered_mm = None
        self.tof_valid = False
        self.candidate = 'CLEAR'
        self.confirmed = 'CLEAR'
        self.streak = 0
        self.stopped = False  # Classification latch only; never commands a motor.
        self._fresh = set()

    def update(self, sensor, raw):
        if sensor == SensorType.ULTRASONIC:
            millimeters = raw / ULTRASONIC_COUNTS_PER_INCH * 25.4
            previous = self.us_filtered_mm
            self.us_filtered_mm = (millimeters if previous is None else
                                   self.US_ALPHA * millimeters + (1 - self.US_ALPHA) * previous)
        elif sensor == SensorType.TOF:
            # The serial protocol has no range-status field; negative means no echo.
            self.tof_valid = raw >= 0
            if self.tof_valid:
                previous = self.tof_filtered_mm
                self.tof_filtered_mm = (raw if previous is None else
                                        self.TOF_ALPHA * raw + (1 - self.TOF_ALPHA) * previous)
        else:
            return
        self._fresh.add(sensor)
        # Count fresh pairs, rather than counting the same samples on UI refreshes.
        if len(self._fresh) < 2:
            return
        self._fresh.clear()
        threshold = self.RESTART_MM if self.stopped else self.STOP_MM
        us_near = int(self.us_filtered_mm) < threshold
        tof_near = (self.tof_valid and self.tof_filtered_mm is not None
                    and 0 <= int(self.tof_filtered_mm) < threshold)
        if us_near:
            scene = 'SOLID' if tof_near else 'TRANSPARENT'
        else:
            scene = 'SMOKE' if tof_near else 'CLEAR'
        if scene == self.candidate:
            self.streak = min(self.streak + 1, self.CONFIRM_N)
        else:
            self.candidate = scene
            self.streak = 1
        if self.streak >= self.CONFIRM_N:
            self.confirmed = self.candidate
        self.stopped = self.confirmed in ('SOLID', 'TRANSPARENT')


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
            if self.derived_line is not None:
                # Twin axes draw as a unit; put measurements above the raw trace.
                self.axes.set_zorder(self.raw_axes.get_zorder() + 1)
                self.axes.patch.set_visible(False)
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


class DcMotorChart(SensorChart):
    """Measured shaft position/RPM, with the current command on each axis."""

    def __init__(self):
        super().__init__('Position (°)', 'RPM')
        self.raw_line.set_linestyle('-')
        self.derived_line.set_label('Position')
        self.raw_line.set_label('RPM')
        self.position_setpoint = self.axes.axhline(
            0, color='tab:blue', linestyle=':', label='Position target')
        self.rpm_setpoint = self.raw_axes.axhline(
            0, color='tab:orange', linestyle=':', label='RPM target')
        self.axes.legend(handles=[self.derived_line, self.position_setpoint,
                                  self.raw_line, self.rpm_setpoint],
                         fontsize=5, loc='upper left', ncol=2)

    def set_setpoints(self, position, rpm):
        self.position_setpoint.set_ydata([position, position])
        self.rpm_setpoint.set_ydata([rpm, rpm])
        for axes in (self.axes, self.raw_axes):
            axes.relim()
            axes.autoscale_view(scalex=False)
        self.draw_idle()

    def set_history(self, history):
        # SensorChart expects [time, right-axis value, left-axis value].
        super().set_history([(stamp, rpm, position) for stamp, position, rpm in history])


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
        self.motor_labels = {}
        self.motor_slider_labels = {}
        self.motor_sliders = {}
        self.motor_position_entries = {}
        self.motor_send_buttons = {}
        self.smoke_fusion = SmokeFusion()
        self._dc_position_deg = 0.0
        self._ultrasonic_filtered_m = None
        self._stepper_requested_deg = None
        self._history_lock = Lock()

        # region Values
        self.sensor_values = {
            SensorType.POTENTIOMETER: None,
            SensorType.THERMISTOR: None,
            SensorType.ULTRASONIC: None,
            SensorType.SLOT: None,
            SensorType.TOF: None,
            SensorType.DC_ENCODER: 0,
            SensorType.DC_RPM: 0.0,
            SensorType.DES_RPM: None,
            SensorType.UI_MODE: None,
            SensorType.STEPPER_MOVED: None,
        }

        self.data_history = {
            SensorType.POTENTIOMETER: self._history(), # [timestamp, rawValue, percent]
            SensorType.THERMISTOR: self._history(), # [timestamp, raw ADC, celsius]
            SensorType.ULTRASONIC: self._history(), # [timestamp, raw ADC, meters]
            SensorType.SLOT: self._history(), # [timestamp, RPM from Arduino]
            SensorType.TOF: self._history(), # [timestamp, millimeters, meters]
            MotorType.SERVO: self._history(), # [timestamp, position]
            MotorType.STEPPER: self._history(), # [timestamp, position]
            SensorType.DC_ENCODER: self._history(), # [timestamp, position degrees, RPM]
        }

        self.last_motor_command_times = {
            MotorType.SERVO: 0.0,
            MotorType.STEPPER: 0.0,
            MotorType.DC: 0.0,
        }

        #endregion Values

        if node is not None:
            # region Publishers and Subscribers
            self.motor_command_pub = node.create_publisher(String, 'motor_commands', 10)

            self.pot_sub = node.create_subscription(Int32, 'potentiometer', self.pot_callback, 10)
            self.thermistor_sub = node.create_subscription(Int32, 'thermistor', self.thermistor_callback, 10)
            self.ultrasonic_sub = node.create_subscription(Int32, 'ultrasonic', self.ultrasonic_callback, 10)
            self.slot_sub = node.create_subscription(Int32, 'slot', self.slot_callback, 10)
            self.tof_sub = node.create_subscription(Int32, 'tof', self.tof_callback, 10)

            self.dc_encoder_sub = node.create_subscription(Int32, 'dc_encoder', self.dc_encoder_callback, 10)
            self.dc_rpm_sub = node.create_subscription(Float32, 'dc_rpm', self.dc_rpm_callback, 10)
            self.des_rpm_sub = node.create_subscription(Float32, 'des_rpm', self.des_rpm_callback, 10)
            self.ui_mode_sub = node.create_subscription(Bool, 'ui_mode', self.ui_mode_callback, 10)
            self.stepper_moved_sub = node.create_subscription(Float32, 'stepper_moved', self.stepper_moved_callback, 10)
            # Stepper and servo are technically open-loop, so no subs for these
            # endregion Publishers and Subscribers

            self._subscriptions = [self.pot_sub, self.thermistor_sub,
                self.ultrasonic_sub, self.slot_sub, self.tof_sub, self.dc_encoder_sub,
                self.dc_rpm_sub, self.des_rpm_sub, self.ui_mode_sub, self.stepper_moved_sub]

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
            'Servo', 'Stepper', 'DC', 'Smoke detection',
        ]), 1)
        self.sensor_history_updated.connect(self._refresh_sensor_chart, Qt.QueuedConnection)
        # endregion UI Setup
        # rqt receives messages on worker threads; update widgets on Qt's thread.
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self.update_ui)
        self._update_timer.start(round(UPDATE_PERIOD * 1000))
    # region Publisher and Subscriber Functions
    def send_motor_command(self, motor_type: MotorType, value: int,
                           mode='P', force=False):
        """Publish the serial protocol, silently dropping commands sent too soon."""
        if self.motor_command_pub is None:
            return False
        if motor_type == MotorType.SERVO:
            if not 0 <= value <= 90:
                return False
            command = f'SER:{value}\n'
        else:
            maximum = 60 if motor_type == MotorType.DC and mode == 'V' else 32767
            if abs(value) > maximum or mode not in ('P', 'V'):
                return False
            if motor_type == MotorType.DC and mode == 'V' and value < 0:
                return False
            direction = '+' if value >= 0 else '-'
            if motor_type == MotorType.STEPPER:
                command = f'STP:{direction}:{abs(value)}\n'
            elif motor_type == MotorType.DC:
                command = f'DCM:{mode}:{direction}:{abs(value)}\n'
            else:
                return False
        now = time.monotonic()
        last_sent = self.last_motor_command_times[motor_type]
        if not force and last_sent and now - last_sent < MIN_CMD_DELAY:
            return False
        msg = String()
        msg.data = command
        self.motor_command_pub.publish(msg)
        self.last_motor_command_times[motor_type] = now
        if motor_type == MotorType.DC:
            self.sensor_charts[SensorType.DC_ENCODER].set_setpoints(
                value if mode == 'P' else 0, value if mode == 'V' else 0)
            self._dc_position_deg = 0.0
            self.sensor_values[SensorType.DC_ENCODER] = 0
            self._record(SensorType.DC_ENCODER, 0.0, self.sensor_values[SensorType.DC_RPM])
        return True

    def _send_motor_position(self, motor):
        entry = self.motor_position_entries[motor]
        if entry.hasAcceptableInput():
            position = int(entry.text())
            if motor == MotorType.STEPPER and self.sensor_values[SensorType.UI_MODE] is False:
                self.motor_labels[motor].setText('Switch to UI control first')
                return
            if self.send_motor_command(motor, position):
                if motor == MotorType.STEPPER:
                    self._stepper_requested_deg = position
                    self.sensor_values[SensorType.STEPPER_MOVED] = None
                    self.motor_labels[motor].setText(f'Requested {position}° | Moved —')
                    return
                self.motor_labels[motor].setText(f'POS: {position}°')
                if motor == MotorType.DC:
                    slider = self.motor_sliders[motor]
                    blocked = slider.blockSignals(True)
                    try:
                        slider.setValue(0)
                    finally:
                        slider.blockSignals(blocked)
                    self.motor_slider_labels[motor].setText('RPM: 0')

    def _send_motor_slider(self, motor, force=False):
        value = self.motor_sliders[motor].value()
        label = f'POS: {value}°' if motor == MotorType.SERVO else f'RPM: {value}'
        self.motor_slider_labels[motor].setText(label)
        sent = self.send_motor_command(motor, value, mode='V', force=force)
        if sent and motor == MotorType.DC:
            self.motor_position_entries[motor].setText('0')
            self.motor_labels[motor].setText('POS: 0°')

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
        if sensor in (SensorType.ULTRASONIC, SensorType.TOF):
            self.smoke_fusion.update(sensor, history[-1][1])
            self._update_detection_status()

    def _update_detection_status(self):
        scene = self.smoke_fusion.confirmed
        smoke = 'SMOKE DETECTED' if scene == 'SMOKE' else 'NO SMOKE DETECTED'
        transparent = ('TRANSPARENT OBJECT DETECTED' if scene == 'TRANSPARENT'
                       else 'NO TRANSPARENT OBJECT DETECTED')
        self.smoke_label.setText(f'{smoke} | {transparent}')

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
        meters = ultrasonic_meters(raw)
        if meters is None or self._ultrasonic_filtered_m is None:
            # Seed from the first valid sample; reset across invalid readings.
            self._ultrasonic_filtered_m = meters
        else:
            self._ultrasonic_filtered_m += ULTRASONIC_EMA_ALPHA * (
                meters - self._ultrasonic_filtered_m)
        self._record(SensorType.ULTRASONIC, raw, self._ultrasonic_filtered_m)

    def slot_callback(self, msg: Int32):
        rpm = max(0, min(SLOT_RPM_MAX, msg.data))
        self.sensor_values[SensorType.SLOT] = rpm
        self._record(SensorType.SLOT, rpm)

    def tof_callback(self, msg: Int32):
        millimeters = msg.data
        self.sensor_values[SensorType.TOF] = millimeters
        self._record(SensorType.TOF, millimeters, tof_meters(millimeters))

    def dc_encoder_callback(self, msg: Int32):
        self.sensor_values[SensorType.DC_ENCODER] = msg.data
        self._dc_position_deg = msg.data * 360.0 / DC_ENCODER_CPR
        self._record(SensorType.DC_ENCODER, self._dc_position_deg,
                     self.sensor_values[SensorType.DC_RPM])

    def dc_rpm_callback(self, msg: Float32):
        self.sensor_values[SensorType.DC_RPM] = msg.data
        self._record(SensorType.DC_ENCODER, self._dc_position_deg, msg.data)

    def des_rpm_callback(self, msg: Float32):
        self.sensor_values[SensorType.DES_RPM] = msg.data

    def stepper_moved_callback(self, msg: Float32):
        self.sensor_values[SensorType.STEPPER_MOVED] = msg.data

    def ui_mode_callback(self, msg: Bool):
        self.sensor_values[SensorType.UI_MODE] = msg.data

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
        self.update_ui_mode_ui()

    def update_ui_mode_ui(self):
        ui_mode = self.sensor_values[SensorType.UI_MODE]
        if ui_mode is not None:
            self.control_mode_label.setText('UI Control' if ui_mode else 'Sensor Control')
    
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
            self._update_range_display(
                SensorType.ULTRASONIC, self._ultrasonic_filtered_m,
                ultrasonic_invalid_reason(raw), ULTRASONIC_MAX_METERS)

    def update_slot_ui(self):
        rpm = self.sensor_values[SensorType.SLOT]
        if rpm is not None:
            self.sensor_labels[SensorType.SLOT].setText(f'RPM: {rpm}')

    def update_tof_ui(self):
        millimeters = self.sensor_values[SensorType.TOF]
        if millimeters is not None:
            self._update_range_display(
                SensorType.TOF, tof_meters(millimeters),
                tof_invalid_reason(millimeters), TOF_MAX_METERS)

    def _update_range_display(self, sensor, meters, invalid_reason, maximum):
        """Show a distance, or INVALID when it is outside the sensor's range."""
        if invalid_reason is None:
            self._update_sensor_display(sensor, meters, f'{meters:.2f} m', maximum)
            return
        self.sensor_labels[sensor].setText(f'Value: INVALID ({invalid_reason})')
        slider = self.sensor_sliders[sensor]
        slider.setValue(slider.maximum() if invalid_reason.startswith('>') else 0)

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
        moved = self.sensor_values[SensorType.STEPPER_MOVED]
        if self._stepper_requested_deg is None or moved is None:
            return
        self.motor_labels[MotorType.STEPPER].setText(
            f'Requested {self._stepper_requested_deg}° | Moved {moved:.3f}°')

    def update_dc_ui(self):
        rpm = self.sensor_values[SensorType.DES_RPM]
        if self.sensor_values[SensorType.UI_MODE] is False and rpm is not None:
            # The refresh timer runs on Qt's thread; ROS callbacks only store data.
            self.sensor_charts[SensorType.DC_ENCODER].set_setpoints(0, rpm)

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
            if text == 'Smoke detection':
                row = QWidget()
                row_layout = QHBoxLayout(row)
                row_layout.setContentsMargins(0, 0, 0, 0)
                row_layout.setSpacing(12)
                self.smoke_label = QLabel()
                self._update_detection_status()
                self.smoke_label.setAccessibleName('Smoke and transparent object detection status')
                self.smoke_label.setWordWrap(True)
                self.control_mode_label = QLabel('—')
                self.control_mode_label.setAccessibleName('Control mode status')
                for title, value in ((None, self.smoke_label),
                                     ('Control Mode', self.control_mode_label)):
                    status_cell = QFrame()
                    status_cell.setProperty('uiCell', True)
                    status_cell.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
                    status_layout = QVBoxLayout(status_cell)
                    status_layout.setContentsMargins(12, 10, 12, 10)
                    if title:
                        status_layout.addWidget(QLabel(title))
                    value.setAlignment(Qt.AlignCenter)
                    status_layout.addWidget(value, 1)
                    row_layout.addWidget(status_cell, 1)
                layout.addWidget(row, 1)
                continue
            cell = QFrame()
            cell.setProperty('uiCell', True)
            cell.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Ignored)
            contents = QHBoxLayout(cell)
            contents.setContentsMargins(12, 10, 12, 10)
            contents.setSpacing(12)
            if sensors:
                self._sensor_contents(contents, text)
            else:
                self._motor_contents(contents, text)
            layout.addWidget(cell, 2)
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
            value = QLabel('RPM: —')
            value.setProperty('valuePlaceholder', True)
            value.setAlignment(Qt.AlignCenter)
            controls_layout.addWidget(value)
            self.sensor_labels[sensor] = value
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
            SensorType.SLOT: ('RPM', None),
            SensorType.TOF: ('Distance (m)', 'Raw (mm)'),
        }[sensor]
        self.sensor_charts[sensor] = SensorChart(*chart)
        self.sensor_charts[sensor].setAccessibleName(f'{name} history graph')
        layout.addWidget(self.sensor_charts[sensor], 3)

    def _motor_contents(self, layout, name):
        motor = {'Servo': MotorType.SERVO, 'Stepper': MotorType.STEPPER,
                 'DC': MotorType.DC}[name]
        controls = QWidget()
        controls.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        contents = QVBoxLayout(controls)
        contents.setContentsMargins(0, 0, 0, 0)
        contents.setSpacing(6)
        title = QLabel(name)
        title.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        contents.addWidget(title)
        contents.addStretch(1)

        if name in ('Stepper', 'DC'):
            position_row = QHBoxLayout()
            position = QLabel('POS: —')
            position.setProperty('valuePlaceholder', True)
            self.motor_labels[motor] = position
            position_row.addWidget(position)
            entry = QLineEdit()
            entry.setAccessibleName(f'{name} position command')
            entry.setPlaceholderText('Relative rotation (°)' if name == 'DC' else 'Position (°)')
            entry.setValidator(QIntValidator(-32767, 32767, entry))
            entry.setMinimumWidth(0)
            position_row.addWidget(entry, 1)
            button = QPushButton('Send')
            button.setAccessibleName(f'Send {name} position command')
            button.clicked.connect(lambda checked=False: self._send_motor_position(motor))
            entry.returnPressed.connect(lambda: self._send_motor_position(motor))
            self.motor_position_entries[motor] = entry
            self.motor_send_buttons[motor] = button
            position_row.addWidget(button)
            contents.addLayout(position_row)
            contents.addStretch(1)

        if name == 'Stepper':
            layout.addWidget(controls, 1)
            return

        value = QLabel('POS: —' if name == 'Servo' else 'RPM: —')
        value.setProperty('valuePlaceholder', True)
        value.setAlignment(Qt.AlignCenter)
        value.setTextInteractionFlags(Qt.TextSelectableByMouse)
        contents.addWidget(value)
        self.motor_slider_labels[motor] = value
        if name == 'Servo':
            self.motor_labels[motor] = value

        slider = QSlider(Qt.Horizontal)
        slider.setAccessibleName(f'{name} slider')
        minimum, maximum = (0, 90) if name == 'Servo' else (0, 60)
        slider.setRange(minimum, maximum)
        slider.setValue(0)
        self.motor_sliders[motor] = slider
        slider.valueChanged.connect(lambda _: self._send_motor_slider(motor))
        slider.sliderReleased.connect(lambda: self._send_motor_slider(motor, force=True))
        contents.addWidget(slider)
        endpoints = QHBoxLayout()
        bounds = ('0°', '90°') if name == 'Servo' else ('0 RPM', '60 RPM')
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
            chart = DcMotorChart()
            chart.setAccessibleName('DC position and RPM history graph')
            self.sensor_charts[SensorType.DC_ENCODER] = chart
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
