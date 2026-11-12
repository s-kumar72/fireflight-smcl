"""Exercise motor controls without a ROS node or connected hardware."""

import unittest
from unittest.mock import Mock, patch

from python_qt_binding.QtWidgets import QApplication

from comms.types import MotorType
from ui.ui_node import MIN_CMD_DELAY, UiWidget


class MotorControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.widget = UiWidget()
        self.widget._update_timer.stop()
        self.publisher = Mock()
        self.widget.motor_command_pub = self.publisher
        self.clock = patch('ui.ui_node.time.monotonic', return_value=100.0)
        self.now = self.clock.start()

    def tearDown(self):
        self.clock.stop()
        self.widget.shutdown()
        self.widget.close()
        self.widget.deleteLater()
        self.app.processEvents()

    def commands(self):
        return [call.args[0].data for call in self.publisher.publish.call_args_list]

    def send_position(self, motor, text):
        self.widget.motor_position_entries[motor].setText(text)
        self.widget.motor_send_buttons[motor].click()

    def test_servo_drag_is_limited_but_release_always_sends(self):
        slider = self.widget.motor_sliders[MotorType.SERVO]
        slider.setSliderDown(True)
        slider.setValue(10)
        slider.setValue(20)
        self.assertEqual(self.commands(), ['SER:10\n'])
        self.now.return_value += MIN_CMD_DELAY + 0.01
        slider.setValue(30)
        slider.setValue(40)
        slider.setSliderDown(False)
        self.assertEqual(self.commands(), ['SER:10\n', 'SER:30\n', 'SER:40\n'])
        self.assertEqual(self.widget.last_motor_command_times[MotorType.SERVO],
                         self.now.return_value)

    def test_signed_positions_and_independent_motor_limits(self):
        self.send_position(MotorType.STEPPER, '-32767')
        self.send_position(MotorType.STEPPER, '12')
        self.send_position(MotorType.DC, '32767')
        self.assertEqual(self.commands(), ['STP:-:32767\n', 'DCM:P:+:32767\n'])
        self.now.return_value += MIN_CMD_DELAY + 0.01
        self.send_position(MotorType.STEPPER, '0')
        self.assertEqual(self.commands()[-1], 'STP:+:0\n')

    def test_dc_velocity_direction_release_and_shared_rate_limit(self):
        slider = self.widget.motor_sliders[MotorType.DC]
        slider.setSliderDown(True)
        slider.setValue(-60)
        self.send_position(MotorType.DC, '100')
        slider.setValue(60)
        slider.setSliderDown(False)
        self.assertEqual(self.commands(), ['DCM:V:-:60\n', 'DCM:V:+:60\n'])
        self.assertEqual(self.widget.motor_slider_labels[MotorType.DC].text(),
                         'RPM: 60')

    def test_invalid_position_entries_do_not_publish(self):
        for motor in (MotorType.STEPPER, MotorType.DC):
            for text in ('', '-', 'abc', '32768', '-32768', '1.5'):
                self.send_position(motor, text)
        self.assertEqual(self.commands(), [])

    def test_dc_position_resets_rpm_without_sending_velocity_command(self):
        slider = self.widget.motor_sliders[MotorType.DC]
        slider.setValue(30)
        self.now.return_value += MIN_CMD_DELAY + 0.01
        self.send_position(MotorType.DC, '-90')
        self.assertEqual(slider.value(), 0)
        self.assertEqual(self.widget.motor_slider_labels[MotorType.DC].text(), 'RPM: 0')
        self.assertEqual(self.widget.motor_position_entries[MotorType.DC].text(), '-90')
        self.assertEqual(self.commands(), ['DCM:V:+:30\n', 'DCM:P:-:90\n'])

    def test_dc_velocity_resets_position_only_when_sent(self):
        self.send_position(MotorType.DC, '90')
        slider = self.widget.motor_sliders[MotorType.DC]
        slider.setSliderDown(True)
        slider.setValue(-20)
        self.assertEqual(self.widget.motor_position_entries[MotorType.DC].text(), '90')
        self.assertEqual(self.widget.motor_labels[MotorType.DC].text(), 'POS: 90°')
        slider.setSliderDown(False)
        self.assertEqual(self.widget.motor_position_entries[MotorType.DC].text(), '0')
        self.assertEqual(self.widget.motor_labels[MotorType.DC].text(), 'POS: 0°')
        self.assertEqual(slider.value(), -20)
        self.assertEqual(self.commands(), ['DCM:P:+:90\n', 'DCM:V:-:20\n'])

    def test_stepper_has_position_entry_without_slider_or_motor_graphs(self):
        self.assertNotIn(MotorType.STEPPER, self.widget.motor_sliders)
        self.assertEqual(len(self.widget.sensor_charts), 6)
        self.widget.motor_position_entries[MotorType.STEPPER].setText('45')
        self.widget.motor_position_entries[MotorType.STEPPER].returnPressed.emit()
        self.assertEqual(self.commands(), ['STP:+:45\n'])


if __name__ == '__main__':
    unittest.main()
