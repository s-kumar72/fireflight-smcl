# Start the UI: ros2 run ui ui_node
"""Layout-only rqt panel for the FireFlight user interface."""

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QLineEdit, QPushButton, QSizePolicy,
    QSlider, QVBoxLayout, QWidget,
)
from rqt_gui_py.plugin import Plugin


class UiWidget(QWidget):
    """Display sensor controls, chart placeholders, and motor cells."""

    def __init__(self):
        super().__init__()
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


class UiPlugin(Plugin):
    """Expose the layout as an rqt plugin without data connections."""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('FireFlightUiPlugin')
        self.widget = UiWidget()
        context.add_widget(self.widget)
        # Wait until rqt has attached the panel to its main window.
        QTimer.singleShot(0, lambda: self.widget.window().showMaximized())


def main(args=None):
    """Launch this panel in its own rqt window."""
    import sys

    from rqt_gui.main import Main

    argv = sys.argv if args is None else [sys.argv[0], *args]
    return Main().main(argv=argv, standalone='ui.ui_node.UiPlugin')


if __name__ == '__main__':
    raise SystemExit(main())
