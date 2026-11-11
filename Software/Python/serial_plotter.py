import csv
import sys
from pathlib import Path
from typing import List, Optional

import pyqtgraph as pg
import serial
from pyqtgraph import mkPen
from PySide6.QtCore import QThread, Signal, QObject, Qt
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QCheckBox,
    QDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QComboBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from serial.tools import list_ports


class SerialWorker(QObject):
    dataReceived = Signal(list)
    statusChanged = Signal(str)
    connectionError = Signal(str)
    responseReceived = Signal(str)
    finished = Signal()

    def __init__(self, port: str, baudrate: int) -> None:
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self._stop_requested = False
        self._streaming = True
        self.serial = None

    def stop(self) -> None:
        self._stop_requested = True
        if self.serial and self.serial.is_open:
            self.serial.close()

    def set_streaming(self, enabled: bool) -> None:
        self._streaming = enabled

    def run(self) -> None:
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.5)
        except serial.SerialException as exc:
            self.connectionError.emit(str(exc))
            self.finished.emit()
            return

        self.statusChanged.emit(f"Connected to {self.port} at {self.baudrate} baud")

        while not self._stop_requested:
            if not self._streaming:
                QThread.msleep(50)
                continue

            try:
                line = self.serial.readline()
            except serial.SerialException as exc:
                self.connectionError.emit(str(exc))
                break

            if not line:
                continue

            text = line.decode("utf-8", errors="ignore").strip()
            if not text:
                continue

            parts = [part.strip() for part in text.split(",") if part.strip()]
            if not parts:
                continue

            try:
                values = [float(part) for part in parts]
            except ValueError:
                self.responseReceived.emit(text)
                continue

            self.dataReceived.emit(values)

        if self.serial and self.serial.is_open:
            self.serial.close()
        self.finished.emit()

    def send_command(self, command: str) -> None:
        if not self.serial or not self.serial.is_open:
            return
        payload = (command.rstrip("\r\n") + "\n").encode("utf-8")
        self.serial.write(payload)
        self.statusChanged.emit(f"Sent: {command}")


class PlotSettingsDialog(QDialog):
    def __init__(self, parent: Optional[QWidget], max_points: int) -> None:
        super().__init__(parent)
        self.setWindowTitle("Plot Settings")
        self.setModal(True)

        self.buffer_input = QLineEdit(str(max_points))
        self.buffer_input.setPlaceholderText("Buffer size")

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Data buffer size (points per channel):"))
        layout.addWidget(self.buffer_input)

        buttons = QHBoxLayout()
        ok_button = QPushButton("OK")
        ok_button.clicked.connect(self.accept)
        cancel_button = QPushButton("Cancel")
        cancel_button.clicked.connect(self.reject)
        buttons.addWidget(ok_button)
        buttons.addWidget(cancel_button)
        layout.addLayout(buttons)

    def get_buffer_size(self) -> int:
        try:
            return max(10, int(self.buffer_input.text().strip()))
        except ValueError:
            return 500


class SerialPlotterWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Serial Plotter")
        self.resize(1100, 700)

        self.sample_index = 0
        self.curves: List[pg.PlotCurveItem] = []
        self.series_buffers: List[List[float]] = []
        self.x_values: List[int] = []
        self.curve_checkboxes: List[QCheckBox] = []
        self.curve_visibility: List[bool] = []
        self.max_points = 500
        self.worker_thread: Optional[QThread] = None
        self.worker: Optional[SerialWorker] = None

        self._build_ui()
        self._refresh_ports()

    def _build_ui(self) -> None:
        central = QWidget(self)
        self.setCentralWidget(central)

        layout = QVBoxLayout(central)

        controls = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setPlaceholderText("Select serial port")
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "115200", "230400", "460800", "921600"])
        self.baud_combo.setCurrentText("115200")

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_serial)
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self.disconnect_serial)
        self.disconnect_button.setEnabled(False)
        self.start_button = QPushButton("Start Stream")
        self.start_button.clicked.connect(self.start_stream)
        self.start_button.setEnabled(False)
        self.stop_button = QPushButton("Stop Stream")
        self.stop_button.clicked.connect(self.stop_stream)
        self.stop_button.setEnabled(False)
        self.save_button = QPushButton("Save Data")
        self.save_button.clicked.connect(self.save_data)
        self.save_button.setEnabled(False)

        controls.addWidget(QLabel("Port:"))
        controls.addWidget(self.port_combo, 2)
        controls.addWidget(QLabel("Baud:"))
        controls.addWidget(self.baud_combo, 1)
        controls.addWidget(self.connect_button)
        controls.addWidget(self.disconnect_button)
        controls.addWidget(self.start_button)
        controls.addWidget(self.stop_button)
        controls.addWidget(self.save_button)
        layout.addLayout(controls)

        self.status_label = QLabel("Disconnected")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        command_row = QHBoxLayout()
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Enter serial command")
        self.send_button = QPushButton("Send")
        self.send_button.clicked.connect(self.send_command)
        self.send_button.setEnabled(False)
        command_row.addWidget(self.command_input, 3)
        command_row.addWidget(self.send_button)
        layout.addLayout(command_row)

        self.response_box = QTextEdit()
        self.response_box.setReadOnly(True)
        self.response_box.setMaximumHeight(140)
        self.response_box.setPlaceholderText("Serial responses will appear here")
        layout.addWidget(self.response_box)

        plot_row = QHBoxLayout()
        self.plot_widget = pg.PlotWidget(title="Live Serial Data")
        self.plot_widget.setBackground("w")
        self.plot_widget.setLabel("left", "Value")
        self.plot_widget.setLabel("bottom", "Sample")
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.addLegend()
        plot_row.addWidget(self.plot_widget, 3)

        self.curves_panel = QWidget(self)
        self.curves_panel.setMaximumWidth(220)
        curves_layout = QVBoxLayout(self.curves_panel)
        curves_layout.addWidget(QLabel("Curves"))
        curves_layout.addStretch()
        plot_row.addWidget(self.curves_panel, 1)
        layout.addLayout(plot_row)

        self.curves_container = curves_layout
        self._add_menu_actions()

    def open_plot_settings(self) -> None:
        dialog = PlotSettingsDialog(self, self.max_points)
        if dialog.exec() == QDialog.Accepted:
            self.max_points = dialog.get_buffer_size()
            self.status_label.setText(f"Plot buffer size set to {self.max_points} points")
            self._trim_buffers_to_limit()

    def _add_menu_actions(self) -> None:
        file_menu = self.menuBar().addMenu("File")
        save_action = QAction("Save Data", self)
        save_action.triggered.connect(self.save_data)
        file_menu.addAction(save_action)

        settings_action = QAction("Plot Settings", self)
        settings_action.triggered.connect(self.open_plot_settings)
        file_menu.addAction(settings_action)

        quit_action = QAction("Quit", self)
        quit_action.triggered.connect(self.close)
        file_menu.addAction(quit_action)

    def _refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        current = self.port_combo.currentText()
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current:
            self.port_combo.setCurrentText(current)
        if ports:
            self.port_combo.setCurrentText(ports[0])

    def connect_serial(self) -> None:
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "No port selected", "Choose a serial port first.")
            return

        if self.worker_thread and self.worker_thread.isRunning():
            return

        baudrate = int(self.baud_combo.currentText())
        self.status_label.setText(f"Connecting to {port}...")
        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(True)
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(True)
        self.save_button.setEnabled(True)
        self.send_button.setEnabled(True)

        self.worker_thread = QThread(self)
        self.worker = SerialWorker(port, baudrate)
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.worker.run)
        self.worker.dataReceived.connect(self.handle_data)
        self.worker.statusChanged.connect(self.update_status)
        self.worker.connectionError.connect(self.handle_connection_error)
        self.worker.responseReceived.connect(self.handle_response)
        self.worker.finished.connect(self.worker_thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.worker_thread.finished.connect(self.worker_thread.deleteLater)
        self.worker_thread.start()

    def disconnect_serial(self) -> None:
        if self.worker is not None:
            self.worker.stop()
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.send_button.setEnabled(False)
        self.status_label.setText("Disconnected")

    def start_stream(self) -> None:
        if self.worker is None:
            QMessageBox.warning(self, "Not connected", "Connect to a serial port first.")
            return
        self.worker.set_streaming(True)
        self.status_label.setText("Streaming data")

    def stop_stream(self) -> None:
        if self.worker is None:
            return
        self.worker.set_streaming(False)
        self.status_label.setText("Streaming paused")

    def _trim_buffers_to_limit(self) -> None:
        for buffer in self.series_buffers:
            while len(buffer) > self.max_points:
                buffer.pop(0)

        if len(self.x_values) > self.max_points:
            self.x_values = self.x_values[-self.max_points:]

        for idx, curve in enumerate(self.curves):
            if idx < len(self.series_buffers):
                curve.setData(self.x_values, self.series_buffers[idx])

    def _refresh_curve_controls(self) -> None:
        while self.curves_container.count():
            item = self.curves_container.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        self.curve_checkboxes = []
        self.curve_visibility = []
        self.curves_container.addWidget(QLabel("Curves"))

        for idx, curve in enumerate(self.curves):
            checked = True
            if idx < len(self.curve_visibility):
                checked = self.curve_visibility[idx]

            checkbox = QCheckBox(f"Channel {idx + 1}", self)
            checkbox.setChecked(checked)
            checkbox.toggled.connect(lambda checked, idx=idx: self._toggle_curve(idx, checked))
            self.curve_checkboxes.append(checkbox)
            self.curve_visibility.append(checked)
            self.curves_container.addWidget(checkbox)

        self.curves_container.addStretch()

    def _toggle_curve(self, index: int, visible: bool) -> None:
        if index < len(self.curves):
            self.curves[index].setVisible(visible)
            if index < len(self.curve_visibility):
                self.curve_visibility[index] = visible

    def handle_data(self, values: List[float]) -> None:
        if not values:
            return

        if not self.series_buffers:
            self.series_buffers = [[] for _ in range(len(values))]
            self.x_values = []
            self.curves = []
            self.curve_checkboxes = []
            self.curve_visibility = []
            self.plot_widget.clear()
            for idx, value in enumerate(values):
                curve = self.plot_widget.plot([], [], name=f"Channel {idx + 1}", pen=mkPen((idx + 1) * 40 % 255, (idx + 2) * 60 % 255, (idx + 3) * 80 % 255))
                self.curves.append(curve)
            self._refresh_curve_controls()

        if len(values) != len(self.series_buffers):
            # Add or remove channels to match the incoming data shape.
            if len(values) > len(self.series_buffers):
                for _ in range(len(values) - len(self.series_buffers)):
                    self.series_buffers.append([])
                    curve = self.plot_widget.plot([], [], name=f"Channel {len(self.series_buffers)}", pen=mkPen((len(self.series_buffers) * 40) % 255, (len(self.series_buffers) * 60) % 255, (len(self.series_buffers) * 80) % 255))
                    self.curves.append(curve)
                self._refresh_curve_controls()
            else:
                self.series_buffers = self.series_buffers[: len(values)]
                self.curves = self.curves[: len(values)]
                self.plot_widget.clear()
                for idx, curve in enumerate(self.curves):
                    self.plot_widget.addLegend()
                    self.plot_widget.plot([], [], name=f"Channel {idx + 1}", pen=curve.opts["pen"])
                self._refresh_curve_controls()

        self.sample_index += 1
        self.x_values.append(self.sample_index)
        if len(self.x_values) > self.max_points:
            self.x_values.pop(0)

        for idx, value in enumerate(values):
            self.series_buffers[idx].append(float(value))
            if len(self.series_buffers[idx]) > self.max_points:
                self.series_buffers[idx].pop(0)

        for idx, curve in enumerate(self.curves):
            if idx < len(self.series_buffers):
                curve.setData(self.x_values, self.series_buffers[idx])
                curve.setVisible(self.curve_visibility[idx] if idx < len(self.curve_visibility) else True)

    def send_command(self) -> None:
        if self.worker is None:
            QMessageBox.warning(self, "Not connected", "Connect to a serial port first.")
            return

        command = self.command_input.text().strip()
        if not command:
            return

        self.worker.send_command(command)
        self.response_box.append(f"> {command}")

    def handle_response(self, text: str) -> None:
        self.response_box.append(text)

    def update_status(self, message: str) -> None:
        self.status_label.setText(message)

    def handle_connection_error(self, message: str) -> None:
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.status_label.setText(message)

    def save_data(self) -> None:
        if not self.series_buffers:
            QMessageBox.information(self, "No data", "No plotted data to save yet.")
            return

        path, _ = QFileDialog.getSaveFileName(self, "Save Serial Data", str(Path.home() / "serial_data.csv"), "CSV Files (*.csv)")
        if not path:
            return

        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["sample"] + [f"channel_{idx + 1}" for idx in range(len(self.series_buffers))])
            for row_idx in range(len(self.x_values)):
                row = [self.x_values[row_idx]]
                for series in self.series_buffers:
                    if row_idx < len(series):
                        row.append(series[row_idx])
                    else:
                        row.append("")
                writer.writerow(row)

        self.status_label.setText(f"Saved data to {path}")


def main() -> int:
    app = QApplication(sys.argv)
    window = SerialPlotterWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
