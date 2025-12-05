import os
import sys
import threading
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32, UInt32, Int8
from ament_index_python.packages import get_package_share_directory

# Qt / UI
try:
    from PySide2 import QtWidgets, QtCore, QtUiTools
except ImportError as e:
    print("ERROR: PySide2/QtUiTools not available. Install with: pip install PySide2 pyqtgraph")
    raise

# Plotting
import pyqtgraph as pg

# Optional NumPy acceleration
try:
    import numpy as np
    USE_NUMPY = True
except ImportError:
    USE_NUMPY = False

# Optional OpenGL acceleration (set True if your GPU/driver is stable)
USE_OPENGL = False
pg.setConfigOptions(useOpenGL=USE_OPENGL, antialias=True)


class RosBridge(QtCore.QObject):
    """Qt bridge for thread-safe UI updates from ROS callbacks."""
    ph_changed = QtCore.Signal(float)
    temp_changed = QtCore.Signal(float)
    vol_changed = QtCore.Signal(int)
    data_point = QtCore.Signal(float, int)  # (pH, volume)


class TitrationUiNode(Node):
    """ROS 2 node handling subscriptions, publications, and logging."""
    def __init__(self, bridge: RosBridge):
        super().__init__('titration_ui')
        self.bridge = bridge

        # Parameters
        self.declare_parameter('log_rate_hz', 5.0)
        self.declare_parameter('output_dir', os.path.join(os.environ.get('HOME', '/tmp'), 'titration_logs'))
        self.log_rate_hz = float(self.get_parameter('log_rate_hz').value)
        self.output_dir = str(self.get_parameter('output_dir').value)

        # Ensure output directory exists
        try:
            os.makedirs(self.output_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().warn(f'Failed to create output_dir: {e}')

        # Latest values
        self.latest_ph = None
        self.latest_temp = None
        self.latest_vol = 0

        # Publishers
        self.pipette_cmd_pub = self.create_publisher(Int8, '/pipette_cmd', 10)

        # Subscribers
        self.create_subscription(Float32, '/ph', self._on_ph, 10)
        self.create_subscription(Float32, '/temperature', self._on_temp, 10)
        self.create_subscription(UInt32, '/titration_vol', self._on_vol, 10)

        # Logging
        self._saving_enabled = False
        self._csv_file = None
        self._csv_writer = None
        self._log_lock = threading.Lock()
        self._log_timer = self.create_timer(1.0 / max(self.log_rate_hz, 0.001), self._on_log_tick)

    # --- Subscriber callbacks ---
    def _on_ph(self, msg: Float32):
        self.latest_ph = float(msg.data)
        # UI updates + plotting are handled in the UI controller
        self.bridge.ph_changed.emit(self.latest_ph)
        # NOTE: do not emit data_point here (UI will use latest volume on ph updates)

    def _on_temp(self, msg: Float32):
        self.latest_temp = float(msg.data)
        self.bridge.temp_changed.emit(self.latest_temp)

    def _on_vol(self, msg: UInt32):
        self.latest_vol = int(msg.data)
        self.bridge.vol_changed.emit(self.latest_vol)
        # Also emit a data point when volume changes to capture that step (UI guards duplicates)
        if self.latest_ph is not None:
            self.bridge.data_point.emit(self.latest_ph, int(self.latest_vol))

    # --- Publisher helpers for buttons ---
    def send_pipette_cmd(self, code: int):
        self.pipette_cmd_pub.publish(Int8(data=int(code)))

    # --- Logging control ---
    def start_saving(self):
        with self._log_lock:
            if self._saving_enabled:
                return
            ts_name = datetime.now().strftime('%Y%m%d-%H%M%S')
            filepath = os.path.join(self.output_dir, f'{ts_name}.csv')
            self.get_logger().info(f'Start logging to {filepath}')
            self._csv_file = open(filepath, mode='w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(['Time', 'pH', 'Temperature', 'Titration volume'])
            self._saving_enabled = True

    def stop_saving(self):
        with self._log_lock:
            if not self._saving_enabled:
                return
            self.get_logger().info('Stop logging')
            try:
                if self._csv_file:
                    self._csv_file.flush()
                    self._csv_file.close()
            except Exception:
                pass
            self._csv_file = None
            self._csv_writer = None
            self._saving_enabled = False

    def _on_log_tick(self):
        with self._log_lock:
            if not self._saving_enabled or self._csv_writer is None:
                return
            now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            ph = '' if self.latest_ph is None else f'{self.latest_ph:.3f}'
            temp = '' if self.latest_temp is None else f'{self.latest_temp:.3f}'
            vol = '' if self.latest_vol is None else str(int(self.latest_vol))
            self._csv_writer.writerow([now_str, ph, temp, vol])

    # --- Cleanup ---
    def destroy_node(self):
        try:
            self.stop_saving()
        except Exception:
            pass
        super().destroy_node()


class UiController(QtCore.QObject):
    """
    Loads the .ui file. If root is QMainWindow, use it directly.
    If root is QWidget, host it inside our own QMainWindow.
    Buffers data and redraws the plot at fixed rate for performance.
    """
    def __init__(self, node: TitrationUiNode, bridge: RosBridge):
        super().__init__()
        self.node = node
        self.bridge = bridge

        # Plotting state (disabled until Start Plot is pressed)
        self._plotting_enabled = False

        # Resolve UI path
        ui_path = os.path.join(get_package_share_directory('titration_ui'), 'ui', 'mainwindow.ui')
        if not os.path.exists(ui_path):
            QtWidgets.QMessageBox.critical(None, 'UI Error', f'UI file not found:\n{ui_path}')
            raise FileNotFoundError(ui_path)

        # Load UI
        loader = QtUiTools.QUiLoader()
        ui_file = QtCore.QFile(ui_path)
        if not ui_file.open(QtCore.QFile.ReadOnly):
            QtWidgets.QMessageBox.critical(None, 'UI Error', f'Failed to open UI file:\n{ui_path}')
            raise RuntimeError(f'Cannot open {ui_path}')
        loaded = loader.load(ui_file)
        ui_file.close()

        if loaded is None:
            QtWidgets.QMessageBox.critical(None, 'UI Error', f'Failed to load UI from:\n{ui_path}')
            raise RuntimeError(f'QUiLoader failed for {ui_path}')

        # Decide window to show
        if isinstance(loaded, QtWidgets.QMainWindow):
            self.window = loaded  # Use as-is
        else:
            # Host QWidget inside our QMainWindow
            self.window = QtWidgets.QMainWindow()
            self.window.setWindowTitle('Titration UI')
            self.window.setCentralWidget(loaded)

        # Intercept close -> stop logging
        self.window.installEventFilter(self)

        # Widget finder (search from window/root)
        def must_find(cls, name: str):
            w = self.window.findChild(cls, name)
            if w is None:
                QtWidgets.QMessageBox.critical(self.window, 'UI Error', f'Widget "{name}" not found in UI.')
                raise RuntimeError(f'Widget "{name}" not found')
            return w

        # Widgets
        self.lblPH = must_find(QtWidgets.QLabel, 'lblPH')
        self.lblTemp = must_find(QtWidgets.QLabel, 'lblTemp')
        self.lblVol = must_find(QtWidgets.QLabel, 'lblVol')

        self.btnAspire = must_find(QtWidgets.QPushButton, 'btnAspire')
        self.btnDispense = must_find(QtWidgets.QPushButton, 'btnDispense')
        self.btnEnterOnline = must_find(QtWidgets.QPushButton, 'btnEnterOnline')
        self.btnExitOnline = must_find(QtWidgets.QPushButton, 'btnExitOnline')
        self.btnSave = must_find(QtWidgets.QPushButton, 'btnStartLogging')
        self.btnStop = must_find(QtWidgets.QPushButton, 'btnStopLogging')
        self.btnStartPlot = must_find(QtWidgets.QPushButton, 'btnStartPlot')
        self.btnClear = must_find(QtWidgets.QPushButton, 'btnClearPlot')

        # Plot setup
        self.plotContainer = must_find(QtWidgets.QWidget, 'plotContainer')
        self.plotLayout = self.plotContainer.layout()
        if self.plotLayout is None:
            self.plotLayout = QtWidgets.QVBoxLayout(self.plotContainer)
            self.plotContainer.setLayout(self.plotLayout)

        self.plot_widget = pg.PlotWidget(background='w')
        self.plotLayout.addWidget(self.plot_widget)
        self.plot = self.plot_widget.getPlotItem()
        self.plot.setTitle('pH vs Titration Volume')
        self.plot.setLabel('left', 'pH')
        self.plot.setLabel('bottom', 'Titration Volume')
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setYRange(0.0, 14.0)
        self.plot.setXRange(0, 1)

        self.plot_curve = self.plot.plot([], [], pen=pg.mkPen(color=(0, 120, 255), width=2))
        # Performance options (guard for older pyqtgraph)
        try:
            self.plot_curve.setDownsampling(auto=True, method='largest')
            self.plot_curve.setClipToView(True)
        except Exception:
            pass

        # Buffers
        if USE_NUMPY:
            self.x_data = np.empty(0, dtype=np.uint32)
            self.y_data = np.empty(0, dtype=np.float32)
        else:
            self.x_data = []
            self.y_data = []
        self.max_vol_seen = 1
        self._last_plotted_vol = None

        # UI refresh timer (decouple from message rate)
        self._refresh_hz = 5.0  # Adjust based on machine load
        self._max_points = 10000  # History cap (tune as needed)
        self._refresh_timer = QtCore.QTimer(self.window)
        self._refresh_timer.timeout.connect(self._plot_refresh)
        self._refresh_timer.start(int(1000.0 / self._refresh_hz))

        # Connect signals for real-time updates
        self.bridge.ph_changed.connect(self._on_ph_update)
        self.bridge.temp_changed.connect(self._on_temp_update)
        self.bridge.vol_changed.connect(self._on_vol_update)
        self.bridge.data_point.connect(self._on_data_point)

        # Button actions -> publish codes
        self.btnAspire.clicked.connect(lambda: self.node.send_pipette_cmd(7))
        self.btnDispense.clicked.connect(lambda: self.node.send_pipette_cmd(8))
        self.btnEnterOnline.clicked.connect(lambda: self.node.send_pipette_cmd(81))
        self.btnExitOnline.clicked.connect(lambda: self.node.send_pipette_cmd(80))

        # Logging control
        self.btnSave.clicked.connect(self.node.start_saving)
        self.btnStop.clicked.connect(self.node.stop_saving)

        # Plot control
        self.btnStartPlot.clicked.connect(self._start_plot)
        self.btnClear.clicked.connect(self._clear_plot)

        # Initial size
        self.window.resize(900, 600)

    # --- UI update handlers (lightweight) ---
    def _on_ph_update(self, ph: float):
        # Update label
        self.lblPH.setText(f'pH: {ph:.3f}')

        # If plotting enabled, append a point using latest volume
        if not self._plotting_enabled:
            return

        vol = self.node.latest_vol
        if vol is None:
            return

        if USE_NUMPY:
            self.x_data = np.append(self.x_data, np.uint32(vol))
            self.y_data = np.append(self.y_data, np.float32(ph))
        else:
            self.x_data.append(int(vol))
            self.y_data.append(float(ph))

        if vol > self.max_vol_seen:
            self.max_vol_seen = vol
        # Redraw happens in _plot_refresh

    def _on_temp_update(self, temp: float):
        self.lblTemp.setText(f'Temperature: {temp:.3f} °C')

    def _on_vol_update(self, vol: int):
        self.lblVol.setText(f'Volume: {vol}')
        if vol > self.max_vol_seen:
            self.max_vol_seen = vol

        if not self._plotting_enabled:
            return

        # Append one point on volume change (paired with latest pH)
        ph = self.node.latest_ph
        if ph is None:
            return
        if self._last_plotted_vol is None or vol != self._last_plotted_vol:
            if USE_NUMPY:
                self.x_data = np.append(self.x_data, np.uint32(vol))
                self.y_data = np.append(self.y_data, np.float32(ph))
            else:
                self.x_data.append(int(vol))
                self.y_data.append(float(ph))
            self._last_plotted_vol = vol
        # Redraw happens in _plot_refresh

    def _on_data_point(self, ph: float, vol: int):
        # Volume-triggered path; obey plotting gate and guard duplicates
        if not self._plotting_enabled:
            return
        if self._last_plotted_vol is None or vol != self._last_plotted_vol:
            if USE_NUMPY:
                self.x_data = np.append(self.x_data, np.uint32(vol))
                self.y_data = np.append(self.y_data, np.float32(ph))
            else:
                self.x_data.append(int(vol))
                self.y_data.append(float(ph))
            self._last_plotted_vol = vol
        # Redraw happens in _plot_refresh

    def _plot_refresh(self):
        # If plotting disabled, do nothing (no redraw)
        if not self._plotting_enabled:
            return

        length = (self.x_data.size if USE_NUMPY else len(self.x_data))
        if length == 0:
            return

        # Cap history to avoid unbounded growth
        if length > self._max_points:
            if USE_NUMPY:
                self.x_data = self.x_data[-self._max_points:]
                self.y_data = self.y_data[-self._max_points:]
            else:
                self.x_data = self.x_data[-self._max_points:]
                self.y_data = self.y_data[-self._max_points:]

        # Update curve
        if USE_NUMPY:
            self.plot_curve.setData(self.x_data, self.y_data)
        else:
            try:
                x_np = np.asarray(self.x_data, dtype=np.uint32)
                y_np = np.asarray(self.y_data, dtype=np.float32)
                self.plot_curve.setData(x_np, y_np)
            except Exception:
                self.plot_curve.setData(self.x_data, self.y_data)

        # Maintain X range from 0..max volume seen
        self.plot.setXRange(0, max(int(self.max_vol_seen), 1), padding=0.02)

    def _start_plot(self):
        # Enable plotting; do not backfill historical data automatically
        self._plotting_enabled = True

    def _clear_plot(self):
        # Reset buffers and plot, and STOP plotting
        self._plotting_enabled = False
        if USE_NUMPY:
            self.x_data = np.empty(0, dtype=np.uint32)
            self.y_data = np.empty(0, dtype=np.float32)
        else:
            self.x_data = []
            self.y_data = []
        self.max_vol_seen = 1
        self._last_plotted_vol = None
        self.plot_curve.setData([], [])
        self.plot.setXRange(0, 1, padding=0.02)

    # --- Event filter to catch window close and stop logging cleanly ---
    def eventFilter(self, watched, event):
        if watched is self.window and event.type() == QtCore.QEvent.Close:
            try:
                self.node.stop_saving()
            except Exception:
                pass
        return super().eventFilter(watched, event)

    def show(self):
        self.window.show()


def main():
    rclpy.init(args=None)
    app = QtWidgets.QApplication(sys.argv)

    bridge = RosBridge()
    node = TitrationUiNode(bridge)

    # Start ROS 2 executor in a background thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Load and show UI
    ui = UiController(node, bridge)
    ui.show()

    # Run Qt event loop
    exit_code = app.exec_()

    # Cleanup
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join(timeout=1.0)
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
