#!/usr/bin/env python3
import math
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QGridLayout
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
from collections import deque
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QGridLayout, QHBoxLayout, QPushButton
import subprocess


# Thiết lập giao diện sáng (white theme)
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')

class RoundedAxis(pg.AxisItem):
    def __init__(self, orientation, decimals=3, **kwargs):
        super().__init__(orientation, **kwargs)
        self.decimals = decimals

    def tickStrings(self, values, scale, spacing):
        return [f"{v:.{self.decimals}f}" for v in values]


class MonitorTab(QWidget):
    def __init__(self, ros_wrapper, control_tab):
        super().__init__()
        self.ERR_ROUND = 3      # mét
        self.YAW_ROUND = 3     # rad

        self.ros = ros_wrapper
        self.control_tab = control_tab
        self.control_tab.goal_reached_signal.connect(self.on_goal_reached)

        self.lds_process = None  # để lưu process của lds50cr.py

        # Dữ liệu ROS
        self.last_odom = None
        self.yaw_points = []

        # ===== MAIN LAYOUT =====
        main_layout = QVBoxLayout(self)
        grid = QGridLayout()
        main_layout.addLayout(grid)
        self.setLayout(main_layout)

        # ===== PATH PLOT =====
        self.path_plot = pg.PlotWidget(title="Robot Path (X-Y)")
        self.path_plot.setLabel('bottom', 'X [m]')
        self.path_plot.setLabel('left', 'Y [m]')
        self.path_plot.showGrid(x=True, y=True)
        self.path_plot.addLegend()

        # --- Odom ---
        self.path_curve = self.path_plot.plot(
            pen=pg.mkPen('b', width=2),
            name="EKF"
        )
        self.odom_point = self.path_plot.plot(
            symbol='o', symbolBrush='r', symbolSize=8
        )

        # --- AMCL ---
        self.amcl_path_curve = self.path_plot.plot(
            pen=pg.mkPen('g', width=2, style=pg.QtCore.Qt.DashLine),
            name="AMCL"
        )
        self.amcl_point = self.path_plot.plot(
            symbol='o', symbolBrush='m', symbolSize=8
        )
        self.x_data, self.y_data = [], []

        # --- AMCL path ---
        self.amcl_x_data, self.amcl_y_data = [], []



        # ===== VELOCITY PLOT =====
        self.vel_plot = pg.PlotWidget(title="Velocity over Time")
        self.vel_plot.setLabel('bottom', 'Time [s]')
        self.vel_plot.setLabel('left', 'Velocity')
        self.vel_plot.showGrid(x=True, y=True)
        self.vel_plot.addLegend()
        self.vel_plot.setYRange(-1, 1)
        self.lin_vel_curve = self.vel_plot.plot(pen=pg.mkPen('b', width=2), name="Linear (m/s)")
        self.ang_vel_curve = self.vel_plot.plot(pen=pg.mkPen('orange', width=2), name="Angular (rad/s)")
        self.time_data = deque(maxlen=200)
        self.lin_vel_data = deque(maxlen=200)
        self.ang_vel_data = deque(maxlen=200)
        self.start_time = None

        # ===== POSITION ERROR PLOT =====
        err_x_axis = RoundedAxis('bottom', decimals=self.ERR_ROUND)
        err_y_axis = RoundedAxis('left', decimals=self.ERR_ROUND)

        self.err_plot = pg.PlotWidget(
            title="Position Error at Goals",
            axisItems={
                'bottom': err_x_axis,
                'left': err_y_axis
            }
        )


        self.err_plot.setLabel('bottom', 'Error X [m]')
        self.err_plot.setLabel('left', 'Error Y [m]')
        self.err_plot.showGrid(x=True, y=True)


        # ===== YAW ERROR PLOT =====
        yaw_x_axis = RoundedAxis('bottom', decimals=self.YAW_ROUND)
        yaw_y_axis = RoundedAxis('left', decimals=self.YAW_ROUND)

        self.yaw_plot = pg.PlotWidget(
            title="Yaw Desired vs Actual",
            axisItems={
                'bottom': yaw_x_axis,
                'left': yaw_y_axis
            }
        )

        self.yaw_plot.setLabel('bottom', 'Yaw Desired [rad]')
        self.yaw_plot.setLabel('left', 'Yaw Actual [rad]')
        self.yaw_plot.showGrid(x=True, y=True)

        # ---- Add plots to grid (2x2) ----
        grid.addWidget(self.path_plot, 0, 0)
        grid.addWidget(self.vel_plot, 0, 1)
        grid.addWidget(self.err_plot, 1, 0)
        grid.addWidget(self.yaw_plot, 1, 1)


        # ===== ADD BUTTONS RUN/STOP SCRIPT =====
        btn_layout = QHBoxLayout()
        self.btn_run_lds = QPushButton("▶️ Run Save")
        self.btn_stop_lds = QPushButton("⏹ Stop Save")
        btn_layout.addWidget(self.btn_run_lds)
        btn_layout.addWidget(self.btn_stop_lds)
        main_layout.addLayout(btn_layout)

        self.btn_run_lds.setCheckable(True)
        self.btn_stop_lds.setCheckable(True)

        self.btn_stop_lds.setEnabled(False)  # ban đầu chưa cho stop


        self.btn_run_lds.clicked.connect(self.run_lds_script)
        self.btn_stop_lds.clicked.connect(self.stop_lds_script)

        self.btn_run_lds.setStyleSheet("""
        QPushButton {
            background-color: #e7f0ff;
            border: 2px solid #0078d7;
            border-radius: 8px;
            font-weight: bold;
        }
        QPushButton:checked {
            background-color: #28a745;
            color: white;
            border: 2px solid #1e7e34;
        }
        """)

        self.btn_stop_lds.setStyleSheet("""
        QPushButton {
            background-color: #ffecec;
            border: 2px solid #cc0000;
            border-radius: 8px;
            font-weight: bold;
        }
        QPushButton:checked {
            background-color: #dc3545;
            color: white;
            border: 2px solid #a71d2a;
        }
        """)

        # ---- ROS Subscriptions ----
        self.ros.node.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.ros.node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            10
        )

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

        # ===== ERROR DATA =====
        self.err_x_data = []
        self.err_y_data = []

        self.err_curve = self.err_plot.plot(
            pen=None,
            symbol='o',
            symbolBrush='r',
            symbolSize=10
        )

        # ===== YAW DATA =====
        self.yaw_x_data = []
        self.yaw_y_data = []

        self.yaw_curve = self.yaw_plot.plot(
            pen=None,
            symbol='o',
            symbolBrush='r',
            symbolSize=10
        )

        self.yaw_plot.enableAutoRange(False)
        self.yaw_plot.setMouseEnabled(x=True, y=True)


        self.err_plot.enableAutoRange(False)
        self.err_plot.setMouseEnabled(x=True, y=True)

        self.err_plot.setXRange(-0.5, 0.5)
        self.err_plot.setYRange(-0.5, 0.5)

        self.yaw_plot.setXRange(-math.pi, math.pi)
        self.yaw_plot.setYRange(-math.pi, math.pi)


    def odom_callback(self, msg):
        """Lưu dữ liệu Odom"""
        self.last_odom = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

        self.ros.last_x = x
        self.ros.last_y = y
        self.ros.last_yaw = yaw

        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.start_time

        self.x_data.append(x)
        self.y_data.append(y)
        self.time_data.append(t)
        self.lin_vel_data.append(msg.twist.twist.linear.x)
        self.ang_vel_data.append(msg.twist.twist.angular.z)

    def update_plot(self):
        # --- ODOM ---
        if len(self.x_data) > 1:
            self.path_curve.setData(self.x_data, self.y_data)
            self.odom_point.setData(
                [self.x_data[-1]],
                [self.y_data[-1]]
            )

        # --- AMCL ---
        if len(self.amcl_x_data) > 1:
            self.amcl_path_curve.setData(
                self.amcl_x_data,
                self.amcl_y_data
            )
            self.amcl_point.setData(
                [self.amcl_x_data[-1]],
                [self.amcl_y_data[-1]]
            )

        # --- Velocity ---
        if len(self.time_data) > 1:
            self.lin_vel_curve.setData(self.time_data, self.lin_vel_data)
            self.ang_vel_curve.setData(self.time_data, self.ang_vel_data)

    def on_goal_reached(self, name, x_err, y_err, yaw_target):
        x_err = round(x_err, self.ERR_ROUND)
        y_err = round(y_err, self.ERR_ROUND)
        yaw_target = round(yaw_target, self.YAW_ROUND)

        # ===== POSITION ERROR =====
        self.err_x_data.append(x_err)
        self.err_y_data.append(y_err)
        self.err_curve.setData(self.err_x_data, self.err_y_data)

        label = pg.TextItem(
            text=f"{name}\n({x_err}, {y_err})",
            anchor=(0.5, -0.3)
        )
        self.err_plot.addItem(label)
        label.setPos(x_err, y_err)

        # ===== YAW ERROR =====
        if self.last_odom:
            o = self.last_odom.pose.pose.orientation
            current_yaw = math.atan2(
                2.0 * (o.w * o.z + o.x * o.y),
                1.0 - 2.0 * (o.y ** 2 + o.z ** 2)
            )
            current_yaw = round(current_yaw, self.YAW_ROUND)

            self.yaw_x_data.append(yaw_target)
            self.yaw_y_data.append(current_yaw)
            self.yaw_curve.setData(self.yaw_x_data, self.yaw_y_data)

            label = pg.TextItem(
                text=f"{name}\n({yaw_target}, {current_yaw})",
                anchor=(0.5, -0.3)
            )
            self.yaw_plot.addItem(label)
            label.setPos(yaw_target, current_yaw)

    def run_lds_script(self):
        if self.lds_process is None or self.lds_process.poll() is not None:
            self.lds_process = subprocess.Popen(
                ["/usr/bin/env", "python3", "/home/nttien/ros2_ws/src/mobile/scripts/lds50cr.py"]
            )
            print("[MonitorTab] LDS50CR script started.")

            # --- UI state ---
            self.btn_run_lds.setChecked(True)
            self.btn_run_lds.setEnabled(False)

            self.btn_stop_lds.setEnabled(True)
            self.btn_stop_lds.setChecked(False)
        else:
            print("[MonitorTab] LDS50CR script is already running.")

    def stop_lds_script(self):
        if self.lds_process and self.lds_process.poll() is None:
            self.lds_process.terminate()
            self.lds_process.wait()
            print("[MonitorTab] LDS50CR script stopped.")
            self.lds_process = None

            # --- UI state ---
            self.btn_run_lds.setEnabled(True)
            self.btn_run_lds.setChecked(False)

            self.btn_stop_lds.setChecked(False)
            self.btn_stop_lds.setEnabled(False)
        else:
            print("[MonitorTab] LDS50CR script is not running.")

    def amcl_callback(self, msg):
        pose = msg.pose.pose

        x = pose.position.x
        y = pose.position.y

        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # --- lưu cho GUI / waypoint ---
        self.ros.last_x = x
        self.ros.last_y = y
        self.ros.last_yaw = yaw

        # --- lưu path AMCL (để đối chiếu) ---
        self.amcl_x_data.append(x)
        self.amcl_y_data.append(y)


