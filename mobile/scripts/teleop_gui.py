#!/usr/bin/env python3
import sys
import math
import yaml
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QGridLayout, QLabel,
    QMessageBox, QHBoxLayout, QScrollArea, QFrame, QTabWidget
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from status_bar import StatusBarWidget
from ros_worker import RosWorker
from monitor_tab import MapManagerTab

from PyQt5.QtCore import QRectF
from PyQt5.QtWidgets import QInputDialog
from PyQt5.QtWidgets import QSpacerItem, QSizePolicy

from PyQt5.QtCore import pyqtSignal

from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QLabel



# ---------------- Helper functions ----------------
def yaw_to_quaternion(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return [qx, qy, qz, qw]


# ---------------- ROS Wrapper ----------------
class RosNodeWrapper:
    def __init__(self):
        self.node = Node('teleop_nav_gui_node')
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

        # ---- realtime state ----
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0
        self.last_v = 0.0
        self.last_w = 0.0
        self.mode = "IDLE"

        self.node.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_cb, 10
        )

    def cmd_vel_cb(self, msg):
        self.last_v = msg.linear.x
        self.last_w = msg.angular.z
        if abs(self.last_v) > 1e-3 or abs(self.last_w) > 1e-3:
            self.mode = "MOTION"
        else:
            self.mode = "IDLE"

class IntroduceTab(QWidget):
    def __init__(self):
        super().__init__()

        self.image_path = "/home/nttien/Downloads/pk.png"
        self.original_pixmap = QPixmap(self.image_path)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.label = QLabel()
        self.label.setAlignment(Qt.AlignCenter)

        layout.addWidget(self.label)

        self.update_pixmap()

    def resizeEvent(self, event):
        self.update_pixmap()
        super().resizeEvent(event)

    def update_pixmap(self):
        if self.original_pixmap.isNull():
            return

        scaled = self.original_pixmap.scaled(
            self.size(),
            Qt.KeepAspectRatio,      # 🔴 GIỮ TỈ LỆ
            Qt.SmoothTransformation
        )
        self.label.setPixmap(scaled)


# ---------------- Control Panel ----------------
class ControlPanel(QWidget):
    goal_reached_signal = pyqtSignal(str, float, float, float)
    def __init__(self, ros_wrapper, goals_yaml_path, history_path):
        super().__init__()
        
        self.ros = ros_wrapper
        self.goals_yaml_path = goals_yaml_path
        self.history_path = history_path

        self.route_list = []
        self.current_goal_index = 0
        self.route_active = False
        self.goals = []



        # ===== Keyboard teleop state =====
        self.cur_v = 0.0
        self.cur_w = 0.0

        self.manual_active = False
        # Timer publish cmd_vel liên tục (20 Hz)
        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self.publish_continuous_cmd)
        # self.cmd_timer.start(50)

        # Cho phép widget nhận phím
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()


        layout = QVBoxLayout(self)
        title = QLabel("RICLAB")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Manual control
        teleop_label = QLabel("D-150 Robot")
        teleop_label.setAlignment(Qt.AlignCenter)
        teleop_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(teleop_label)

        grid = QGridLayout()
        layout.addLayout(grid)

        # Movement buttons
        self.btn_w = QPushButton("W")
        self.btn_a = QPushButton("A")
        self.btn_s = QPushButton("S")
        self.btn_d = QPushButton("D")

        for b in (self.btn_w, self.btn_a, self.btn_s, self.btn_d):
            b.setFixedSize(64, 48)
            b.setFont(QFont("Arial", 12, QFont.Bold))
            b.setObjectName("controlButton")

        grid.addWidget(self.btn_w, 0, 1)
        grid.addWidget(self.btn_a, 1, 0)
        grid.addWidget(self.btn_d, 1, 2)
        grid.addWidget(self.btn_s, 2, 1)

        # Movement bindings
        self.btn_w.pressed.connect(lambda: self.publish_cmd(0.25, 0.0))
        self.btn_w.released.connect(self.stop_robot)
        self.btn_s.pressed.connect(lambda: self.publish_cmd(-0.25, 0.0))
        self.btn_s.released.connect(self.stop_robot)
        self.btn_a.pressed.connect(lambda: self.publish_cmd(0.0, 0.5))
        self.btn_a.released.connect(self.stop_robot)
        self.btn_d.pressed.connect(lambda: self.publish_cmd(0.0, -0.5))
        self.btn_d.released.connect(self.stop_robot)

        # Goals section
        layout.addSpacing(10)
        goals_label = QLabel("📍 Predefined Goals")
        goals_label.setAlignment(Qt.AlignCenter)
        goals_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(goals_label)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        layout.addWidget(scroll_area, stretch=1)

        container = QWidget()
        self.goals_layout = QVBoxLayout(container)
        scroll_area.setWidget(container)

        # Route display
        layout.addSpacing(10)
        route_label = QLabel("Current Route")
        route_label.setAlignment(Qt.AlignCenter)
        route_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(route_label)

        self.route_display = QLabel("(Empty)")
        self.route_display.setAlignment(Qt.AlignCenter)
        self.route_display.setObjectName("routeDisplay")
        layout.addWidget(self.route_display)

        # Current target label
        self.current_target_label = QLabel("Current Target: (None)")
        self.current_target_label.setAlignment(Qt.AlignCenter)
        self.current_target_label.setObjectName("currentTarget")
        layout.addWidget(self.current_target_label)

        # Buttons
        btn_layout = QHBoxLayout()
        self.btn_start_route = QPushButton("▶️ Start Route")
        self.btn_cancel_route = QPushButton("🗑️ Cancel Route")

        for b in (self.btn_start_route, self.btn_cancel_route):
            b.setFixedHeight(44)
            b.setFont(QFont("Arial", 12, QFont.Bold))
        btn_layout.addWidget(self.btn_start_route)
        btn_layout.addWidget(self.btn_cancel_route)
        layout.addLayout(btn_layout)

        self.btn_start_route.clicked.connect(self.start_route)
        self.btn_cancel_route.clicked.connect(self.clear_route)

        # Status label
        self.status_label = QLabel("Status: Idle")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        # Load goals
        self.load_goals(self.goals_yaml_path)

        # ROS spin timer
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.spin_once)
        self.spin_timer.start(10)

        self._send_goal_future = None
        self._result_future = None
        self._goal_handle = None

    def cancel_current_goal(self):
        if self._goal_handle:
            self.status_label.setText("⏹ Cancelling current goal...")
            cancel_future = self._goal_handle.cancel_goal_async()

            def cancel_done(_):
                self.status_label.setText("❌ Goal cancelled")
                self.route_active = False
                self._goal_handle = None

            cancel_future.add_done_callback(cancel_done)

            

    # ---------- ROS ----------
    def spin_once(self):
        try:
            rclpy.spin_once(self.ros.node, timeout_sec=0.02)
        except Exception:
            pass

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.ros.cmd_pub.publish(msg)
        self.status_label.setText(f"Manual: v={v:.2f}, w={w:.2f}")
        # Cập nhật state manual
        self.cur_v = v
        self.cur_w = w

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.ros.cmd_pub.publish(msg)
        self.status_label.setText("Stopped")
        self.cur_v = 0.0
        self.cur_w = 0.0
    
    def publish_continuous_cmd(self):
        if not self.manual_active:
            return

        msg = Twist()
        msg.linear.x = self.cur_v
        msg.angular.z = self.cur_w
        self.ros.cmd_pub.publish(msg)

    # ---------- Goals ----------
    def load_goals(self, filepath):
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            self.goals = data.get('goals', [])

            # Clear layout trước
            while self.goals_layout.count():
                item = self.goals_layout.takeAt(0)
                if item.widget():
                    item.widget().deleteLater()

            # Sử dụng QGridLayout thay cho QVBoxLayout
            grid_layout = QGridLayout()
            self.goals_layout.addLayout(grid_layout)

            columns = 3  # số cột bạn muốn
            for index, g in enumerate(self.goals):
                name = g.get('name', 'Goal')
                btn = QPushButton(name)
                btn.setFont(QFont("Arial", 11, QFont.Bold))
                btn.setObjectName("goalButton")
                btn.clicked.connect(lambda _, goal=g: self.add_to_route(goal))

                row = index // columns
                col = index % columns
                grid_layout.addWidget(btn, row, col)

            # Thêm spacer để đẩy các nút lên trên
            spacer = QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding)
            grid_layout.addItem(spacer, (len(self.goals) + columns - 1) // columns, 0, 1, columns)

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load YAML:\n{e}")

    def add_to_route(self, goal):
        self.route_list.append(goal)
        names = [g.get('name', 'Goal') for g in self.route_list]
        self.route_display.setText(" → ".join(names))

    def clear_route(self):
        self.route_list.clear()
        self.route_display.setText("(Empty)")
        self.status_label.setText("Route cleared")

    # ---------- Route ----------
    def start_route(self):
        # if self.route_active:
        #     print("[ROUTE] Route already active, ignore start")
        #     return
        if self.route_active:
            self.cancel_current_goal()

        
        if not self.route_list:
            QMessageBox.warning(self, "Empty Route", "Please select at least one goal.")
            return
        if not self.ros.nav_client.wait_for_server(timeout_sec=2.0):
            QMessageBox.warning(self, "Nav2", "NavigateToPose server not available.")
            return

        self.save_route_to_history()
        self.current_goal_index = 0
        self.route_active = True
        self.status_label.setText("Starting route...")
        self.send_next_goal()

    def send_next_goal(self):
        print(f"[ROUTE] send_next_goal called, index = {self.current_goal_index}")

        if self.current_goal_index >= len(self.route_list):
            self.status_label.setText("✅ Route complete!")
            self.current_target_label.setText("Current Target: (None)")
            self.route_active = False
            return

        goal = self.route_list[self.current_goal_index]
        name = goal.get('name', f"Goal {self.current_goal_index}")
        self.current_target_label.setText(f"Current Target: {name}")
        self.status_label.setText(f"Sending goal {self.current_goal_index+1}/{len(self.route_list)}: {name}")

        pose = PoseStamped()
        pose.header.frame_id = goal.get('frame_id', 'map')
        pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        pos = goal.get('position', [0.0, 0.0, 0.0])
        ori = goal.get('orientation', None)
        yaw = goal.get('yaw', None)

        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2] if len(pos) > 2 else 0.0

        if ori:
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = ori
        elif yaw:
            q = yaw_to_quaternion(yaw)
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        else:
            pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        print(f"[ROUTE] Sending goal {self.current_goal_index+1}/{len(self.route_list)}")

        self._send_goal_future = self.ros.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # goal_handle = future.result()
        self._goal_handle = future.result()
        goal_handle = self._goal_handle


        if not goal_handle.accepted:
            self.status_label.setText("❌ Goal rejected.")
            self.route_active = False
            return
        self.status_label.setText("Navigating...")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        wrapped = future.result()
        print(f"[ROUTE] result_callback index BEFORE = {self.current_goal_index}")

        if wrapped is None:
            self.status_label.setText("❌ Result is None")
            self.route_active = False
            return

        status = wrapped.status
        result = wrapped.result
        
        print(f"[NAV] Result status = {status}")

        goal = self.route_list[self.current_goal_index]
        name = goal.get('name', f"Goal {self.current_goal_index}")

        if status == 4:  # SUCCEEDED
            self.status_label.setText(f"✅ Reached {name}")
        else:
            self.status_label.setText(f"⚠️ Failed at {name} (status={status})")
        # ---- emit signal ----
        gx, gy = goal["position"][:2]
        gq = goal.get("orientation", [0.0, 0.0, 0.0, 1.0])
        g_yaw = math.atan2(
            2.0 * (gq[3]*gq[2] + gq[0]*gq[1]),
            1.0 - 2.0 * (gq[1]**2 + gq[2]**2)
        )

        x_err = self.ros.last_x - gx
        y_err = self.ros.last_y - gy
        self.goal_reached_signal.emit(name, x_err, y_err, g_yaw)

        # ===== 🔴 BƯỚC 4: CHẶN KHI ROUTE KHÔNG CÒN ACTIVE =====
        if not self.route_active:
            print("[ROUTE] Route inactive → stop sending next goal")
            return

        # ---- GỬI GOAL TIẾP ----
        self.current_goal_index += 1
        print(f"[ROUTE] result_callback index AFTER = {self.current_goal_index}")
        self.send_next_goal()




    # ---------- History ----------
    def save_route_to_history(self):
        import datetime
        if not self.route_list:
            return
        names = [g.get('name', 'Goal') for g in self.route_list]
        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        route_entry = {'route': names, 'timestamp': now}
        data = {'history': []}
        if os.path.exists(self.history_path):
            try:
                with open(self.history_path, 'r') as f:
                    data = yaml.safe_load(f) or {'history': []}
            except Exception:
                data = {'history': []}
        data['history'].append(route_entry)
        with open(self.history_path, 'w') as f:
            yaml.safe_dump(data, f)

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return

        self.manual_active = True

        if not self.cmd_timer.isActive():
            self.cmd_timer.start(50)  # 20 Hz

        if event.key() == Qt.Key_W:
            self.cur_v = 0.25
            self.cur_w = 0.0
            self.status_label.setText("Manual: W")
        elif event.key() == Qt.Key_S:
            self.cur_v = -0.25
            self.cur_w = 0.0
            self.status_label.setText("Manual: S")
        elif event.key() == Qt.Key_A:
            self.cur_v = 0.0
            self.cur_w = 0.5
            self.status_label.setText("Manual: A")
        elif event.key() == Qt.Key_D:
            self.cur_v = 0.0
            self.cur_w = -0.5
            self.status_label.setText("Manual: D")


    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return

        self.cur_v = 0.0
        self.cur_w = 0.0
        self.manual_active = False

        self.cmd_timer.stop()

        msg = Twist()
        self.ros.cmd_pub.publish(msg)  # stop 1 lần

    def load_route_from_history(self, route_names):
        """
        route_names: list[str]
        """
        self.route_list.clear()

        for name in route_names:
            goal = next(
                (g for g in self.goals if g.get("name") == name),
                None
            )
            if goal:
                self.route_list.append(goal)

        if not self.route_list:
            QMessageBox.warning(self, "Route", "No valid goals found.")
            return

        names = [g.get("name") for g in self.route_list]
        self.route_display.setText(" → ".join(names))
        self.status_label.setText("Route loaded from history")



# ---------------- History Tab ----------------
class HistoryTab(QWidget):
    route_selected = pyqtSignal(list)  # danh sách tên waypoint

    def __init__(self, history_path):
        super().__init__()
        self.history_path = history_path
        layout = QVBoxLayout(self)
        title = QLabel("Select Route to Rerun")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        layout.addWidget(self.scroll)

        self.container = QWidget()
        self.vbox = QVBoxLayout(self.container)
        self.scroll.setWidget(self.container)

        btn_layout = QHBoxLayout()
        self.btn_refresh = QPushButton("🔄 Refresh")
        self.btn_clear = QPushButton("🗑️ Clear History")
        btn_layout.addWidget(self.btn_refresh)
        btn_layout.addWidget(self.btn_clear)
        layout.addLayout(btn_layout)

        self.btn_refresh.clicked.connect(self.load_history)
        self.btn_clear.clicked.connect(self.clear_history)

        self.load_history()

    def load_history(self):
        for i in reversed(range(self.vbox.count())):
            item = self.vbox.itemAt(i).widget()
            if item:
                item.deleteLater()

        if not os.path.exists(self.history_path):
            lbl = QLabel("No history file found.")
            self.vbox.addWidget(lbl)
            return

        try:
            with open(self.history_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            history = data.get('history', [])
            if not history:
                lbl = QLabel("No routes recorded yet.")
                lbl.setAlignment(Qt.AlignCenter)
                self.vbox.addWidget(lbl)
                return

            text = ""
            for i, entry in enumerate(history, 1):
                route_str = " → ".join(entry.get('route', []))
                time = entry.get('timestamp', '')
            for entry in history:
                route_names = entry.get('route', [])
                time = entry.get('timestamp', '')

                btn = QPushButton(" → ".join(route_names))
                btn.setToolTip(time)
                btn.setMinimumHeight(40)
                btn.clicked.connect(
                    lambda _, r=route_names: self.route_selected.emit(r)
                )

                self.vbox.addWidget(btn)

            lbl = QLabel(text)
            lbl.setStyleSheet("""
                color: #002b5c;
                background-color: #f0f6ff;
                padding: 10px;
                border-radius: 8px;
                font-size: 13px;
            """)
            lbl.setTextFormat(Qt.RichText)
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            self.vbox.addWidget(lbl)

        except Exception as e:
            lbl = QLabel(f"Error reading history: {e}")
            self.vbox.addWidget(lbl)

    def clear_history(self):
        confirm = QMessageBox.question(self, "Confirm", "Clear all history?")
        if confirm == QMessageBox.Yes:
            with open(self.history_path, 'w') as f:
                yaml.safe_dump({'history': []}, f)
            self.load_history()


# ---------------- Main Window ----------------
class MainWindow(QWidget):
    def __init__(self, ros_wrapper, goals_path, history_path):
        super().__init__()
        self.ros = ros_wrapper
        self.setWindowTitle("Mobile Robot GUI")
        self.resize(1000, 700)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # ===== STATUS BAR (TOP) =====
        self.status_bar = StatusBarWidget()
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status_bar)
        self.status_timer.start(66)  # ~15 Hz
        main_layout.addWidget(self.status_bar)

        # ===== TAB WIDGET =====
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs, 1)

        # ===== INTRODUCE TAB =====
        self.introduce_tab = IntroduceTab()
        self.tabs.addTab(self.introduce_tab, "Introduce")


        self.control_tab = ControlPanel(ros_wrapper, goals_path, history_path)
        self.history_tab = HistoryTab(history_path)

        self.tabs.addTab(self.control_tab, "Control")
 

  
        self.map_manager_tab = MapManagerTab(ros_wrapper, self.control_tab)
        self.map_manager_tab.waypoint_requested.connect(
            self.on_waypoint_requested,
            Qt.QueuedConnection
        )

        self.tabs.addTab(self.map_manager_tab, "Map Manager")

        from monitor_realtime_tab import MonitorTab
        self.monitor_tab = MonitorTab(ros_wrapper, self.control_tab)
        self.tabs.addTab(self.monitor_tab, "Monitor")

        self.tabs.addTab(self.history_tab, "History")

        self.history_tab.route_selected.connect(
            self.control_tab.load_route_from_history
        )

        # ===== ROS WORKER =====
        self.current_mode = "IDLE"
        self.ros_worker = RosWorker(ros_wrapper.node)

        ros_wrapper.node.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.ros_worker.odom_callback,
            10
        )

        ros_wrapper.node.create_subscription(
            Twist,
            "/cmd_vel",
            self.ros_worker.cmd_callback,
            10
        )

        self.ros_worker.signal_snapshot.connect(self.on_snapshot)
        self.ros_worker.signal_mode.connect(self.on_mode_changed)
        self.ros_worker.start()

        # self.monitor_tab.waypoint_requested.connect(
        #     self.on_waypoint_requested
        # )

    def on_waypoint_requested(self, x, y, yaw):
        print(f"[GUI] waypoint requested: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        name, ok = QInputDialog.getText(
            self,
            "Save Waypoint",
            "Enter waypoint name:"
        )

        if not ok or not name:
            return

        self.save_waypoint_to_yaml(name, x, y, yaw)
        # self.monitor_tab.draw_waypoint(x, y, yaw, name)
        wp = self.map_manager_tab.draw_waypoint(
            x, y, yaw,
            name=name,
            color="#3498db",
            source="runtime"
        )
        self.map_manager_tab.waypoints["runtime"].append(wp)


    def save_waypoint_to_yaml(self, name, x, y, yaw):
        yaml_path = "/home/nttien/ros2_ws/src/mobile/config/predefined_goals.yaml"

        data = {"goals": []}

        if os.path.exists(yaml_path):
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f) or {"goals": []}

        # remove goal trùng tên
        data["goals"] = [
            g for g in data["goals"]
            if g.get("name") != name
        ]

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        new_goal = {
            "name": name,
            "frame_id": "map",
            "position": [float(x), float(y), 0.0],
            "orientation": [0.0, 0.0, qz, qw]
        }

        data["goals"].append(new_goal)

        with open(yaml_path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

        print(f"[WAYPOINT SAVED] {name} @ ({x:.2f}, {y:.2f})")
        

    def on_snapshot(self, snapshot):
        self.status_bar.update_status(snapshot, self.current_mode)

    def on_mode_changed(self, mode):
        self.current_mode = mode
    
    def update_status_bar(self):
        snap = {
            "x": self.ros.last_x,
            "y": self.ros.last_y,
            "yaw": self.ros.last_yaw,
            "v": self.ros.last_v,
            "w": self.ros.last_w,
        }
        self.status_bar.update_status(snap, self.ros.mode)





# ---------------- Modern Style ----------------
def apply_modern_theme(app):
    app.setStyleSheet("""
        QWidget {
            background-color: #f4f9ff;
            color: #002b5c;
            font-family: 'Segoe UI';
        }
        QLabel {
            color: #003366;
        }
        QPushButton {
            background-color: #e7f0ff;
            border: 1px solid #0078d7;
            border-radius: 8px;
            padding: 6px 12px;
            color: #003366;
            font-weight: 600;
        }
        QPushButton:hover {
            background-color: #cce4ff;
        }
        QPushButton:pressed {
            background-color: #99ccff;
        }
        QPushButton#controlButton {
            background-color: #dceeff;
            border: 2px solid #0078d7;
            color: #002b5c;
            font-size: 14px;
            font-weight: bold;
        }
        QLabel#currentTarget {
            background-color: #fff0f0;
            color: #b22222;
            font-weight: bold;
            border: 1px solid #ffaaaa;
            border-radius: 6px;
            padding: 5px;
        }
        QLabel#routeDisplay {
            background-color: #f0f8ff;
            color: #004b91;
            border: 1px solid #99c4ff;
            border-radius: 6px;
            padding: 6px;
        }
        QTabBar::tab {
            background: #e8f1ff;
            border: 1px solid #0078d7;
            border-radius: 6px;
            padding: 6px 12px;
            min-width: 100px;
            min-height: 28px;
            color: #002b5c;
            font-size: 13px;
            font-weight: 600;
        }
        QTabBar::tab:selected {
            background: #0078d7;
            color: white;
            font-weight: 600;   /* không bold để không phình chữ */
        }
        QTabBar::tab:hover {
            background: #bcd8ff;
        }

    """)


# ---------------- Entry Point ----------------
if __name__ == "__main__":
    rclpy.init()
    ros_wrapper = RosNodeWrapper()

    goals_path = "/home/nttien/ros2_ws/src/mobile/config/predefined_goals.yaml"
    history_path = "/home/nttien/ros2_ws/src/mobile/config/history.yaml"

    app = QApplication(sys.argv)
    apply_modern_theme(app)

    window = MainWindow(ros_wrapper, goals_path, history_path)
    window.show()
    app.exec_()

    ros_wrapper.destroy()
    
