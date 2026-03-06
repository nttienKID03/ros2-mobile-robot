import os
import yaml
import numpy as np
from PIL import Image
from collections import deque

import rclpy
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseWithCovarianceStamped


from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer, QRectF
from PyQt5.QtGui import QPolygonF, QTransform, QBrush
from PyQt5.QtCore import QPointF

import pyqtgraph as pg
from PyQt5.QtCore import pyqtSignal, Qt

from waypoint_dialog import WaypointDialog
from PyQt5.QtWidgets import QPushButton, QHBoxLayout

from PyQt5.QtGui import QCursor

from geometry_msgs.msg import PolygonStamped, Point32

from PyQt5 import QtWidgets, QtCore

from nav2_msgs.srv import LoadMap


import yaml, os
import math



# ===== WHITE THEME =====
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')

class MapViewBox(pg.ViewBox):
    def __init__(self, monitor):
        super().__init__()
        self.monitor = monitor
        # self.dragging = False
        

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:

            items = self.scene().items(ev.scenePos())
            for item in items:
                if hasattr(item, "_waypoint_ref"):
                    super().mousePressEvent(ev)
                    return


            # 🔴 EDIT YAW ƯU TIÊN CAO NHẤT
            if self.monitor.mode == self.monitor.MODE_EDIT_YAW:
                self.monitor.dragging = True
                self.monitor.wp_edit_center = self.monitor.get_waypoint_position(
                    self.monitor.selected_waypoint
                )
                # self.monitor.yaw_text.show()
                ev.accept()
                return

            # 🟢 CHỈ SELECT KHI KHÔNG EDIT
            items = self.scene().items(ev.scenePos())
            for item in items:
                if hasattr(item, "_waypoint_ref"):
                    self.monitor.select_waypoint(item._waypoint_ref)
                    ev.accept()
                    return

        super().mousePressEvent(ev)


    def mouseMoveEvent(self, ev):
        if self.monitor.dragging and self.monitor.mode == self.monitor.MODE_EDIT_YAW:
            pos = self.mapSceneToView(ev.scenePos())
            self.monitor.update_selected_waypoint_yaw(pos.x(), pos.y())
            ev.accept()
            return
        super().mouseMoveEvent(ev)


    def mouseReleaseEvent(self, ev):
        if self.monitor.dragging and self.monitor.mode == self.monitor.MODE_EDIT_YAW:
            self.monitor.finish_edit_yaw()
            self.monitor.dragging = False
            ev.accept()
            return
        super().mouseReleaseEvent(ev)


class WaypointVisual:
    def __init__(self, point, arrow, label, source, color):
        self.point = point      # ScatterPlotItem
        self.arrow = arrow      # QGraphicsPolygonItem
        self.label = label      # TextItem | None
        self.source = source    # "yaml" | "runtime"
        self.color = color      # luu mau goc


class MapManagerTab(QWidget):
    MODE_NORMAL = 0
    MODE_ADD_MAP = 1
    MODE_ADD_ROBOT = 2   
    MODE_EDIT_YAW = 3 
    MODE_ADD_OBSTACLE_LINE = 4 
    waypoint_requested = pyqtSignal(float, float, float)
    def __init__(self, ros_wrapper, control_tab):
        super().__init__()




        self.mode = self.MODE_NORMAL


        self.ros = ros_wrapper
        self.control_tab = control_tab

        self.obstacle_points = []    # lưu các điểm nhấn chuột
        self.obstacles = []          # lưu QGraphicsLineItem của tất cả obstacle

        # ===== EDIT YAW STATE =====
        self.dragging = False
        self.wp_edit_center = None
        self.pending_yaw = 0.0


        # ===== DATA =====
        self.last_odom = None
        self.x_data = deque(maxlen=2000)
        self.y_data = deque(maxlen=2000)


        # ===== WAYPOINT REGISTRY =====
        self.waypoints = {
            "yaml": [],
            "runtime": []
        }

        # ===== SELECTION STATE =====
        self.selected_waypoint = None



        # ===== MAP PLOT =====
        self.map_plot = pg.PlotWidget()
        self.map_plot.setBackground('w')
        self.map_plot.hideAxis('bottom')
        self.map_plot.hideAxis('left')

        self.map_plot.setCursor(QCursor(Qt.CrossCursor))
        self.map_plot.setCursor(QCursor(Qt.ArrowCursor))



        self.map_plot.setStyleSheet("""
            border: 1px solid #cfdfff;
            border-radius: 8px;
        """)
        # ===== TOP BAR (MAP TOOLS) =====
        self.btn_add_map_wp = QPushButton("Add Waypoint (Map) 📍")
        self.btn_add_map_wp.setCheckable(True)
        self.btn_add_map_wp.setFixedHeight(36)
        self.btn_add_map_wp.setStyleSheet("""
            QPushButton {
                background-color: #e7f0ff;
                border: 1px solid #0078d7;
                border-radius: 8px;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #cce4ff;
            }
        """)

      

        # ===== TOOLBAR =====
        toolbar = QHBoxLayout()

        self.btn_add_wp_robot = QPushButton("Add Waypoint (Robot) 📍")
        self.btn_add_wp_robot.setFixedHeight(36)
        self.btn_add_wp_robot.clicked.connect(self.add_waypoint_from_robot)
        self.btn_add_map_wp.toggled.connect(self.toggle_add_map_mode)

        self.btn_delete_obstacle = QPushButton("Delete Obstacle Line 🗑")
        self.btn_delete_obstacle.setFixedHeight(36)
        self.btn_delete_obstacle.setCheckable(False)
        self.btn_delete_obstacle.clicked.connect(self.delete_selected_obstacle_line)
        toolbar.addWidget(self.btn_delete_obstacle)




        # DATA OBSTACLE
        self.obstacle_lines = []        # list các QGraphicsLineItem đang hiển thị
        self.obstacle_points = []       # điểm tạm thời khi click
        self.obstacle_ids = []          # id tương ứng với file

        from geometry_msgs.msg import PolygonStamped, Point32

        self.obstacle_pub = self.ros.node.create_publisher(
            PolygonStamped,
            "/gui_obstacle_lines",
            10)


        toolbar.addWidget(self.btn_add_wp_robot)
        toolbar.addStretch(1)

      


       

        # self.btn_edit_yaw = QPushButton("✏ Edit Yaw")
        # self.btn_edit_yaw.setFixedHeight(36)
        # self.btn_edit_yaw.setEnabled(False)
        # self.btn_edit_yaw.clicked.connect(self.enable_edit_yaw)
        self.btn_edit_yaw = QPushButton("Edit Yaw ✏")
        self.btn_edit_yaw.setCheckable(True)
        self.btn_edit_yaw.setEnabled(False)
        self.btn_edit_yaw.setFixedHeight(36)
        self.btn_edit_yaw.clicked.connect(self.toggle_edit_yaw_mode)


        toolbar.addWidget(self.btn_edit_yaw)

        self.btn_add_obstacle_line = QPushButton("Add Obstacle Line ➕")
        self.btn_add_obstacle_line.setCheckable(True)
        self.btn_add_obstacle_line.setFixedHeight(36)
        self.btn_add_obstacle_line.clicked.connect(self.toggle_add_obstacle_line)
        toolbar.addWidget(self.btn_add_obstacle_line)


        self.btn_delete_waypoint = QPushButton("Delete Waypoint 🗑")
        self.btn_delete_waypoint.clicked.connect(self.delete_selected_waypoint)

        toolbar.addWidget(self.btn_delete_waypoint)

        self.btn_export_map = QPushButton("Export Map GUI 🗺")
        self.btn_export_map.setFixedHeight(36)
        self.btn_export_map.clicked.connect(self.export_map_gui)


        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)
        self.setLayout(main_layout)
        main_layout.addWidget(self.map_plot, 1)

        # ===== FLOATING EXPORT MAP BUTTON (ON MAP) =====
        self.export_panel = QWidget(self.map_plot)
        self.export_panel.setFixedSize(140, 44)
        self.export_panel.setStyleSheet("""
            background-color: rgba(255, 255, 255, 200);
            border-radius: 10px;
        """)

        layout = QHBoxLayout(self.export_panel)
        layout.setContentsMargins(6, 6, 6, 6)

        self.btn_export_map = QPushButton("🗺 Export Map")
        self.btn_export_map.setCursor(Qt.PointingHandCursor)
        self.btn_export_map.clicked.connect(self.export_map_gui)

        self.btn_export_map.setStyleSheet("""
        QPushButton {
            background-color: #2d8cff;
            color: white;
            border-radius: 6px;
            font-weight: 600;
            padding: 6px;
        }
        QPushButton:hover {
            background-color: #1c6fd5;
        }
        """)

        layout.addWidget(self.btn_export_map)
        self.export_panel.show()

        bottom_panel = QtWidgets.QWidget()
        bottom_panel.setFixedHeight(90)

        grid = QtWidgets.QGridLayout(bottom_panel)
        grid.setContentsMargins(16, 8, 16, 8)   # 👈 TRÁI = PHẢI
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(6)

        # 🔑 3 cột giãn đều
        for col in range(3):
            grid.setColumnStretch(col, 1)

        buttons = [
            self.btn_add_map_wp,
            self.btn_add_wp_robot,
            self.btn_edit_yaw,
            self.btn_add_obstacle_line,
            self.btn_delete_waypoint,
            self.btn_delete_obstacle,
            # self.btn_export_map,
        ]

        for i, btn in enumerate(buttons):
            btn.setMinimumHeight(34)
            btn.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Expanding
            )
            grid.addWidget(btn, i // 3, i % 3)

        main_layout.addWidget(bottom_panel, 0)


        # ===== VIEW BOX =====
        # self.view_box = self.map_plot.getViewBox()
        self.view_box = MapViewBox(self)



        self.map_plot.setCentralItem(self.view_box)
        # self.view_box.addItem(self.yaw_text)

        # self.map_plot.getPlotItem().setViewBox(self.view_box)
        
        self.view_box.setAspectLocked(True)
        self.view_box.enableAutoRange(False)
        self.view_box.scene().sigMouseClicked.connect(self.on_map_clicked)

        # ===== YAW TEXT (PHẢI TẠO SỚM) =====
        self.yaw_text = pg.TextItem(
            text="",
            color=(200, 0, 0),
            anchor=(0.5, 1.5)
        )
        self.yaw_text.setZValue(40)
        self.yaw_text.hide()
        self.view_box.addItem(self.yaw_text)

        # ===== PATH & ROBOT =====
        self.path_curve = self.map_plot.plot(
            pen=pg.mkPen('#0078d7', width=2)
        )
        # self.robot_point = self.map_plot.plot(
        #     pen=None,
        #     symbol='o',
        #     symbolBrush='#e74c3c',
        #     symbolSize=10
        # )

        # ===== LOAD MAP =====
        map_yaml = "/home/nttien/ros2_ws/src/mobile/map/map.yaml"
        self.load_map(map_yaml)

        self.load_obstacle_lines_from_yaml()

        # ===== ROS SUB =====
        self.ros.node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.odom_callback,
            10
        )
        self.create_robot_item()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(66)  # ~15 Hz
        #=====Load Point===
        # self.yaml_waypoint_items = []   # giữ các item vẽ từ YAML
        self.goals_yaml_path = "/home/nttien/ros2_ws/src/mobile/config/predefined_goals.yaml"
        self.load_waypoints_from_yaml()

        self.load_map_client = self.ros.node.create_client(
            LoadMap,
            '/map_server/load_map'
        )

        




    # ======================================================
    # MAP LOADER
    # ======================================================
    def load_map(self, yaml_path):
        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        img_path = os.path.join(
            os.path.dirname(yaml_path),
            cfg['image']
        )

        resolution = cfg['resolution']
        origin_x, origin_y, _ = cfg['origin']

        img = Image.open(img_path).convert('L')
        data = np.array(img).astype(np.uint8)

        # đảo trục Y cho đúng hệ tọa độ map
        data = np.flipud(data)

        # Chuẩn hóa giống RViz:
        # 0   = occupied (đen)
        # 205 = unknown
        # 254 = free (trắng)

        vis = np.zeros_like(data)

        # free space
        vis[data >= 250] = 255

        # unknown
        vis[(data > 100) & (data < 250)] = 200

        # occupied
        vis[data <= 100] = 40

        data = vis



        h, w = data.shape

        self.map_item = pg.ImageItem(data)
        self.map_item.setOpts(axisOrder='row-major')

        self.map_item.setRect(
            QRectF(
                origin_x,
                origin_y,
                w * resolution,
                h * resolution
            )
        )

        self.view_box.addItem(self.map_item)
        self.map_item.setZValue(-10)

    # ======================================================
    # ODOM CALLBACK
    # ======================================================
    def odom_callback(self, msg):
        self.last_odom = msg

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.ros.last_x = x
        self.ros.last_y = y
        self.ros.last_yaw = yaw

        self.x_data.append(x)
        self.y_data.append(y)

        # chỉ update robot pose (KHÔNG vẽ path ở đây)
        self.update_robot_pose(x, y, yaw)



    # ======================================================
    # UPDATE PLOT
    # ======================================================
    def update_plot(self):
        if len(self.x_data) > 1:
            self.path_curve.setData(self.x_data, self.y_data)
            # self.robot_point.setData(
            #     [self.x_data[-1]],
            #     [self.y_data[-1]]
            # )


    def create_robot_item(self):
        # Robot arrow (chuẩn AMR)
        poly = QPolygonF([
            QPointF(0.45, 0.0),    # mũi
            QPointF(-0.25, 0.22),
            QPointF(-0.15, 0.0),
            QPointF(-0.25, -0.22)
        ])

        self.robot_item = pg.QtWidgets.QGraphicsPolygonItem(poly)

        # màu thân
        self.robot_item.setBrush(QBrush(pg.mkColor("#3498db")))  # xanh dương

        # viền
        self.robot_item.setPen(pg.mkPen("#1f4e79", width=1.5))

        self.robot_item.setZValue(30)
        self.view_box.addItem(self.robot_item)

    def update_robot_pose(self, x, y, yaw):
        t = QTransform()
        t.translate(x, y)
        t.rotateRadians(yaw)
        self.robot_item.setTransform(t)

    def draw_waypoint(self, x, y, yaw, name="", color="#1abc9c", source="runtime"):
        poly = QPolygonF([
            QPointF(0.45, 0.0),
            QPointF(-0.25, 0.2),
            QPointF(-0.15, 0.0),
            QPointF(-0.25, -0.2)
        ])

        arrow = pg.QtWidgets.QGraphicsPolygonItem(poly)
        arrow.setBrush(QBrush(pg.mkColor(color)))
        arrow.setPen(pg.mkPen("k", width=0.05))
        arrow.setZValue(16)

        arrow.setFlag(arrow.ItemIsSelectable, True)
        arrow.setAcceptHoverEvents(True)

        t = QTransform()
        t.translate(x, y)
        t.rotateRadians(yaw)
        arrow.setTransform(t)

        self.view_box.addItem(arrow)

        label = None
        if name:
            label = pg.TextItem(text=name, color=(0, 0, 0), anchor=(0.5, -0.8))
            label.setPos(x, y)
            label.setZValue(17)
            self.view_box.addItem(label)


        wp = WaypointVisual(
            point=None,
            arrow=arrow,
            label=label,
            source=source,
            color=color
        )
        arrow._waypoint_ref = wp



        arrow.setAcceptedMouseButtons(Qt.LeftButton)   # 🔑 CỰC KỲ QUAN TRỌNG

        # def on_arrow_click(ev, wp=wp):
        #     self.select_waypoint(wp)
        #     ev.accept()

        def on_arrow_press(ev, wp=wp):
            if self.mode != self.MODE_EDIT_YAW:
                self.select_waypoint(wp)
                return

            self.dragging = True
            self.wp_edit_center = self.get_waypoint_position(wp)
            self.pending_yaw = 0.0

            # Show yaw text ngay lập tức
            self.yaw_text.setText("0.0°")
            self.yaw_text.setPos(self.wp_edit_center[0], self.wp_edit_center[1])
            self.yaw_text.setZValue(50)
            self.yaw_text.show()

            ev.accept()




        def on_arrow_move(ev, wp=wp):
            if self.dragging and self.mode == self.MODE_EDIT_YAW:
                pos = self.view_box.mapSceneToView(ev.scenePos())
                self.update_selected_waypoint_yaw(pos.x(), pos.y())
                ev.accept()



        def on_arrow_release(ev, wp=wp):
            if self.dragging and self.mode == self.MODE_EDIT_YAW:
                self.finish_edit_yaw()
                self.dragging = False
                ev.accept()


        arrow.mousePressEvent = on_arrow_press
        arrow.mouseMoveEvent = on_arrow_move
        arrow.mouseReleaseEvent = on_arrow_release



        return wp




    def show_waypoint_preview(self, x, y):
        if hasattr(self, "preview_item"):
            self.view_box.removeItem(self.preview_item)

        self.preview_item = pg.ScatterPlotItem(
            [x], [y],
            symbol='o',
            size=14,
            brush=pg.mkBrush("#2ecc71"),
            pen=pg.mkPen("k", width=1)
        )

        self.preview_item.setZValue(20)
        self.view_box.addItem(self.preview_item)

    def on_map_clicked(self, event):
        pos = event.scenePos()
        if not self.view_box.sceneBoundingRect().contains(pos):
            return

        p = self.view_box.mapSceneToView(pos)
        x, y = p.x(), p.y()

        if self.mode == self.MODE_ADD_MAP:
            print(f"[MAP CLICK] x={x:.2f}, y={y:.2f}")
            self.waypoint_requested.emit(x, y, 0.0)
            # quay về mode thường
            self.mode = self.MODE_NORMAL
            self.btn_add_map_wp.setChecked(False)
            return

        if self.mode == self.MODE_ADD_OBSTACLE_LINE:
            pos = event.scenePos()
            if not self.view_box.sceneBoundingRect().contains(pos):
                return
            p = self.view_box.mapSceneToView(pos)
            x, y = p.x(), p.y()
            self.obstacle_points.append((x, y))
            print(f"[Obstacle Click] x={x:.2f}, y={y:.2f}")

            if len(self.obstacle_points) == 2:
                x0, y0 = self.obstacle_points[0]
                x1, y1 = self.obstacle_points[1]

                # 1️⃣ Vẽ line trên GUI
                line = pg.QtWidgets.QGraphicsLineItem(x0, y0, x1, y1)
                line.setPen(pg.mkPen("red", width=3))
                line.setZValue(50)
                line.setFlag(line.ItemIsSelectable, True)
                self.view_box.addItem(line)

                # tạo id line
                line_id = self.generate_new_line_id()
                self.obstacle_ids.append(line_id)
                self.obstacles.append(line)
                self.obstacle_lines.append(line)

                # 2️⃣ Lưu line vào YAML
                self.save_obstacle_line(line_id, x0, y0, x1, y1)

                # 3️⃣ Publish tới Nav2
                self.publish_obstacle_line(self.obstacle_points, line_id)

                # reset temp points
                self.obstacle_points = []


    # def request_add_waypoint(self, x, y):
    #     self.waypoint_requested.emit(x, y)


    def save_waypoint_to_yaml(self, name, x, y, yaw):
        yaml_path = "/home/nttien/ros2_ws/src/mobile/config/predefined_goals.yaml"

        if os.path.exists(yaml_path):
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}

        data[name] = {
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw)
        }

        with open(yaml_path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=True)

        print(f"[YAML SAVED] {name}")

    def start_waypoint_drag(self, x, y):
        self.mode = self.MODE_EDIT_YAW   # 🔴 BẮT BUỘC
        self.dragging = True

        self.wp_start = (x, y)
        self.pending_yaw = 0.0

        poly = QPolygonF([
            QPointF(0.45, 0.0),
            QPointF(-0.25, 0.2),
            QPointF(-0.15, 0.0),
            QPointF(-0.25, -0.2)
        ])

        self.wp_item = pg.QtWidgets.QGraphicsPolygonItem(poly)
        self.wp_item.setBrush(QBrush(pg.mkColor("#2ecc71")))
        self.wp_item.setPen(pg.mkPen("#1e8449", width=1.5))
        self.wp_item.setZValue(25)

        t = QTransform()
        t.translate(x, y)
        self.wp_item.setTransform(t)

        self.view_box.addItem(self.wp_item)


    def update_waypoint_yaw(self, x, y):
        x0, y0 = self.wp_start
        yaw = math.atan2(y - y0, x - x0)

        t = QTransform()
        t.translate(x0, y0)
        t.rotateRadians(yaw)   # 🔴 đảo cho đúng hệ

        self.wp_item.setTransform(t)
        self.pending_yaw = yaw


    def finish_waypoint_drag(self):
        x, y = self.wp_start
        yaw = self.pending_yaw

        print(f"[WAYPOINT SET] x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        # ✅ 1. VẼ WAYPOINT RUNTIME
        wp = self.draw_waypoint(
            x, y, yaw,
            name="",              # runtime chưa cần tên
            color="#2ecc71",
            source="runtime"
        )

        self.waypoints["runtime"].append(wp)

        # ✅ 2. AUTO SELECT
        self.select_waypoint(wp)

        # ✅ 3. GỌI POPUP SAVE (luồng cũ)
        self.waypoint_requested.emit(x, y, yaw)

        self.mode = self.MODE_NORMAL
        self.btn_add_map_wp.setChecked(False)


    def add_waypoint_from_robot(self):
        x = self.ros.last_x
        y = self.ros.last_y
        yaw = self.ros.last_yaw

        print(f"[ADD WP ROBOT] x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        # gọi ngược về MainWindow (luồng cũ)
        self.waypoint_requested.emit(x, y, yaw)


    def load_waypoints_from_yaml(self):
        if not os.path.exists(self.goals_yaml_path):
            print("[MapManager] No predefined_goals.yaml found")
            return

        with open(self.goals_yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}

        goals = data.get("goals", [])
        print(f"[MapManager] Loading {len(goals)} waypoints from YAML")

        for goal in goals:
            self.draw_yaml_waypoint(goal)

    def draw_yaml_waypoint(self, goal):
        name = goal.get("name", "WP")
        pos = goal.get("position", [0.0, 0.0])
        ori = goal.get("orientation", [0, 0, 0, 1])

        x, y = pos[0], pos[1]
        qz, qw = ori[2], ori[3]

        yaw = math.atan2(
            2.0 * qw * qz,
            1.0 - 2.0 * qz * qz
        )

        wp = self.draw_waypoint(
            x, y, yaw,
            name=name,
            color="orange",
            source="yaml"
        )

        self.waypoints["yaml"].append(wp)

    def enable_add_from_map(self):
        self.mode = self.MODE_ADD_MAP
        print("[MapManager] MODE_ADD_MAP enabled")


    def toggle_add_map_mode(self, checked):
        if checked:
            self.mode = self.MODE_ADD_MAP
            self.btn_add_map_wp.setText("✖ Exit Add Waypoint")
            self.btn_add_map_wp.setStyleSheet("""
                QPushButton {
                    background-color: #ffecec;
                    border: 1px solid #e74c3c;
                    border-radius: 8px;
                    font-weight: 600;
                    color: #c0392b;
                }
            """)
            print("[MapManager] MODE_ADD_MAP enabled")
        else:
            self.mode = self.MODE_NORMAL
            self.btn_add_map_wp.setText("📍 Add Waypoint (Map)")
            self.btn_add_map_wp.setStyleSheet("")
            print("[MapManager] MODE_NORMAL")

    def on_waypoint_clicked(self, plot, points):
        if not points:
            return

        point = points[0]
        wp = getattr(point, "_waypoint_ref", None)
        if not wp:
            return

        self.select_waypoint(wp)

    def select_waypoint(self, wp):
        # bỏ highlight cũ
        if self.selected_waypoint:
            self.set_waypoint_highlight(self.selected_waypoint, False)

        self.selected_waypoint = wp
        self.set_waypoint_highlight(wp, True)

        self.btn_edit_yaw.setEnabled(True)

        if self.mode == self.MODE_EDIT_YAW:
            self.wp_edit_center = self.get_waypoint_position(wp)
            self.pending_yaw = None
            self.dragging = False
            print("[EditYaw] Selected new waypoint")

        print(f"[MapManager] Selected waypoint ({wp.source})")




    # def set_waypoint_highlight(self, wp, highlight: bool):
    #     if highlight:
    #         wp.point.setSize(18)
    #         wp.point.setBrush(pg.mkBrush("#f1c40f"))   # vàng
    #         wp.arrow.setBrush(QBrush(pg.mkColor("#f39c12")))
    #     else:
    #         wp.point.setSize(12)

    #         # trả lại màu theo source
    #         if wp.source == "yaml":
    #             color = "orange"
    #         else:
    #             color = "#3498db"

    #         wp.point.setBrush(pg.mkBrush(color))
    #         wp.arrow.setBrush(QBrush(pg.mkColor(color)))

    def enable_edit_yaw(self):
        if not self.selected_waypoint:
            return

        if self.mode == self.MODE_EDIT_YAW:
            self.mode = self.MODE_NORMAL
            print("[MapManager] EXIT MODE_EDIT_YAW")
            return

        self.mode = self.MODE_EDIT_YAW
        self.dragging = False
        print("[MapManager] MODE_EDIT_YAW enabled")



    def get_waypoint_position(self, wp):
        t = wp.arrow.transform()
        return t.dx(), t.dy()

    def update_selected_waypoint_yaw(self, x, y):
        wp = self.selected_waypoint
        x0, y0 = self.wp_edit_center

        yaw = math.atan2(y - y0, x - x0)

        t = QTransform()
        t.translate(x0, y0)
        t.rotateRadians(yaw)
        wp.arrow.setTransform(t)

        if wp.label:
            wp.label.setPos(x0, y0)

        yaw_deg = math.degrees(yaw)

        self.yaw_text.setText(f"{yaw_deg:.1f}°")
        self.yaw_text.setPos(x0, y0)
        self.yaw_text.show()   # ✅ SHOW Ở ĐÂY

        self.pending_yaw = yaw



    def finish_edit_yaw(self):
        wp = self.selected_waypoint
        yaw = self.pending_yaw

        print(f"[EditYaw] yaw={math.degrees(yaw):.1f} deg")

        if wp.source == "yaml":
            self.update_yaml_waypoint_yaw(wp, yaw)

        self.yaw_text.hide()
        self.mode = self.MODE_NORMAL
        self.exit_edit_yaw_mode()

    def update_yaml_waypoint_yaw(self, wp, yaw):
        name = wp.label.toPlainText()
        path = self.goals_yaml_path

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        for g in data.get("goals", []):
            if g.get("name") == name:
                g["orientation"][2] = math.sin(yaw / 2.0)
                g["orientation"][3] = math.cos(yaw / 2.0)
                break

        with open(path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

        print(f"[YAML] Updated yaw for {name}")

    # def toggle_edit_yaw_mode(self, checked):
    #     if not self.selected_waypoint:
    #         self.btn_edit_yaw.setChecked(False)
    #         return

    #     if checked:
    #         self.mode = self.MODE_EDIT_YAW
    #         self.dragging = False
    #         self.wp_edit_center = self.get_waypoint_position(self.selected_waypoint)
    #         self.pending_yaw = 0.0

    #         # --- Hiển thị yaw ngay lập tức ---
    #         wp = self.selected_waypoint
    #         x0, y0 = self.wp_edit_center
    #         t = wp.arrow.transform()
    #         yaw = 0.0  # nếu muốn hiển thị yaw hiện tại của waypoint, có thể tính từ transform
    #         self.pending_yaw = yaw

    #         yaw_deg = math.degrees(yaw)
    #         self.yaw_text.setText(f"{yaw_deg:.1f}°")
    #         self.yaw_text.setPos(x0, y0)
    #         self.yaw_text.setZValue(50)
    #         self.yaw_text.show()
    #         # -------------------------------

    #         self.btn_edit_yaw.setText("✖ Exit Edit Yaw")
    #         self.btn_edit_yaw.setStyleSheet("""
    #             QPushButton {
    #                 background-color: #fff3cd;
    #                 border: 1px solid #f39c12;
    #                 border-radius: 8px;
    #                 font-weight: 600;
    #                 color: #7d6608;
    #             }
    #             QPushButton:hover {
    #                 background-color: #ffe8a1;
    #             }
    #         """)
    #         print("[MapManager] MODE_EDIT_YAW enabled")
    #     else:
    #         self.exit_edit_yaw_mode()

    def toggle_edit_yaw_mode(self, checked):
        if not self.selected_waypoint:
            self.btn_edit_yaw.setChecked(False)
            return

        if checked:
            self.mode = self.MODE_EDIT_YAW
            self.dragging = False
            self.wp_edit_center = self.get_waypoint_position(self.selected_waypoint)

            # 🔑 LẤY YAW TỪ QUATERNION HOẶC TỪ TRANSFORM
            wp = self.selected_waypoint

            # Nếu waypoint load từ YAML, lấy từ orientation
            if wp.source == "yaml":
                name = wp.label.toPlainText()
                with open(self.goals_yaml_path, "r") as f:
                    data = yaml.safe_load(f)
                for g in data.get("goals", []):
                    if g.get("name") == name:
                        qz = g["orientation"][2]
                        qw = g["orientation"][3]
                        yaw = math.atan2(2 * qw * qz, 1 - 2 * qz * qz)
                        break
            else:
                # runtime: lấy từ transform
                t = wp.arrow.transform()
                yaw = math.atan2(t.m21(), t.m11())

            self.pending_yaw = yaw

            # Hiển thị yaw
            yaw_deg = math.degrees(yaw)
            self.yaw_text.setText(f"{yaw_deg:.1f}°")
            x0, y0 = self.wp_edit_center
            self.yaw_text.setPos(x0, y0)
            self.yaw_text.setZValue(50)
            self.yaw_text.show()

            # Update nút
            self.btn_edit_yaw.setText("✖ Exit Edit Yaw")
            self.btn_edit_yaw.setStyleSheet("""
                QPushButton {
                    background-color: #fff3cd;
                    border: 1px solid #f39c12;
                    border-radius: 8px;
                    font-weight: 600;
                    color: #7d6608;
                }
                QPushButton:hover {
                    background-color: #ffe8a1;
                }
            """)
            print("[MapManager] MODE_EDIT_YAW enabled")
        else:
            self.exit_edit_yaw_mode()


    def exit_edit_yaw_mode(self):
        self.mode = self.MODE_NORMAL
        self.dragging = False

        self.btn_edit_yaw.setText("✏ Edit Yaw")
        self.btn_edit_yaw.setStyleSheet("")
        self.btn_edit_yaw.setChecked(False)

        # Ẩn yaw_text
        self.yaw_text.hide()

        print("[MapManager] EXIT MODE_EDIT_YAW")


    def set_waypoint_highlight(self, wp, highlight: bool):
        if highlight:
            wp.arrow.setBrush(QBrush(pg.mkColor("#f1c40f")))  # vàng
            wp.arrow.setPen(pg.mkPen("#c0392b", width=0.2))  # viền đậm
            wp.arrow.setZValue(25)
        else:
            wp.arrow.setBrush(QBrush(pg.mkColor(wp.color)))
            wp.arrow.setPen(pg.mkPen("k", width=0.05))
            wp.arrow.setZValue(16)

    def toggle_add_obstacle_line(self, checked):
        if checked:
            self.mode = self.MODE_ADD_OBSTACLE_LINE
            self.obstacle_points = []
            # Feedback trực quan
            self.btn_add_obstacle_line.setText("✖ Exit Obstacle Mode")
            self.btn_add_obstacle_line.setStyleSheet("""
                QPushButton {
                    background-color: #ffecec;
                    border: 1px solid #e74c3c;
                    border-radius: 8px;
                    font-weight: 600;
                    color: #c0392b;
                }
                QPushButton:hover {
                    background-color: #ffcaca;
                }
            """)
            print("[MapManager] MODE_ADD_OBSTACLE_LINE enabled")
        else:
            self.mode = self.MODE_NORMAL
            self.btn_add_obstacle_line.setText("➕ Add Obstacle Line")
            self.btn_add_obstacle_line.setStyleSheet("")
            print("[MapManager] MODE_NORMAL")

    def publish_obstacle_line(self, line_points, line_id, delete=False):
        poly_msg = PolygonStamped()
        poly_msg.header.stamp = self.ros.node.get_clock().now().to_msg()
        poly_msg.header.frame_id = "map"
        for x, y in line_points:
            pt = Point32()
            pt.x = x
            pt.y = y
            pt.z = 0.0
            poly_msg.polygon.points.append(pt)
        # dùng header.frame_id hoặc thêm field khác để báo delete
        if delete:
            # Nav2 plugin có thể check flag này để remove line
            poly_msg.header.frame_id = "delete_line"
        self.obstacle_pub.publish(poly_msg)
        print(f"[Obstacle] {'Deleted' if delete else 'Published'} line {line_id}")

    OBSTACLE_YAML_PATH = "/home/nttien/ros2_ws/src/mobile/config/obstacle_line.yaml"

    def save_obstacle_line(self, line_id, x0, y0, x1, y1):
        if os.path.exists(self.OBSTACLE_YAML_PATH):
            with open(self.OBSTACLE_YAML_PATH, "r") as f:
                data = yaml.safe_load(f) or {"lines": []}
        else:
            data = {"lines": []}

        data["lines"].append({
            "id": line_id,
            "points": [[x0, y0], [x1, y1]]
        })

        with open(self.OBSTACLE_YAML_PATH, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

        print(f"[Obstacle] Saved line {line_id}")


    def delete_obstacle_line(self, line_id):
        if not os.path.exists(self.OBSTACLE_YAML_PATH):
            return

        with open(self.OBSTACLE_YAML_PATH, "r") as f:
            data = yaml.safe_load(f) or {"lines": []}

        data["lines"] = [l for l in data["lines"] if l["id"] != line_id]

        with open(self.OBSTACLE_YAML_PATH, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

        print(f"[Obstacle] Deleted line {line_id} from YAML")


    def delete_selected_obstacle_line(self):
        selected = self.view_box.scene().selectedItems()

        if not selected:
            print("[Obstacle] No obstacle line selected")
            return

        for item in selected:
            if item in self.obstacle_lines:
                idx = self.obstacle_lines.index(item)
                line_id = self.obstacle_ids[idx]

                self.view_box.removeItem(item)
                self.obstacle_lines.pop(idx)
                self.obstacle_ids.pop(idx)

                self.delete_obstacle_line(line_id)
                self.publish_obstacle_line([], line_id, delete=True)

                print(f"[Obstacle] Deleted line {line_id}")
                break


    def load_obstacle_lines_from_yaml(self):
        if not os.path.exists(self.OBSTACLE_YAML_PATH):
            print("[Obstacle] No obstacle YAML found")
            return

        with open(self.OBSTACLE_YAML_PATH, "r") as f:
            data = yaml.safe_load(f) or {"lines": []}

        print(f"[Obstacle] Loading {len(data['lines'])} lines from YAML")

        for line_data in data.get("lines", []):
            points = line_data["points"]
            x0, y0 = points[0]
            x1, y1 = points[1]
            line_id = line_data["id"]

            line = pg.QtWidgets.QGraphicsLineItem(x0, y0, x1, y1)
            line.setPen(pg.mkPen("red", width=3))
            line.setZValue(50)
            line.setFlag(line.ItemIsSelectable, True)
            self.view_box.addItem(line)

            self.obstacle_lines.append(line)
            self.obstacles.append(line)
            self.obstacle_ids.append(line_id)

            # publish line tới robot
            self.publish_obstacle_line(points, line_id)

    def generate_new_line_id(self):
        # lấy số max từ các line_id hiện tại
        max_id = 0
        for lid in self.obstacle_ids:
            try:
                n = int(lid.split("_")[1])
                max_id = max(max_id, n)
            except:
                continue
        return f"line_{max_id + 1}"

    def delete_selected_waypoint(self):
        wp = self.selected_waypoint
        if not wp:
            print("[Waypoint] No waypoint selected")
            return

        # 1️⃣ remove arrow
        self.view_box.removeItem(wp.arrow)

        # 2️⃣ remove label
        if wp.label:
            self.view_box.removeItem(wp.label)

        # 3️⃣ remove khỏi registry
        self.waypoints[wp.source].remove(wp)

        # 4️⃣ remove YAML nếu cần
        if wp.source == "yaml":
            name = wp.label.toPlainText()
            self.delete_yaml_waypoint_by_name(name)

        self.selected_waypoint = None
        self.btn_edit_yaw.setEnabled(False)

        print("[Waypoint] Deleted")


    def delete_waypoint_from_file(self, wp_id):
        import yaml, os

        path = "/home/nttien/ros2_ws/src/mobile/config/waypoints.yaml"

        if not os.path.exists(path):
            return

        with open(path, "r") as f:
            data = yaml.safe_load(f) or {"waypoints": []}

        data["waypoints"] = [
            w for w in data["waypoints"] if w.get("id") != wp_id
        ]

        with open(path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

        print(f"[Waypoint] Removed {wp_id} from YAML")

    def delete_yaml_waypoint_by_name(self, name):
        path = self.goals_yaml_path

        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}

        data["goals"] = [g for g in data.get("goals", []) if g.get("name") != name]

        with open(path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

        print(f"[YAML] Deleted waypoint {name}")

    def export_map_gui(self):
        print("[ExportMap] Exporting map_gui...")

        MAP_YAML_SRC = "/home/nttien/ros2_ws/src/mobile/map/map.yaml"
        MAP_OUT_DIR  = "/home/nttien/ros2_ws/src/mobile/map"
        MAP_NAME     = "map_gui"

        # 1️⃣ Load map gốc
        with open(MAP_YAML_SRC, "r") as f:
            cfg = yaml.safe_load(f)

        resolution = cfg["resolution"]
        origin = cfg["origin"]

        img_path = os.path.join(os.path.dirname(MAP_YAML_SRC), cfg["image"])
        img = Image.open(img_path).convert("L")
        data = np.array(img)

        h, w = data.shape

        # 2️⃣ Vẽ obstacle line lên map (đen = occupied)
        for line in self.obstacle_lines:
            x0 = line.line().x1()
            y0 = line.line().y1()
            x1 = line.line().x2()
            y1 = line.line().y2()

            self.draw_line_on_map(data, x0, y0, x1, y1, resolution, origin)

        # 3️⃣ Save PGM
        out_pgm = os.path.join(MAP_OUT_DIR, MAP_NAME + ".pgm")
        Image.fromarray(data).save(out_pgm)

        # 4️⃣ Save YAML
        out_yaml = os.path.join(MAP_OUT_DIR, MAP_NAME + ".yaml")
        cfg["image"] = MAP_NAME + ".pgm"

        with open(out_yaml, "w") as f:
            yaml.safe_dump(cfg, f)

        print(f"[ExportMap] DONE → {out_yaml}")

        self.reload_nav2_map(out_yaml)


    def draw_line_on_map(self, img, x0, y0, x1, y1, resolution, origin):
        ox, oy, _ = origin

        def world_to_map(x, y):
            mx = int((x - ox) / resolution)
            my = int((y - oy) / resolution)
            return mx, my

        mx0, my0 = world_to_map(x0, y0)
        mx1, my1 = world_to_map(x1, y1)

        # Bresenham line
        dx = abs(mx1 - mx0)
        dy = abs(my1 - my0)
        sx = 1 if mx0 < mx1 else -1
        sy = 1 if my0 < my1 else -1
        err = dx - dy

        while True:
            if 0 <= my0 < img.shape[0] and 0 <= mx0 < img.shape[1]:
                img[img.shape[0] - my0 - 1, mx0] = 0  # occupied

            if mx0 == mx1 and my0 == my1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                mx0 += sx
            if e2 < dx:
                err += dx
                my0 += sy

    def resizeEvent(self, event):
        super().resizeEvent(event)
        margin = 12
        self.export_panel.move(
            self.map_plot.width() - self.export_panel.width() - margin,
            self.map_plot.height() - self.export_panel.height() - margin
        )

    def reload_nav2_map(self, map_yaml_path):
        if not self.load_map_client.wait_for_service(timeout_sec=2.0):
            print("[Nav2] map_server/load_map not available")
            return

        req = LoadMap.Request()
        req.map_url = map_yaml_path

        future = self.load_map_client.call_async(req)
        rclpy.spin_until_future_complete(self.ros.node, future)

        if future.result() and future.result().result == 0:
            print("[Nav2] Map reloaded successfully")
        else:
            print("[Nav2] Failed to reload map")
