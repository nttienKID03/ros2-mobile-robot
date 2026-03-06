from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt


class StatusBarWidget(QWidget):
    def __init__(self):
        super().__init__()

        # ===== LABELS =====
        self.lbl_pose = QLabel("🧭 X:0.00  Y:0.00  θ:0.00")
        self.lbl_vel  = QLabel("🚀 v:0.00  ω:0.00")
        self.lbl_mode = QLabel("🟢 IDLE")

        for lbl in (self.lbl_pose, self.lbl_vel, self.lbl_mode):
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("""
                background-color: #eef5ff;
                border-radius: 6px;
                padding: 4px 10px;
                font-weight: 600;
            """)

        # ===== LAYOUT =====
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 4, 10, 4)
        layout.setSpacing(12)

        layout.addWidget(self.lbl_pose)
        layout.addWidget(self.lbl_vel)
        layout.addStretch(1)          # ⬅️ CỰC QUAN TRỌNG: tạo khoảng thở
        layout.addWidget(self.lbl_mode)

        self.setFixedHeight(42)

        # init
        self.update_status({}, "IDLE")

    def update_status(self, snapshot, mode):
        x = snapshot.get("x", 0.0)
        y = snapshot.get("y", 0.0)
        yaw = snapshot.get("yaw", 0.0)
        v = snapshot.get("v", 0.0)
        w = snapshot.get("w", 0.0)

        # ---- update text ----
        self.lbl_pose.setText(f"🧭 X:{x:.2f}  Y:{y:.2f}  θ:{yaw:.2f}")
        self.lbl_vel.setText(f"🚀 v:{v:.2f}  ω:{w:.2f}")

        # ---- mode coloring ----
        if mode == "MOTION":
            self.lbl_mode.setText("🟡 MOTION")
            self.lbl_mode.setStyleSheet("""
                background-color: #fff4cc;
                color: #b58900;
                border-radius: 6px;
                padding: 4px 12px;
                font-weight: 700;
            """)
        else:
            self.lbl_mode.setText("🟢 IDLE")
            self.lbl_mode.setStyleSheet("""
                background-color: #e8f8f0;
                color: #1e8449;
                border-radius: 6px;
                padding: 4px 12px;
                font-weight: 700;
            """)
