from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QDoubleSpinBox,
    QPushButton
)


class WaypointDialog(QDialog):
    def __init__(self, x, y, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add Waypoint")
        self.setModal(True)

        self.result_data = None

        layout = QVBoxLayout(self)

        layout.addWidget(QLabel(f"x = {x:.2f}, y = {y:.2f}"))

        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("Waypoint name (e.g. C201A)")
        layout.addWidget(self.name_edit)

        yaw_layout = QHBoxLayout()
        yaw_layout.addWidget(QLabel("Yaw (rad):"))
        self.yaw_spin = QDoubleSpinBox()
        self.yaw_spin.setRange(-3.14, 3.14)
        self.yaw_spin.setSingleStep(0.1)
        yaw_layout.addWidget(self.yaw_spin)
        layout.addLayout(yaw_layout)

        btn_layout = QHBoxLayout()
        add_btn = QPushButton("ADD")
        cancel_btn = QPushButton("CANCEL")
        btn_layout.addWidget(add_btn)
        btn_layout.addWidget(cancel_btn)
        layout.addLayout(btn_layout)

        add_btn.clicked.connect(self.on_add)
        cancel_btn.clicked.connect(self.reject)

    def on_add(self):
        name = self.name_edit.text().strip()
        if not name:
            return

        self.result_data = {
            "name": name,
            "yaw": self.yaw_spin.value()
        }
        self.accept()
