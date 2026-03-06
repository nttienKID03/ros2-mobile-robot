from collections import deque
import math
import time

class TrackingModel:
    def __init__(self, max_path_length=1000):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.0
        self.w = 0.0

        self.path = deque(maxlen=max_path_length)
        self.last_update_time = time.time()

    def update_pose(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

        if not self.path or math.hypot(
            x - self.path[-1][0],
            y - self.path[-1][1]
        ) > 0.02:
            self.path.append((x, y))

        self.last_update_time = time.time()

    def update_velocity(self, v, w):
        self.v = v
        self.w = w

    def snapshot(self):
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "v": self.v,
            "w": self.w,
            "path": list(self.path),
            "timestamp": self.last_update_time
        }
