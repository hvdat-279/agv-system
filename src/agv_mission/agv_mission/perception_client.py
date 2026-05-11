import time
from agv_interfaces.msg import DetectedCargo

class PerceptionClient:
    def __init__(self, node):
        self.node = node
        self.subscriber = self.node.create_subscription(
            DetectedCargo,
            '/detected_cargo',
            self.cargo_callback,
            10
        )
        self.latest_cargo = None
        self.last_seen_time = 0.0

    def cargo_callback(self, msg):
        if msg.detected:
            self.latest_cargo = msg
            self.last_seen_time = time.time()

    def get_detection(self, timeout=30.0):
        # We check if we have a recent detection within the timeout
        if self.latest_cargo is not None:
            time_since_seen = time.time() - self.last_seen_time
            if time_since_seen < timeout:
                return self.latest_cargo
        return None

    def is_cargo_detected(self):
        # Return True if we saw cargo in the last 2 seconds
        if self.latest_cargo is not None:
            time_since_seen = time.time() - self.last_seen_time
            return time_since_seen < 2.0
        return False
