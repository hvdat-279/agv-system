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
        self.last_seen_time = None

    def cargo_callback(self, msg):
        if msg.detected:
            self.latest_cargo = msg
            self.last_seen_time = self.node.get_clock().now()

    def get_detection(self, timeout=30.0):
        # We check if we have a recent detection within the timeout
        if self.latest_cargo is not None and self.last_seen_time is not None:
            time_since_seen = self.node.get_clock().now() - self.last_seen_time
            if (time_since_seen.nanoseconds / 1e9) < timeout:
                return self.latest_cargo
        return None

    def is_cargo_detected(self):
        # Return True if we saw cargo in the last 2 seconds
        if self.latest_cargo is not None and self.last_seen_time is not None:
            time_since_seen = self.node.get_clock().now() - self.last_seen_time
            return (time_since_seen.nanoseconds / 1e9) < 2.0
        return False
