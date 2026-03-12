#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('robot_description_publisher')
        self.declare_parameter('robot_description', '')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self._publisher = self.create_publisher(String, '/robot_description', qos)
        self._published = False
        self._timer = self.create_timer(1.0, self._publish_once)

    def _publish_once(self) -> None:
        if self._published:
            return

        description = self.get_parameter('robot_description').value
        if not description:
            self.get_logger().error('robot_description parameter is empty')
            return

        msg = String()
        msg.data = description
        self._publisher.publish(msg)
        self.get_logger().info('Published robot_description')
        self._published = True
        self._timer.cancel()


def main() -> None:
    rclpy.init()
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
