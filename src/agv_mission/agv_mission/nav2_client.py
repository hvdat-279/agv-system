import math


class Nav2Client:
    def __init__(self, node):
        self.node = node
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        self._action_client = ActionClient(
            node, NavigateToPose, 'navigate_to_pose'
        )
        self.goal_done = False
        self.goal_success = False
        self.goal_handle = None

    def send_goal(self, x, y, yaw):
        self.node.get_logger().info(
            f'Nav2Client: Sending goal (x={x}, y={y}, yaw={yaw})'
        )
        self.goal_done = False
        self.goal_success = False

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('Nav2 server not available!')
            self.goal_done = True
            return False

        from nav2_msgs.action import NavigateToPose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.node.get_logger().error('Nav2Client: Goal rejected!')
            self.goal_done = True
            self.goal_success = False
            return
        self.node.get_logger().info('Nav2Client: Goal accepted.')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        # action_msgs.msg.GoalStatus.STATUS_SUCCEEDED = 4
        if status == 4:
            self.goal_success = True
            self.node.get_logger().info(
                'Nav2Client: Goal reached successfully!'
            )
        else:
            self.goal_success = False
            self.node.get_logger().error(
                f'Nav2Client: Goal failed with status: {status}'
            )
        self.goal_done = True

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.node.get_logger().info('Nav2Client: Goal canceled.')
