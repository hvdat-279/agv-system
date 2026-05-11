from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GripperControl:
    def __init__(self, node):
        self.node = node
        # Tùy thuộc vào controller, nếu dùng joint_trajectory_controller cho gripper
        self._action_client = ActionClient(self.node, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.action_done = False
        self.action_success = False
        self._is_closed = False

    def send_trajectory(self, position, duration_sec=1.0):
        self.action_done = False
        self.action_success = False
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("Gripper controller server not available!")
            self.action_done = True
            return False

        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        # Thay đổi tên joint cho phù hợp với URDF
        trajectory.joint_names = ['gripper_left_joint', 'gripper_right_joint']

        point = JointTrajectoryPoint()
        point.positions = [position, position] # Trái và phải đối xứng
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Gripper goal rejected!')
            self.action_done = True
            self.action_success = False
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4: # SUCCEEDED
            self.action_success = True
        else:
            self.action_success = False
        self.action_done = True

    def close(self):
        self.node.get_logger().info("Gripper: Closing")
        self._is_closed = True
        return self.send_trajectory(0.015, duration_sec=1.0) # Vị trí đóng

    def open(self):
        self.node.get_logger().info("Gripper: Opening")
        self._is_closed = False
        return self.send_trajectory(0.0, duration_sec=1.0) # Vị trí mở

    def is_closed(self):
        return self._is_closed
