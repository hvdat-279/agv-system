import math
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmControl:
    def __init__(self, node):
        self.node = node
        self._action_client = ActionClient(self.node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.action_done = False
        self.action_success = False

    def send_trajectory(self, joint_positions, duration_sec=2.0):
        self.action_done = False
        self.action_success = False
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("Arm controller server not available!")
            self.action_done = True
            return False

        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4']

        point = JointTrajectoryPoint()
        point.positions = joint_positions
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
            self.node.get_logger().error('Arm goal rejected!')
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

    def move_to_grasp(self, distance, angle):
        self.node.get_logger().info(f"Arm: Moving to grasp (dist={distance}, angle={angle})")
        # Giả lập tính toán IK (Inverse Kinematics) đơn giản
        j1 = angle
        j2 = 0.5
        j3 = -1.0
        j4 = 0.5
        return self.send_trajectory([j1, j2, j3, j4], duration_sec=3.0)

    def move_to_release(self):
        self.node.get_logger().info("Arm: Moving to release")
        return self.send_trajectory([0.0, 0.2, -0.5, 0.3], duration_sec=2.0)

    def move_home(self):
        self.node.get_logger().info("Arm: Moving home")
        return self.send_trajectory([0.0, 0.0, 0.0, 0.0], duration_sec=2.0)
