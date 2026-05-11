import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import time
import math

from .states import MissionState
from .nav2_client import Nav2Client
from .perception_client import PerceptionClient
from .arm_control import ArmControl
from .gripper_control import GripperControl

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # Parameters
        self.declare_parameter('pickup_goal_x', -2.0)
        self.declare_parameter('pickup_goal_y', -2.0)
        self.declare_parameter('pickup_goal_yaw', 0.0)
        
        self.declare_parameter('red_sort_x', -3.0)
        self.declare_parameter('red_sort_y', -3.0)
        self.declare_parameter('blue_sort_x', -3.0)
        self.declare_parameter('blue_sort_y', 3.0)
        self.declare_parameter('yellow_sort_x', 3.0)
        self.declare_parameter('yellow_sort_y', -3.0)
        
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)
        
        self.declare_parameter('detect_timeout', 30.0)
        self.declare_parameter('grasp_retries', 3)
        self.declare_parameter('nav_timeout', 60.0)
        
        # Clients
        self.nav2_client = Nav2Client(self)
        self.perception_client = PerceptionClient(self)
        self.arm_client = ArmControl(self)
        self.gripper_client = GripperControl(self)
        
        # State FSM
        self.current_state = MissionState.IDLE
        self.previous_state = None
        self.state_initialized = False
        self.sub_state = 0
        
        # Variables
        self.detected_color = ""
        self.detected_distance = 0.0
        self.detected_angle = 0.0
        self.grasp_attempts = 0
        self.detect_start_time = 0.0
        
        # Start command
        self.start_sub = self.create_subscription(Empty, '/start_mission', self.start_callback, 10)
        self.start_requested = False
        
        # FSM Timer
        self.timer = self.create_timer(0.1, self.run_fsm)
        self.get_logger().info("Mission Controller Initialized. Waiting for /start_mission")

    def start_callback(self, msg):
        self.get_logger().info("Received start mission command!")
        self.start_requested = True

    def transition_to(self, new_state):
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_initialized = False
        self.sub_state = 0
        self.get_logger().info(f"Transitioned to: {new_state}")

    def run_fsm(self):
        if self.current_state == MissionState.IDLE:
            self.handle_idle()
        elif self.current_state == MissionState.NAV_TO_PICKUP:
            self.handle_nav_to_pickup()
        elif self.current_state == MissionState.DETECT:
            self.handle_detect()
        elif self.current_state == MissionState.GRASP:
            self.handle_grasp()
        elif self.current_state == MissionState.NAV_TO_SORT:
            self.handle_nav_to_sort()
        elif self.current_state == MissionState.RELEASE:
            self.handle_release()
        elif self.current_state == MissionState.RETURN:
            self.handle_return()

    def handle_idle(self):
        if self.start_requested:
            self.start_requested = False
            self.transition_to(MissionState.NAV_TO_PICKUP)

    def handle_nav_to_pickup(self):
        if not self.state_initialized:
            x = self.get_parameter('pickup_goal_x').value
            y = self.get_parameter('pickup_goal_y').value
            yaw = self.get_parameter('pickup_goal_yaw').value
            self.nav2_client.send_goal(x, y, yaw)
            self.state_initialized = True
            
        if self.nav2_client.goal_done:
            if self.nav2_client.goal_success:
                self.transition_to(MissionState.DETECT)
            else:
                self.get_logger().error("Failed to navigate to pickup!")
                self.transition_to(MissionState.IDLE)

    def handle_detect(self):
        if not self.state_initialized:
            self.detect_start_time = time.time()
            self.state_initialized = True
            self.get_logger().info("Searching for cargo...")
            
        timeout = self.get_parameter('detect_timeout').value
        if time.time() - self.detect_start_time > timeout:
            self.get_logger().error("Detection timeout! No cargo found.")
            self.transition_to(MissionState.IDLE)
            return

        cargo = self.perception_client.get_detection(timeout=2.0)
        if cargo is not None:
            self.get_logger().info(f"Found {cargo.color} cargo at dist: {cargo.distance}")
            self.detected_color = cargo.color
            self.detected_distance = cargo.distance
            self.detected_angle = cargo.angle
            self.transition_to(MissionState.GRASP)

    def handle_grasp(self):
        if not self.state_initialized:
            self.grasp_attempts += 1
            max_retries = self.get_parameter('grasp_retries').value
            if self.grasp_attempts > max_retries:
                self.get_logger().error("Max grasp retries reached! Aborting.")
                self.grasp_attempts = 0
                self.transition_to(MissionState.IDLE)
                return
            
            self.sub_state = 1
            self.arm_client.move_to_grasp(self.detected_distance, self.detected_angle)
            self.state_initialized = True
            
        if self.sub_state == 1 and self.arm_client.action_done:
            if self.arm_client.action_success:
                self.sub_state = 2
                self.gripper_client.close()
            else:
                self.get_logger().warn("Arm move to grasp failed, retrying...")
                self.state_initialized = False # retry
                
        if self.sub_state == 2 and self.gripper_client.action_done:
            if self.gripper_client.action_success:
                self.get_logger().info("Grasp successful!")
                self.grasp_attempts = 0
                # Nâng tay lên một chút trước khi chạy
                self.sub_state = 3
                self.arm_client.move_home()
            else:
                self.state_initialized = False
                
        if self.sub_state == 3 and self.arm_client.action_done:
             self.transition_to(MissionState.NAV_TO_SORT)

    def handle_nav_to_sort(self):
        if not self.state_initialized:
            if self.detected_color == "red":
                x = self.get_parameter('red_sort_x').value
                y = self.get_parameter('red_sort_y').value
            elif self.detected_color == "blue":
                x = self.get_parameter('blue_sort_x').value
                y = self.get_parameter('blue_sort_y').value
            else: # yellow or unknown
                x = self.get_parameter('yellow_sort_x').value
                y = self.get_parameter('yellow_sort_y').value
                
            self.nav2_client.send_goal(x, y, 0.0)
            self.state_initialized = True
            
        if self.nav2_client.goal_done:
            if self.nav2_client.goal_success:
                self.transition_to(MissionState.RELEASE)
            else:
                self.get_logger().error("Failed to navigate to sort station!")
                self.transition_to(MissionState.IDLE)

    def handle_release(self):
        if not self.state_initialized:
            self.sub_state = 1
            self.arm_client.move_to_release()
            self.state_initialized = True
            
        if self.sub_state == 1 and self.arm_client.action_done:
            self.sub_state = 2
            self.gripper_client.open()
            
        if self.sub_state == 2 and self.gripper_client.action_done:
            self.sub_state = 3
            self.arm_client.move_home()
            
        if self.sub_state == 3 and self.arm_client.action_done:
            self.get_logger().info("Release successful!")
            self.transition_to(MissionState.RETURN)

    def handle_return(self):
        if not self.state_initialized:
            x = self.get_parameter('home_x').value
            y = self.get_parameter('home_y').value
            yaw = self.get_parameter('home_yaw').value
            self.nav2_client.send_goal(x, y, yaw)
            self.state_initialized = True
            
        if self.nav2_client.goal_done:
            if self.nav2_client.goal_success:
                self.get_logger().info("Mission Complete!")
            else:
                self.get_logger().error("Failed to return home.")
            self.transition_to(MissionState.IDLE)

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
