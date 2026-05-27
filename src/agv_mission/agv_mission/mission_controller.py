from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Empty
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .nav2_client import Nav2Client
from .perception_client import PerceptionClient
from .states import MissionState


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
        self.declare_parameter('fork_joint_name', 'base_to_fork_lift')
        self.declare_parameter('fork_tip_frame', 'fork_tip')
        self.declare_parameter('fork_lift_up', 0.12)
        self.declare_parameter('fork_lift_down', 0.0)
        self.declare_parameter('fork_travel_height', 0.04)
        self.declare_parameter('fork_move_time', 2.0)
        self.declare_parameter('cargo_offset_z', 0.1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cargo_pubs = {
            'red': self.create_publisher(
                Pose, '/model/cargo_red_1/cmd_pose', 10
            ),
            'blue': self.create_publisher(
                Pose, '/model/cargo_blue_1/cmd_pose', 10
            ),
            'yellow': self.create_publisher(
                Pose, '/model/cargo_yellow_1/cmd_pose', 10
            ),
        }
        self.carried_color = None

        # Clients
        self.nav2_client = Nav2Client(self)
        self.perception_client = PerceptionClient(self)
        self.fork_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/lifter_controller/follow_joint_trajectory',
        )

        # State FSM
        self.current_state = MissionState.IDLE
        self.state_initialized = False

        # Variables
        self.detected_color = ''
        self.detect_start_time = None
        self.action_start_time = None
        self.fork_goal_done = True
        self.fork_goal_success = False
        self.fork_step = 0

        # Start command
        self.start_sub = self.create_subscription(
            Empty, '/start_mission', self.start_callback, 10
        )
        self.start_requested = False

        # FSM Timer
        self.timer = self.create_timer(0.1, self.run_fsm)
        self.get_logger().info(
            'Mission Controller Initialized. Waiting for /start_mission'
        )

    def start_callback(self, msg):
        self.get_logger().info('Received start mission command!')
        self.start_requested = True

    def transition_to(self, new_state):
        self.current_state = new_state
        self.state_initialized = False
        self.get_logger().info(f'Transitioned to: {new_state}')

    def _now(self):
        return self.get_clock().now()

    def _elapsed(self, start_time):
        if start_time is None:
            return 0.0
        return (self._now() - start_time).nanoseconds / 1e9

    def run_fsm(self):
        if self.carried_color and self.carried_color in self.cargo_pubs:
            self.publish_cargo_at_fork(self.carried_color)

        if self.current_state == MissionState.IDLE:
            self.handle_idle()
        elif self.current_state == MissionState.NAV_TO_PICKUP:
            self.handle_nav_to_pickup()
        elif self.current_state == MissionState.DETECT_COLOR:
            self.handle_detect_color()
        elif self.current_state == MissionState.LOAD_CARGO:
            self.handle_load_cargo()
        elif self.current_state == MissionState.NAV_TO_SORT:
            self.handle_nav_to_sort()
        elif self.current_state == MissionState.UNLOAD_CARGO:
            self.handle_unload_cargo()
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
                self.transition_to(MissionState.DETECT_COLOR)
            else:
                self.get_logger().error('Failed to navigate to pickup!')
                self.transition_to(MissionState.IDLE)

    def handle_detect_color(self):
        if not self.state_initialized:
            self.detect_start_time = self._now()
            self.state_initialized = True
            self.get_logger().info('Searching for cargo...')

        timeout = self.get_parameter('detect_timeout').value
        if self._elapsed(self.detect_start_time) > timeout:
            self.get_logger().error('Detection timeout! No cargo found.')
            self.transition_to(MissionState.IDLE)
            return

        cargo = self.perception_client.get_detection(timeout=2.0)
        if cargo is not None:
            self.get_logger().info(
                f'Found {cargo.color} cargo at dist: {cargo.distance}'
            )
            self.detected_color = cargo.color
            self.transition_to(MissionState.LOAD_CARGO)

    def handle_load_cargo(self):
        if not self.state_initialized:
            self.fork_step = 0
            self.state_initialized = True
            self.get_logger().info('Loading cargo with forklift...')

        if self.fork_step == 0:
            if self.send_fork_goal(
                self.get_parameter('fork_lift_down').value
            ):
                self.fork_step = 1
        elif self.fork_step == 1:
            if self.fork_goal_done:
                if not self.fork_goal_success:
                    self.get_logger().error(
                        'Fork lower failed. Aborting load.'
                    )
                    self.transition_to(MissionState.IDLE)
                    return
                if self.send_fork_goal(
                    self.get_parameter('fork_lift_up').value
                ):
                    self.fork_step = 2
        elif self.fork_step == 2:
            if self.fork_goal_done:
                if self.fork_goal_success:
                    self.get_logger().info('Loading complete!')
                    self.carried_color = self.detected_color
                    self.transition_to(MissionState.NAV_TO_SORT)
                else:
                    self.get_logger().error(
                        'Fork lift failed. Aborting load.'
                    )
                    self.transition_to(MissionState.IDLE)

    def handle_nav_to_sort(self):
        if not self.state_initialized:
            if self.detected_color == 'red':
                x = self.get_parameter('red_sort_x').value
                y = self.get_parameter('red_sort_y').value
            elif self.detected_color == 'blue':
                x = self.get_parameter('blue_sort_x').value
                y = self.get_parameter('blue_sort_y').value
            else:  # yellow or unknown
                x = self.get_parameter('yellow_sort_x').value
                y = self.get_parameter('yellow_sort_y').value

            self.nav2_client.send_goal(x, y, 0.0)
            self.state_initialized = True

        if self.nav2_client.goal_done:
            if self.nav2_client.goal_success:
                self.transition_to(MissionState.UNLOAD_CARGO)
            else:
                self.get_logger().error(
                    'Failed to navigate to sort station!'
                )
                self.transition_to(MissionState.IDLE)

    def handle_unload_cargo(self):
        if not self.state_initialized:
            self.fork_step = 0
            self.state_initialized = True
            self.get_logger().info('Unloading cargo with forklift...')

        if self.fork_step == 0:
            if self.send_fork_goal(
                self.get_parameter('fork_lift_down').value
            ):
                self.fork_step = 1
        elif self.fork_step == 1:
            if self.fork_goal_done:
                if not self.fork_goal_success:
                    self.get_logger().error(
                        'Fork lower failed. Aborting unload.'
                    )
                    self.transition_to(MissionState.IDLE)
                    return
                self.drop_cargo()
                if self.send_fork_goal(
                    self.get_parameter('fork_travel_height').value
                ):
                    self.fork_step = 2
        elif self.fork_step == 2:
            if self.fork_goal_done:
                if self.fork_goal_success:
                    self.get_logger().info('Unloading complete!')
                    self.carried_color = None
                    self.transition_to(MissionState.RETURN)
                else:
                    self.get_logger().error(
                        'Fork raise failed after unload.'
                    )
                    self.transition_to(MissionState.IDLE)

    def drop_cargo(self):
        if self.carried_color and self.carried_color in self.cargo_pubs:
            self.publish_cargo_at_fork(self.carried_color)

    def publish_cargo_at_fork(self, color):
        try:
            tip_frame = self.get_parameter('fork_tip_frame').value
            offset_z = float(self.get_parameter('cargo_offset_z').value)
            t = self.tf_buffer.lookup_transform(
                'map', tip_frame, rclpy.time.Time()
            )
            msg = Pose()
            msg.position.x = t.transform.translation.x
            msg.position.y = t.transform.translation.y
            msg.position.z = t.transform.translation.z + offset_z
            msg.orientation = t.transform.rotation
            self.cargo_pubs[color].publish(msg)
        except tf2_ros.LookupException as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().debug(f'TF extrapolation failed: {e}')

    def send_fork_goal(self, position):
        move_time = float(self.get_parameter('fork_move_time').value)
        joint_name = self.get_parameter('fork_joint_name').value
        if not self.fork_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Fork controller not available.')
            self.fork_goal_done = True
            self.fork_goal_success = False
            return False

        traj = JointTrajectory()
        traj.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(seconds=move_time).to_msg()
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.fork_goal_done = False
        self.fork_goal_success = False
        self._fork_goal_future = self.fork_client.send_goal_async(goal)
        self._fork_goal_future.add_done_callback(
            self.fork_goal_response_callback
        )
        return True

    def fork_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Fork trajectory rejected.')
            self.fork_goal_done = True
            self.fork_goal_success = False
            return
        self._fork_result_future = goal_handle.get_result_async()
        self._fork_result_future.add_done_callback(self.fork_result_callback)

    def fork_result_callback(self, future):
        status = future.result().status
        self.fork_goal_success = (status == 4)
        self.fork_goal_done = True

    def handle_return(self):
        if not self.state_initialized:
            x = self.get_parameter('home_x').value
            y = self.get_parameter('home_y').value
            yaw = self.get_parameter('home_yaw').value
            self.nav2_client.send_goal(x, y, yaw)
            self.state_initialized = True

        if self.nav2_client.goal_done:
            if self.nav2_client.goal_success:
                self.get_logger().info('Mission Complete!')
            else:
                self.get_logger().error('Failed to return home.')
            self.transition_to(MissionState.IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
