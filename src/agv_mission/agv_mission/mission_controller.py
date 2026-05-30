import math

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, TwistStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Empty, String
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
        self.attach_pubs = {
            'red': self.create_publisher(
                Empty, '/model/agv/attach_red', 10
            ),
            'blue': self.create_publisher(
                Empty, '/model/agv/attach_blue', 10
            ),
            'yellow': self.create_publisher(
                Empty, '/model/agv/attach_yellow', 10
            ),
        }
        self.detach_pubs = {
            'red': self.create_publisher(
                Empty, '/model/agv/detach_red', 10
            ),
            'blue': self.create_publisher(
                Empty, '/model/agv/detach_blue', 10
            ),
            'yellow': self.create_publisher(
                Empty, '/model/agv/detach_yellow', 10
            ),
        }
        self.carried_color = None
        self.teleport_process = None

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10
        )
        self.state_pub = self.create_publisher(
            String, '/mission_state', 10
        )

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
        self.x_start = 0.0
        self.y_start = 0.0
        self.teleport_counter = 0

        # Start command
        self.start_sub = self.create_subscription(
            Empty, '/start_mission', self.start_callback, 10
        )
        self.start_requested = False

        # FSM Timer
        self.timer = self.create_timer(0.1, self.run_fsm)
        # Publish state timer (10 Hz) để perception luôn biết trạng thái
        self.state_timer = self.create_timer(0.1, self._publish_current_state)
        self.get_logger().info(
            'Mission Controller Initialized. Waiting for /start_mission'
        )

    def _publish_current_state(self):
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)

    def start_callback(self, msg):
        self.get_logger().info('Received start mission command!')
        self.start_requested = True

    def transition_to(self, new_state):
        self.current_state = new_state
        self.state_initialized = False
        self.get_logger().info(f'Transitioned to: {new_state}')
        msg = String()
        msg.data = new_state
        self.state_pub.publish(msg)

    def _now(self):
        return self.get_clock().now()

    def _elapsed(self, start_time):
        if start_time is None:
            return 0.0
        return (self._now() - start_time).nanoseconds / 1e9

    def drive(self, vx, wz):
        msg = TwistStamped()
        msg.header.stamp = self._now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(vx)
        msg.twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed in get_robot_pose: {e}')
            return None

    def run_fsm(self):
        # Teleport cargo to follow the fork, but NOT during unload/retreat
        # to avoid physics instability when dropping
        if self.carried_color and self.current_state not in (
            MissionState.UNLOAD_CARGO, MissionState.RETREAT_SORT
        ):
            self.teleport_counter += 1
            # Reduced frequency: every 5 ticks (~500ms) instead of every 2
            if self.teleport_counter % 5 == 0:
                self.publish_cargo_at_fork(self.carried_color)

        if self.current_state == MissionState.IDLE:
            self.handle_idle()
        elif self.current_state == MissionState.NAV_TO_PICKUP:
            self.handle_nav_to_pickup()
        elif self.current_state == MissionState.DETECT_COLOR:
            self.handle_detect_color()
        elif self.current_state == MissionState.APPROACH_CARGO:
            self.handle_approach_cargo()
        elif self.current_state == MissionState.LOAD_CARGO:
            self.handle_load_cargo()
        elif self.current_state == MissionState.RETREAT_PICKUP:
            self.handle_retreat_pickup()
        elif self.current_state == MissionState.NAV_TO_SORT:
            self.handle_nav_to_sort()
        elif self.current_state == MissionState.APPROACH_SORT:
            self.handle_approach_sort()
        elif self.current_state == MissionState.UNLOAD_CARGO:
            self.handle_unload_cargo()
        elif self.current_state == MissionState.RETREAT_SORT:
            self.handle_retreat_sort()
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
            self.transition_to(MissionState.APPROACH_CARGO)

    def handle_approach_cargo(self):
        if not self.state_initialized:
            self.fork_step = 0
            self.action_start_time = self._now()
            self.state_initialized = True
            pose = self.get_robot_pose()
            if pose is not None:
                self.x_start, self.y_start, _ = pose
            else:
                self.x_start, self.y_start = -1.0, -2.0
            self.get_logger().info('Approaching cargo shelf...')

        # Step 0: lower the fork to lift height (0.0)
        if self.fork_step == 0:
            if self.send_fork_goal(self.get_parameter('fork_lift_down').value):
                self.fork_step = 1
        elif self.fork_step == 1:
            if self.fork_goal_done:
                if not self.fork_goal_success:
                    self.get_logger().error('Fork lower failed during approach. Aborting.')
                    self.transition_to(MissionState.IDLE)
                    return
                # Start driving forward
                self.fork_step = 2
                self.action_start_time = self._now()  # Reset timer for the drive phase
        elif self.fork_step == 2:
            pose = self.get_robot_pose()
            dist = 0.0
            if pose is not None:
                x, y, _ = pose
                dist = ((x - self.x_start)**2 + (y - self.y_start)**2)**0.5

            elapsed = self._elapsed(self.action_start_time)
            # Stop after 0.15 meters of travel or 8.0 seconds timeout
            if dist >= 0.15 or elapsed > 8.0:
                self.drive(0.0, 0.0)  # Stop
                self.transition_to(MissionState.LOAD_CARGO)
            else:
                self.drive(0.08, 0.0)  # Drive forward slowly (local +x)

    def handle_load_cargo(self):
        if not self.state_initialized:
            self.fork_step = 0
            self.state_initialized = True
            self.get_logger().info('Loading cargo with forklift...')
            self.carried_color = self.detected_color

        if self.fork_step == 0:
            if self.send_fork_goal(self.get_parameter('fork_lift_up').value):
                self.fork_step = 1
        elif self.fork_step == 1:
            if self.fork_goal_done:
                if self.fork_goal_success:
                    self.get_logger().info('Loading complete! Attaching cargo.')
                    # Teleport cargo to fork tip first
                    self.publish_cargo_at_fork(self.carried_color)
                    # Publish attach command to Gazebo DetachableJoint plugin
                    self.attach_pubs[self.carried_color].publish(Empty())
                    self.transition_to(MissionState.RETREAT_PICKUP)
                else:
                    self.get_logger().error('Fork lift failed. Aborting load.')
                    self.transition_to(MissionState.IDLE)

    def handle_retreat_pickup(self):
        if not self.state_initialized:
            self.action_start_time = self._now()
            self.state_initialized = True
            pose = self.get_robot_pose()
            if pose is not None:
                self.x_start, self.y_start, _ = pose
            else:
                self.x_start, self.y_start = -1.45, -2.0
            self.get_logger().info('Retreating from shelf...')

        pose = self.get_robot_pose()
        dist = 0.0
        if pose is not None:
            x, y, _ = pose
            dist = ((x - self.x_start)**2 + (y - self.y_start)**2)**0.5

        elapsed = self._elapsed(self.action_start_time)
        # Drive back 0.20m or timeout
        if dist >= 0.20 or elapsed > 8.0:
            self.drive(0.0, 0.0)  # Stop
            # Lower the fork slightly to travel height for safety during transport
            self.send_fork_goal(self.get_parameter('fork_travel_height').value)
            self.transition_to(MissionState.NAV_TO_SORT)
        else:
            self.drive(-0.08, 0.0)  # Drive backward (local -x)

    def handle_nav_to_sort(self):
        if not self.state_initialized:
            if self.detected_color == 'red':
                x_sort = self.get_parameter('red_sort_x').value
                y_sort = self.get_parameter('red_sort_y').value
                # Red station is on the right (+x), approach from left, face east
                goal_x = x_sort - 1.0
                goal_y = y_sort
                goal_yaw = 0.0
            elif self.detected_color == 'blue':
                x_sort = self.get_parameter('blue_sort_x').value
                y_sort = self.get_parameter('blue_sort_y').value
                # Blue station is on the left (-x), approach from right, face west
                goal_x = x_sort + 1.0
                goal_y = y_sort
                goal_yaw = 3.14159
            else:  # yellow or unknown
                x_sort = self.get_parameter('yellow_sort_x').value
                y_sort = self.get_parameter('yellow_sort_y').value
                # Yellow station is at the bottom (-y), approach from top, face south
                goal_x = x_sort
                goal_y = y_sort + 1.0
                goal_yaw = -1.5708

            self.nav2_client.send_goal(goal_x, goal_y, goal_yaw)
            self.state_initialized = True

        if self.nav2_client.goal_done:
            if self.nav2_client.goal_success:
                self.transition_to(MissionState.APPROACH_SORT)
            else:
                self.get_logger().error('Failed to navigate to sort station!')
                self.transition_to(MissionState.IDLE)

    def handle_approach_sort(self):
        if not self.state_initialized:
            self.action_start_time = self._now()
            self.state_initialized = True
            pose = self.get_robot_pose()
            if pose is not None:
                self.x_start, self.y_start, _ = pose
            else:
                if self.detected_color == 'red':
                    self.x_start, self.y_start = 2.0, 3.0
                elif self.detected_color == 'blue':
                    self.x_start, self.y_start = -2.0, 3.0
                else:
                    self.x_start, self.y_start = 0.0, -2.0
            self.get_logger().info('Approaching sorting station...')

        pose = self.get_robot_pose()
        should_stop = False
        if pose is not None:
            x, y, _ = pose
            if self.detected_color == 'red':
                if x >= 2.30:
                    should_stop = True
            elif self.detected_color == 'blue':
                if x <= -2.30:
                    should_stop = True
            else: # yellow
                if y <= -2.30:
                    should_stop = True

        dist = 0.0
        if pose is not None:
            dist = ((x - self.x_start)**2 + (y - self.y_start)**2)**0.5

        elapsed = self._elapsed(self.action_start_time)
        # Drive forward to a safe absolute pose (10cm away from table) to overlap forks,
        # but stop if we exceed distance/time to prevent infinite driving
        if should_stop or dist >= 0.50 or elapsed > 10.0:
            self.drive(0.0, 0.0)
            # Publish stop multiple times to ensure diff_drive receives it
            for _ in range(3):
                self.drive(0.0, 0.0)
            self.transition_to(MissionState.UNLOAD_CARGO)
        else:
            self.drive(0.08, 0.0)

    def handle_unload_cargo(self):
        if not self.state_initialized:
            self.fork_step = 0
            self.state_initialized = True
            self.get_logger().info('Unloading cargo with forklift...')

        # Keep sending stop command every tick to prevent drift
        self.drive(0.0, 0.0)

        if self.fork_step == 0:
            if self.send_fork_goal(self.get_parameter('fork_lift_down').value):
                self.fork_step = 1
        elif self.fork_step == 1:
            if self.fork_goal_done:
                if not self.fork_goal_success:
                    self.get_logger().error('Fork lower failed. Aborting unload.')
                    self.transition_to(MissionState.IDLE)
                    return
                # Stop cargo tracking FIRST to prevent teleport interference
                color_to_drop = self.carried_color
                self.carried_color = None
                # Detach the cargo physically in Gazebo (non-blocking)
                self.drop_cargo(color_to_drop)
                # Lift the forks slightly to clear the table
                if self.send_fork_goal(self.get_parameter('fork_travel_height').value):
                    self.fork_step = 2
        elif self.fork_step == 2:
            if self.fork_goal_done:
                if self.fork_goal_success:
                    self.get_logger().info('Unloading complete!')
                    self.transition_to(MissionState.RETREAT_SORT)
                else:
                    self.get_logger().error('Fork raise failed after unload.')
                    self.transition_to(MissionState.IDLE)

    def handle_retreat_sort(self):
        if not self.state_initialized:
            self.action_start_time = self._now()
            self.state_initialized = True
            pose = self.get_robot_pose()
            if pose is not None:
                self.x_start, self.y_start, _ = pose
            else:
                if self.detected_color == 'red':
                    self.x_start, self.y_start = 2.30, 3.0
                elif self.detected_color == 'blue':
                    self.x_start, self.y_start = -2.30, 3.0
                else:
                    self.x_start, self.y_start = 0.0, -2.30
            self.get_logger().info('Retreating from sorting station...')

        pose = self.get_robot_pose()
        should_stop = False
        if pose is not None:
            x, y, _ = pose
            if self.detected_color == 'red':
                if x <= 1.85:
                    should_stop = True
            elif self.detected_color == 'blue':
                if x >= -1.85:
                    should_stop = True
            else: # yellow
                if y >= -1.85:
                    should_stop = True

        dist = 0.0
        if pose is not None:
            dist = ((x - self.x_start)**2 + (y - self.y_start)**2)**0.5

        elapsed = self._elapsed(self.action_start_time)
        # Drive back to a safe absolute pose (well away from the table),
        # or stop if we exceed distance/time to prevent infinite driving
        if should_stop or dist >= 0.60 or elapsed > 10.0:
            self.drive(0.0, 0.0)
            for _ in range(3):
                self.drive(0.0, 0.0)
            self.transition_to(MissionState.RETURN)
        else:
            self.drive(-0.08, 0.0)

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

    def drop_cargo(self, color=None):
        if color is None:
            color = self.carried_color
        if color and color in self.cargo_pubs:
            # 1. Publish detach command multiple times for reliability
            for _ in range(3):
                self.detach_pubs[color].publish(Empty())
            self.get_logger().info(f'Detached cargo {color} from forks.')

            # 2. Teleport cargo to the exact table surface (NON-BLOCKING)
            # Poses of sorting tables: Red: 3.0, 3.0, 0.20 | Blue: -3.0, 3.0, 0.20
            # Yellow: 0.0, -3.0, 0.20
            if color == 'red':
                tx, ty, tz = 3.0, 3.0, 0.20
            elif color == 'blue':
                tx, ty, tz = -3.0, 3.0, 0.20
            else:
                tx, ty, tz = 0.0, -3.0, 0.20

            import subprocess
            cmd = [
                'gz', 'service', '-s', '/world/warehouse/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--req', (
                    f'name: "cargo_{color}_1", '
                    f'position: {{x: {tx}, y: {ty}, z: {tz}}}, '
                    f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'
                )
            ]
            # Use Popen instead of run to avoid blocking the node
            subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            self.get_logger().info(f'Placed cargo {color} on sorting table.')

    def publish_cargo_at_fork(self, color):
        try:
            tip_frame = self.get_parameter('fork_tip_frame').value
            offset_z = float(self.get_parameter('cargo_offset_z').value)
            t = self.tf_buffer.lookup_transform(
                'map', tip_frame, rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z + offset_z
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w

            self.get_logger().info(
                f'Teleporting cargo {color} to: x={x:.3f}, y={y:.3f}, z={z:.3f}'
            )

            if self.teleport_process is None or self.teleport_process.poll() is not None:
                import subprocess
                cmd = [
                    'gz', 'service', '-s', '/world/warehouse/set_pose',
                    '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean',
                    '--req', (
                        f'name: "cargo_{color}_1", '
                        f'position: {{x: {x}, y: {y}, z: {z}}}, '
                        f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
                    )
                ]
                self.teleport_process = subprocess.Popen(
                    cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )

            # Keep publishing to the ROS topic for compatibility
            msg = Pose()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw
            self.cargo_pubs[color].publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to set cargo pose: {e}')

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

    def destroy_node(self):
        if self.teleport_process and self.teleport_process.poll() is None:
            try:
                self.teleport_process.kill()
            except Exception:
                pass
        super().destroy_node()


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
