import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sdp_interfaces.action import Follow
from sdp_interfaces.srv import MotorEnable

from .api_client import APIClient
from .transformations import euler_from_quaternion

from .command_statuses import CommandStatus
from .task_types import TaskTypes
from .robot_states import RobotStates


class RobotStateController(Node):

    def __init__(self):
        super().__init__('robot_state_controller')

        self.robot_state = RobotStates.Init
        self.serial_number = None
        self.battery = 100 # int [0-100] 
        self.blocked = False
        self.aborted = False
        self.current_command_id = None

        self.api_client = APIClient(self.serial_number)

        self.robot_pose = None

        # Client for the Navigation action serverc
        self._follow_action_client = ActionClient(self, Follow, 'follow')

        # Follow Testing 
        # self.create_timer(10, self.send_follow_goal)
        self.send_follow_goal()


        # Client for Motor Switch service
        self._motor_switch_client = self.create_client(MotorEnable, 'motor_switch')

        # Robot pose subscriber
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry', self.odometry_callback, 10)


    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        rotation = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        self.robot_pose = [x, y, rotation[2]]


    def switch_off_motors(self):
        # wait for the service to become available
        while not self._motor_switch_client.wait_for_service(timeout_sec=1.0):
            continue
        
        request = MotorEnable.Request()
        request.enabled = False

        future = self._motor_switch_client.call_async(request)
        future.add_done_callback(self.motor_switch_off_done_callback)


    def motor_switch_off_done_callback(self, future):
        are_motors_enabled = future.result().enabled
        if not are_motors_enabled:
            self.get_logger().info('Motors are off!')
        else: 
            self.get_logger().info('Not managed to shut the motors! Repeating...')
            self.switch_off_motors()
            

    def switch_on_motors(self):
        # wait for the service to become available
        while not self._motor_switch_client.wait_for_service(timeout_sec=1.0):
            continue
        
        request = MotorEnable.Request()
        request.enabled = True

        future = self._motor_switch_client.call_async(request)
        future.add_done_callback(self.motor_switch_on_done_callback)


    def motor_switch_on_done_callback(self, future):
        are_motors_enabled = future.result().enabled
        if are_motors_enabled:
            self.get_logger().info('Motors are running!')
        else: 
            self.get_logger().info('Not managed to turn on the motors! Repeating...')
            self.switch_on_motors()
            

    def send_follow_goal(self):
        goal_msg = Follow.Goal()

        # GENERATE THE PATH HERE
        goal_msg.path = [-2., -8., -2., -4., 0., -4., 0., -8., 2., -8., 2., -4.]

        self._follow_action_client.wait_for_server()

        self._send_follow_goal_future = self._follow_action_client.send_goal_async(goal_msg)

        self._send_follow_goal_future.add_done_callback(self.follow_goal_response_callback)


    # Executes when NavNode accepts/rejects the goal
    def follow_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Follow goal rejected :(')
            return

        self.get_logger().info('Follow goal accepted :)')

        self._get_follow_result_future = goal_handle.get_result_async()
        self._get_follow_result_future.add_done_callback(self.get_follow_result_callback)

    def get_follow_result_callback(self, future):
        nav_status_code = future.result().result.status
        if nav_status_code == 0:
            self.get_logger().info('Robot has successfully followed the path!')
        else:
            self.get_logger().info('Robot has failed to follow the path!')
            self.get_logger().info('Nav status code: {}'.format(nav_status_code))


def main(args=None):
    rclpy.init(args=args)

    robot_state_controller = RobotStateController()

    rclpy.spin(robot_state_controller)


if __name__ == '__main__':
    main()