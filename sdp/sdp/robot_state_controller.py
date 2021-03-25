import asyncio
import json
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from .patterns import Point, gen_circular, gen_zig_zag, TURN_DIR

from nav_msgs.msg import Odometry
from sdp_interfaces.action import Follow
from sdp_interfaces.srv import MotorEnable, NavConfirmation

from .api_client import APIClient
from .transformations import euler_from_quaternion

from .command_statuses import CommandStatus
from .task_types import TaskTypes
from .robot_states import RobotStates

class RobotStateController(Node):

    def __init__(self):
        super().__init__('robot_state_controller')

        self.init_complete = False

        self.robot_state = RobotStates.Init
        
        self.serial_number = "serial1"
        self.battery = 100 # int [0-100] 
        self.blocked = False

        self.current_command = TaskTypes.Idle
        self.current_command_id = None
        self.current_command_status = CommandStatus.Pending

        self.api_client = APIClient(self.serial_number)

        self.robot_pose = None

        # Client for the Navigation action serverc
        self._follow_action_client = ActionClient(self, Follow, 'follow')

        # Follow Testing 
        # self.create_timer(10, self.send_follow_goal)
        # self.send_follow_goal()


        # Client for Motor Switch service
        self._motor_switch_client = self.create_client(MotorEnable, 'motor_switch')

        self._nav_confirmation_service = self.create_service(NavConfirmation, 'nav_confirm', self.end_of_nav_callback) 

        # Robot pose subscriber
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry', self.odometry_callback, 10)

        self.api_timer = self.create_timer(1.0, self.api_call)


    def end_of_nav_callback(self, request, response):
        if request.success:
            self.get_logger().info('Finished navigation!')
            if self.robot_state == RobotStates.Task:
                self._next_state(self.current_command_id, self.current_command, CommandStatus.Completed)
        else:
            self.get_logger().info("Shouldn't ever be the case!")
        
        return response


    async def api_call(self):
        if self.robot_state != RobotStates.Init:
            # inject paused status whenever robot is in the middle of the task and there is obstacle in front
            command_status = CommandStatus.Paused if self.blocked and self.robot_state == RobotStates.Task else self.current_command_status
            poll_res = json.loads(await self.api_client.poll(self.battery, self.current_command_id, command_status))

            print(poll_res)

            self._next_state(poll_res['command_id'], self.api_client.instruction_to_robot_task(poll_res)) # TODO: Implement TASK exctraction
        else:
            init_res = json.loads(await self.api_client.init(self.battery))
            print(init_res)
            self._next_state(init_res['command_id'],  self.api_client.instruction_to_robot_task(init_res))

    def _next_state(self, id: int, instruction: TaskTypes, status: CommandStatus = None,):
        
        # DEBUGGING: print state before changes
        # self.get_logger().info('Before state update:')
        # self._print_robot_state()

        if self.robot_state == RobotStates.Init:
            self.current_command = instruction
            self.current_command_id = id
            self.current_command_status = CommandStatus.InProgress
            
            if instruction.value == RobotStates.Idle.value:
                self.robot_state = RobotStates.Idle
            else:
                self.robot_state = RobotStates.Task

        if self.robot_state == RobotStates.Idle:

            self.get_logger().info('Inside Idle')
                       
            if instruction in [TaskTypes.AbortLowBattery, TaskTypes.AbortSafety]:
                # Abort instructions are executed immediately

                self.get_logger().info('INSIDE FRICKING bATTERRY:')

                self.current_command = instruction
                self.current_command_id = id
                self.current_command_status = CommandStatus.InProgress
                self.switch_off_motors()
            
            elif instruction in [TaskTypes.TaskCircular, TaskTypes.TaskZigZag]:
                self.robot_state = RobotStates.Task

                self.current_command = instruction
                self.current_command_id = id
                self.current_command_status = CommandStatus.InProgress

                if instruction == TaskTypes.TaskCircular:
                    new_path = self.generate_pattern(gen_circular, curr_x_side=math.copysign(1, self.robot_pose[0]), angle=self.robot_pose[2])
                elif instruction == TaskTypes.TaskZigZag:
                    new_path = self.generate_pattern(gen_zig_zag, curr_x_side=math.copysign(1, self.robot_pose[0]), angle=self.robot_pose[2])

                # TODO: Add moving from starting point and to finish point
                self.send_follow_goal(new_path)

            elif instruction == TaskTypes.Idle:
                self.current_command = instruction
                self.current_command_id = id
                self.current_command_status = CommandStatus.InProgress


        
        if self.robot_state == RobotStates.Task:
            
            # TODO: Do nothing for LowBattery?

            if id == self.current_command_id:
                
                if status is not None:
                    # TODO: How to switch to idle?
                    # Send completed as long as it assigns a new task
                    self.current_command_status = status
                
            elif instruction == TaskTypes.AbortSafety:
                self.robot_state = RobotStates.Idle

                # Abort instructions are executed immediately
                self.current_command = instruction
                self.current_command_id = id
                self.current_command_status = CommandStatus.InProgress
                self.switch_off_motors()

            elif self.current_command_status == CommandStatus.Completed:
                self.robot_state = RobotStates.Idle
                self._next_state(id, instruction, status)

        # DEBUGGING: print state before changes
        self.get_logger().info('After state update:')
        self._print_robot_state()


    def _print_robot_state(self):
        self.get_logger().info(f'State:\nRobot State: {self.robot_state.value}\nCurrent Task: ({self.current_command_id}) {self.current_command.value}\nTask Status {self.current_command_status.value}\nObstacle Detected?: {self.blocked}')
        pass


    def generate_circular_path(self, end_x, end_y):
        path = gen_circular(Point(-7.5, -3.), TURN_DIR.RIGHT, 12., 2, 0)
        return sum([ [p.x, p.y] for p in path ], [])

    def generate_zigzag_path(self, end_x, end_y):
        path = gen_zig_zag(Point(-7.5, -3.), TURN_DIR.RIGHT, 12., 2, 0) 
        path = sum([ [p.x, p.y] for p in path ], [])
        # for p in path:
        #     self.get_logger().info(f'{p}')
        return path
    
    def generate_pattern(self, path_generator, curr_x_side, angle):
        def reverse_path(path):
            rev_path = []
            for i in reversed(range(0, len(path), 2)):
                rev_path.append(path[i])
                rev_path.append(path[i+1])
            return rev_path
            
        DIST_FROM_CENTERLINE = 7.0

        start_y_side = 1 if angle >= 0 else -1
        path = path_generator(Point(-DIST_FROM_CENTERLINE, 2.0), TURN_DIR.LEFT, 7.0, 2.0, 0)
        
        # convert from list of points to list of form [x1, y1, x2, y2, ...]
        path = sum([ [p.x, p.y] for p in path ], [])

        # reverse the side
        if path[-2] * curr_x_side < 0:
            path = [ coord if i % 2 else -coord for i, coord in enumerate(path) ]
        path_2 = [ -coord if i % 2 else coord for i, coord in enumerate(path) ] 

        pattern = reverse_path(path) + path_2 if start_y_side > 0 else reverse_path(path_2) + path

        # Robot needs tp come back to the center of the court
        pattern += [math.copysign(DIST_FROM_CENTERLINE, pattern[-2]), math.copysign(2.0, pattern[-1])]

        self.get_logger().info(f'Generated new path: {pattern}')

        return pattern

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
            self.shut_down_controller()

        else: 
            self.get_logger().info('Not managed to shut the motors! Repeating...')
            self.switch_off_motors()

    def shut_down_controller(self):
        # Stop polling the server
        self.api_timer.cancel()

        # Cancel any active nav tasks
        EMPTY_PATH = []
        self.send_follow_goal(EMPTY_PATH)
        
        self.get_logger().info('About to poll completed abort')

        # Update Abort instruction status to completed and notify the server 
        self.current_command_status = CommandStatus.Completed 
        self.api_client.poll(self.battery, self.current_command_id, self.current_command_status)

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
            

    def send_follow_goal(self, path):
        goal_msg = Follow.Goal()

        # GENERATE THE PATH HERE
        goal_msg.path = path

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

        self.current_command_status = CommandStatus.InProgress

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
