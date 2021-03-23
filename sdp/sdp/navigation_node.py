import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sdp_interfaces.srv import NavConfirmation
from sdp_interfaces.action import Follow

from .transformations import euler_from_quaternion

import time, math

RAD_PER_DEG = math.pi / 180.0

def angle(src_x, src_y, dst_x, dst_y):
    return math.atan2(dst_y - src_y, dst_x - src_x)

def almost(alpha: float, beta: float, tolerance: float):
    return math.isclose(alpha, beta, abs_tol=tolerance)

def get_next(a_list):
    return a_list.pop(0) if a_list else None

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2+(y2-y1)**2)

def in_range(x1, y1, x2, y2, threshold: float = 0.2):
    dist = euclidean_distance(x1, y1, x2, y2)
    return dist <= threshold


DEFAULT_LINEAR_VEL = .2
DEFUALT_ANGULAR_VEL = 2.1

class NavigationServer(Node):

    def __init__(self):
        super().__init__('navigation_server')
        self._action_server = ActionServer(
            self,
            Follow,
            'follow',
            self.execute_callback)

        self.robot_pose = None
        self.path_to_follow = None

        # Robot pose subscriber
        self.odom_subscriber = self.create_subscription(Odometry, '/GOD/odometry', self.odometry_callback, 10)

        # velocity command publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self._nav_confirm_client = self.create_client(NavConfirmation, 'nav_confirm')


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

        # self.get_logger().info(f'{self.robot_pose}')

        self.navigate_callback()


    def goal_callback(self, goal_request):
        self.get_logger().info(f'{goal_request}')
        return GoalResponse.ACCEPT


    def navigate_callback(self):
        if self.path_to_follow is not None:
            if self.path_to_follow:
                target = self.path_to_follow[:2]
                
                self.get_logger().info('Going to point <{}, {}>'.format(target[0], target[1]))

                if not in_range(self.robot_pose[0], self.robot_pose[1], target[0], target[1]):
                    # Initialize cmd_vel message
                    cmd_vel = Twist()
                    
                    # calculate direction to the target point
                    theta = angle(self.robot_pose[0], self.robot_pose[1], target[0], target[1])
                    
                    # accomodate for wraparound in range [-pi, pi]
                    if theta > math.pi/2 and self.robot_pose[2] < -math.pi/2:
                        theta -= 2 * math.pi
                    elif theta < -math.pi/2 and self.robot_pose[2] > math.pi/2:
                        theta += 2 * math.pi

                    # self.get_logger().info(f"Theta: {theta}")
                    


                    # Course correction code:
                    distance = euclidean_distance(self.robot_pose[0], self.robot_pose[1], target[0], target[1])
                    # angle_tolerance = 20 * RAD_PER_DEG
                    angle_tolerance = max(math.pi/2 - math.atan(distance/.05), .05)

                    self.get_logger().info(f'ROBOT: {self.robot_pose[2]}, TARGET: {theta}')
                    
                    if almost(self.robot_pose[2], theta, tolerance=angle_tolerance):
                        cmd_vel.linear.x = DEFAULT_LINEAR_VEL
                    elif self.robot_pose[2] < theta:
                        cmd_vel.linear.x = DEFAULT_LINEAR_VEL
                        cmd_vel.angular.z = DEFUALT_ANGULAR_VEL
                    else: # self.robot_pose[2] > theta
                        cmd_vel.linear.x = DEFAULT_LINEAR_VEL
                        cmd_vel.angular.z = -DEFUALT_ANGULAR_VEL

                    # publish the message
                    self.cmd_vel_publisher.publish(cmd_vel)
                else:
                    print(f"In range of <{target[0]}, {target[1]}>")
                    
                    self.path_to_follow = self.path_to_follow[2:] # UPDATE THE REMAINING PATH WHEN POINT IS REACHED

            else:
                # Send stop command
                # By default all the messages are init with 0s
                self.path_to_follow = None
                self.cmd_vel_publisher.publish(Twist())
                self.notify_controller_about_completion(True)
        

    def notify_controller_about_completion(self, succeeded):
        # wait for the service to become available
        while not self._nav_confirm_client.wait_for_service(timeout_sec=1.0):
            continue
        
        request = NavConfirmation.Request()
        request.success = succeeded

        future = self._nav_confirm_client.call_async(request)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        self.goal_handle = goal_handle
        goal_handle.succeed()
        
        # format [x1, y1, x2, y2, ..., xn, yn]
        self.path_to_follow = goal_handle.request.path

        result = Follow.Result()
        result.status = 0

        return result

        



def main(args=None):
    rclpy.init(args=args)

    navigation_server = NavigationServer()

    rclpy.spin(navigation_server)


if __name__ == '__main__':
    main()