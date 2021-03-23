import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

from sdp_interfaces.action import Follow
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from .transformations import euler_from_quaternion

import time, math

RAD_PER_DEG = math.pi / 180.0

def angle(src_x, src_y, dst_x, dst_y):
    return math.atan2(dst_y - src_y, dst_x - src_x)

def almost(alpha: float, beta: float, tolerance: float = 10):
    return math.isclose(alpha, beta, abs_tol=(tolerance * RAD_PER_DEG))

def get_next(a_list):
    return a_list.pop(0) if a_list else None

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2+(y2-y1)**2)

def in_range(x1, y1, x2, y2, threshold: float = 0.2):
    dist = euclidean_distance(x1, y1, x2, y2)
    return dist <= threshold


DEFAULT_LINEAR_VEL = .125
DEFUALT_ANGULAR_VEL = .25

class NavigationServer(Node):

    def __init__(self):
        super().__init__('navigation_server')
        self._action_server = ActionServer(
            self,
            Follow,
            'follow',
            self.execute_callback)

        self.robot_pose = None
        self.path_to_follow = []

        self.goal_handle = None

        # Robot pose subscriber
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry', self.odometry_callback, 10)

        # velocity command publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # self._action_server.

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

        self.get_logger().info(f'{self.robot_pose}')


    def goal_callback(self, goal_request):
        self.get_logger().info(f'{goal_request}')
        return GoalResponse.ACCEPT


    def navigate_callback(self):
        pass 

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        self.goal_handle = goal_handle
        goal_handle.succeed()
        
        # format [x1, y1, x2, y2, ..., xn, yn]
        self.path_to_follow = goal_handle.request.path

        while self.path_to_follow:
            target = self.path_to_follow[:2]

            feedback_msg = Follow.Feedback()
            feedback_msg.next_point = target

            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info('Going to point <{}, {}>'.format(feedback_msg.next_point[0], feedback_msg.next_point[1]))

            while not in_range(self.robot_pose[0], self.robot_pose[1], target[0], target[1]):
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
                angle_tolerance = 20 * RAD_PER_DEG

                # self.get_logger().info(f'ROBOT: {self.robot_pose[2]}, TARGET: {theta}')
                
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

                    
            print(f"In range of <{target[0]}, {target[1]}>")
            
            path_to_follow = path_to_follow[2:] # UPDATE THE REMAINING PATH WHEN POINT IS REACHED
            

        # Send stop command
        # By default all the messages are init with 0s
        self.cmd_vel_publisher.publish(Twist())
    
        result = Follow.Result()
        result.status = 0

        return result


def main(args=None):
    rclpy.init(args=args)

    navigation_server = NavigationServer()

    rclpy.spin(navigation_server)


if __name__ == '__main__':
    main()