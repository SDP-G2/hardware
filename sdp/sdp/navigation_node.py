import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_interfaces.action import Follow

import time


class NavigationServer(Node):

    def __init__(self):
        super().__init__('navigation_server')
        self._action_server = ActionServer(
            self,
            Follow,
            'follow',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        goal_handle.succeed()
        
        # format [x1, y1, x2, y2, ..., xn, yn]
        path_to_follow = goal_handle.request.path

        while path_to_follow:
            feedback_msg = Follow.Feedback()
            feedback_msg.next_point = path_to_follow[:2]

            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info('Going to point <{}, {}>'.format(feedback_msg.next_point[0], feedback_msg.next_point[1]))

            #
            # INSERT THE CODE PROCESSING THE PATH HERE
            #

            time.sleep(5) # SIMULATES THE PROCESSING

            path_to_follow = path_to_follow[2:] # UPDATE THE REMAINING PATH WHEN POINT IS REACHED


        result = Follow.Result()
        result.status = 0

        return result


def main(args=None):
    rclpy.init(args=args)

    navigation_server = NavigationServer()

    rclpy.spin(navigation_server)


if __name__ == '__main__':
    main()