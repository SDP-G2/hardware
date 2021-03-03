import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_interfaces.action import Follow


class RobotStateController(Node):

    def __init__(self):
        super().__init__('robot_state_controller')
        self._follow_action_client = ActionClient(self, Follow, 'follow')

        self.create_timer(30, self.send_follow_goal)
        self.send_follow_goal()

    def send_follow_goal(self):
        goal_msg = Follow.Goal()

        # GENERATE THE PATH HERE
        goal_msg.path = [0., 0., 1., 1., 2., 2., 3., 3., 0., 0.]

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