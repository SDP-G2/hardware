# Copyright 1996-2021 Soft_illusion.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from webots_ros2_core.webots_node import WebotsNode
from geometry_msgs.msg import Twist

from time import sleep

TIMESTEP = 16

class SlaveController(WebotsNode):
    def __init__(self, args):
        super().__init__('slave_controller_node', args)

        self.timestep = TIMESTEP

        sleep(15)

        # Enable Camera
        self.start_device_manager({
            'camera_0': {
                'topic_name': 'camera_0_feed',
                'timestep': self.timestep
            }
        })

        self.sensor = self.robot.getCamera('camera_0')
        self.sensor.enable(self.timestep)

        # Enable wheel motors
        self.motor_front_left = self.robot.getMotor('wheel_motor_front_left')
        self.motor_front_left.setPosition(float('inf'))
        self.motor_front_left.setVelocity(0)

        self.motor_front_right = self.robot.getMotor('wheel_motor_front_right')
        self.motor_front_right.setPosition(float('inf'))
        self.motor_front_right.setVelocity(0)

        self.motor_rear_left = self.robot.getMotor('wheel_motor_rear_left')
        self.motor_rear_left.setPosition(float('inf'))
        self.motor_rear_left.setVelocity(0)

        self.motor_rear_right = self.robot.getMotor('wheel_motor_rear_right')
        self.motor_rear_right.setPosition(float('inf'))
        self.motor_rear_right.setVelocity(0)

        self.motor_max_speed = self.motor_rear_left.getMaxVelocity()

        
        self.get_logger().info('All devices enabled and configured')

        # Create Subscriber
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

        

    def cmd_vel_callback(self, msg):
        wheel_gap = 0.1
        wheel_radius = 0.04

        self.get_logger().info('GOT CMD VEL MESSAGE!')

        left_speed = ((2.0 * msg.linear.x - msg.angular.z * wheel_gap) / (2.0 * wheel_radius))
        right_speed = ((2.0 * msg.linear.x + msg.angular.z * wheel_gap) / (2.0 * wheel_radius))
        left_speed = min(self.motor_max_speed, max(-self.motor_max_speed, left_speed))
        right_speed = min(self.motor_max_speed, max(-self.motor_max_speed, right_speed))

        self.motor_front_left.setVelocity(left_speed)
        self.motor_front_right.setVelocity(right_speed)
        self.motor_rear_left.setVelocity(left_speed)
        self.motor_rear_right.setVelocity(right_speed)


def main(args=None):
    rclpy.init(args=args)
    controller = SlaveController(args=args)
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()