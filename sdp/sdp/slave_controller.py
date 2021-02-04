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

TIMESTEP = 16

DEVICE_CONFIG = {
    'camera': {
        'topic_name': 'camera',
        'timestep': TIMESTEP
    }
}

class SlaveController(WebotsNode):
    def __init__(self, args):
        super().__init__('slave_controller_node', args)

        self.timestep = TIMESTEP

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

        # Enable camera
        # self.start_device_manager(DEVICE_CONFIG)
        
        self.get_logger().info('All devices enabled and configured')

        # Create Subscriber
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

    def cmd_vel_callback(self, msg):
        wheel_gap = 0.1
        wheel_radius = 0.04

        left_speed = ((2.0 * msg.linear.x - msg.angular.z * wheel_gap) / (2.0 * wheel_radius))
        right_speed = ((2.0 * msg.linear.x + msg.angular.z * wheel_gap) / (2.0 * wheel_radius))
        left_speed = min(self.motor_max_speed, max(-self.motor_max_speed, left_speed))
        right_speed = min(self.motor_max_speed, max(-self.motor_max_speed, right_speed))

        self.motor_front_left.setVelocity(left_speed)
        self.motor_front_right.setVelocity(right_speed)
        self.motor_rear_left.setVelocity(left_speed)
        self.motor_rear_right.setVelocity(right_speed)


# def main(args=None):
#     rclpy.init(args=args)
#     controller = SlaveController(args=args)
#     rclpy.spin(controller)

#     controller.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

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


class ServiceNodeVelocity(WebotsNode):
    def __init__(self, args):
        super().__init__('slave_node', args)

        # Enable 3 sensors
        self.service_node_vel_timestep = 16

        self.get_logger().info('Sensor enabled')

        # Front wheels
        self.left_motor_front = self.robot.getMotor('wheel_motor_front_left')
        self.left_motor_front.setPosition(float('inf'))
        self.left_motor_front.setVelocity(0)

        self.right_motor_front = self.robot.getMotor('wheel_motor_front_right')
        self.right_motor_front.setPosition(float('inf'))
        self.right_motor_front.setVelocity(0)

        # Rear wheels
        self.left_motor_rear = self.robot.getMotor('wheel_motor_rear_left')
        self.left_motor_rear.setPosition(float('inf'))
        self.left_motor_rear.setVelocity(0)

        self.right_motor_rear = self.robot.getMotor('wheel_motor_rear_right')
        self.right_motor_rear.setPosition(float('inf'))
        self.right_motor_rear.setVelocity(0)

        self.motor_max_speed = self.left_motor_rear.getMaxVelocity()

        # Create Subscriber
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmdVel_callback, 1)

    def cmdVel_callback(self, msg):
        wheel_gap = 0.1  # in meter
        wheel_radius = 0.04  # in meter

        left_speed = ((2.0 * msg.linear.x - msg.angular.z *
                       wheel_gap) / (2.0 * wheel_radius))
        right_speed = ((2.0 * msg.linear.x + msg.angular.z *
                        wheel_gap) / (2.0 * wheel_radius))
        left_speed = min(self.motor_max_speed,
                         max(-self.motor_max_speed, left_speed))
        right_speed = min(self.motor_max_speed,
                          max(-self.motor_max_speed, right_speed))

        self.left_motor_front.setVelocity(left_speed)
        self.right_motor_front.setVelocity(right_speed)
        self.left_motor_rear.setVelocity(left_speed)
        self.right_motor_rear.setVelocity(right_speed)


def main(args=None):
    rclpy.init(args=args)
    client_vel = ServiceNodeVelocity(args=args)
    rclpy.spin(client_vel)

    client_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
