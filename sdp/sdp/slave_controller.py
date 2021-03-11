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
from geometry_msgs.msg import Twist, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from sdp_interfaces.srv import MotorEnable
from sensor_msgs.msg import Image

from .transformations import quaternion_from_euler
import tf2_ros

from time import sleep
import math

class SlaveController(WebotsNode):
    def __init__(self, args):
        super().__init__('slave_controller_node', args)

        # Disable interface for GOD sensors as they are used only internally
        self.start_device_manager({
            'god_imu': {
                'disable': True
            },
            'god_gps': {
                'disable': True
            },
            'god_gyro': {
                'disable': True
            }
        })

        # Enable GOD sensors (GPS, IMU, GYRO) for exact pose and velocity
        self.god_gps = self.robot.getDevice('god_gps')
        self.god_gps.enable(self.timestep)

        self.god_imu = self.robot.getDevice('god_imu')
        self.god_imu.enable(self.timestep) 

        self.god_gyro = self.robot.getDevice('god_gyro')
        self.god_gyro.enable(self.timestep)

        # Enable wheel motors
        self.motor_front_left = self.robot.getDevice('wheel_motor_front_left')
        self.motor_front_left.setPosition(float('inf'))
        self.motor_front_left.setVelocity(0)

        self.motor_front_right = self.robot.getDevice('wheel_motor_front_right')
        self.motor_front_right.setPosition(float('inf'))
        self.motor_front_right.setVelocity(0)

        self.motor_rear_left = self.robot.getDevice('wheel_motor_rear_left')
        self.motor_rear_left.setPosition(float('inf'))
        self.motor_rear_left.setVelocity(0)

        self.motor_rear_right = self.robot.getDevice('wheel_motor_rear_right')
        self.motor_rear_right.setPosition(float('inf'))
        self.motor_rear_right.setVelocity(0)

        self.motor_max_speed = self.motor_front_left.getMaxVelocity()

        # Enable motor encoders
        motor_encoder_names = ['ps_front_left', 'ps_front_right']
        self.motor_encoders = [self.robot.getDevice(encoder_name) for encoder_name in motor_encoder_names]
        for motor_encoder in self.motor_encoders:
            motor_encoder.enable(self.timestep)

        ## Initialize variables storing the reading of sensors
        # Encoders
        self.last_update_time = self.robot.getTime() # simulation time

        self.last_encoder_readings = [0.0, 0.0]
        self.encoder_readings = self.last_encoder_readings[:]

        # GOD properties
        self.current_position = [0, 0, 0] # god_gps.getValues()
        self.current_orienation = [0, 0, 0] # god_imu.getRollPitchYaw()
        self.current_ang_vel = [0, 0, 0] # god_gyro.getValues()
        self.current_speed = 0 # god_gps.getSpeed()

        # Pose estimated updated by the robot
        self.estimated_pose = [0, 0, 0]


        self.are_motors_enabled = True

        ## Robot properties 
        # Those two parameters need to be determined/adjusted experimentally
        self.wheel_radius = .04

        # .12 for better odometry for arc turning; .14 for better in-place turning 
        self.wheel_separation = .14 # 

        # CHEAT: flag for initalising initial robot estimate with GOD values
        self.INIT_VARS = False

        ## Node's publishers / subscribers 

        # GOD odometry publisher
        self.god_odometry_publisher = self.create_publisher(Odometry, '/GOD/odometry', 10)
        self.create_timer(self.timestep * 1e-3, self.publish_god_odometry)

        # Estimated publisher
        self.odometry_publisher = self.create_publisher(Odometry, '/odometry', 10)
        self.create_timer(self.timestep * 1e-3, self.update_odometry)

        # Aruco pose estimate subscriber
        # TODO: Replace with time-STAMPED message (e.g. Vector3Stamped or Odometry)
        self.aruco_pose_subscriber = self.create_subscription(Vector3, '/odom_aruco', self.merge_aruco_odometry, 10)

        # Velocity controller subscriber
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Service blocking motors
        self.motor_switch_service = self.create_service(MotorEnable, 'motor_switch', self.switch_motors_onoff) 

        # ROS Transform broadcaster
        self.transform_broadcaster = tf2_ros.TransformBroadcaster(node=self)

        self.get_logger().info('All devices enabled and configured')

    
    def switch_motors_onoff(self, request, response):
        self.are_motors_enabled = request.enabled

        # Stop motors if we are blocking them
        if not self.are_motors_enabled:
            self.motor_front_left.setVelocity(0)
            self.motor_rear_left.setVelocity(0)
            self.motor_front_right.setVelocity(0)
            self.motor_rear_right.setVelocity(0)

        self.get_logger().info('Turning the motors {}!'.format('on' if self.are_motors_enabled else 'off'))

        response.enabled = self.are_motors_enabled
        return response



    def update_odometry(self):
        # exit if the variables are not initialised
        if not self.INIT_VARS:
            return
        
        # simulation time
        time = self.robot.getTime()
        
        # ROS time (not sure if it is synced with simulation time) - as stamped messages require ROS time
        # TODO: check if it is the same time
        time_msg = self.get_clock().now().to_msg()

        dt = time - self.last_update_time

        # Proceed if only some time has passed since last calculation
        if dt == 0:
            return


        # determine distance travelled by each wheel in time dt
        self.encoder_readings = [encoder.getValue() for encoder in self.motor_encoders]
        dist_travelled = [(self.encoder_readings[i] - self.last_encoder_readings[i]) * self.wheel_radius for i in range(len(self.encoder_readings))]
        
        velocities = [dist / dt for dist in dist_travelled]
        
        # Two cases based on the relative velocities of the wheels
        # If both of them have the same velocity (within some tolerance range)
        # then robot is moving straight
        # Otherwise, calculation of the turn are necessary
        # Helpful resource: https://www.usna.edu/Users/cs/crabbe/SI475/current/mob-kin/mobkin.pdf
        if abs(velocities[0] - velocities[1]) * dt < .00001:
            v = (velocities[0] + velocities[1]) / 2
            w = 0.0

            # simple update step of the linear case
            # theta (yaw) is angle between world X-axis and robot direction of movement (its X-axis) 
            self.estimated_pose[0] += v * math.cos(self.estimated_pose[2]) * dt
            self.estimated_pose[1] += v * math.sin(self.estimated_pose[2]) * dt
        else:
            ## V2 - source: https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf
            # distance travelled by the center of the robot
            d_center = (dist_travelled[0] + dist_travelled[1]) / 2
            
            # phi = change of theta (note the order of variables)
            phi = (dist_travelled[1] - dist_travelled[0]) / self.wheel_separation

            # update step for the case when robot is turning
            self.estimated_pose[0] += d_center * math.cos(self.estimated_pose[2] + phi/2)
            self.estimated_pose[1] += d_center * math.sin(self.estimated_pose[2] + phi/2)
            self.estimated_pose[2] += phi

            v = d_center / dt
            w = phi / dt

            ## V1 - source: https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf
            # v = (velocities[0] + velocities[1]) / 2
            # w = (velocities[1] - velocities[0]) / self.wheel_separation
            
            # # aliases for easier to read calculations (variable names as in the PDF)
            # v_l = velocities[0]
            # v_r = velocities[1]
            # l = self.wheel_separation
            
            # self.estimated_pose[2] += w * dt
            # # self.estimated_pose[2] = current_orienation[2] # orientaion (theta) is the easies to lose

            # if self.estimated_pose[2] < -math.pi:
            #     self.estimated_pose[2] += 2 * math.pi
            # elif self.estimated_pose[2] > math.pi:
            #     self.estimated_pose[2] -= 2 * math.pi
            
            # self.estimated_pose[0] += v * l / (v_r - v_l) * math.sin(w*dt)
            
            # self.estimated_pose[1] += (-1) * v * l / (v_r - v_l) * math.cos(w*dt) + v * l / (v_r - v_l)


        # update 'last' variables
        self.last_encoder_readings = self.encoder_readings[:]
        self.last_update_time = time

        # self.get_logger().info(f'dist: {math.dist(self.estimated_pose[:2], [self.current_position[2], self.current_position[0]])}') # DEBUG

        # Publish updated odometry
        self.publish_odometry(v, w, time_msg)


    def publish_odometry(self, linear_vel, angular_vel, msg_timestamp):
        # Robot node publishes the current transform to its main link in addition to its odometry
        # not sure if it is necessary

        t = TransformStamped()

        t.header.stamp = msg_timestamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.estimated_pose[0]
        t.transform.translation.y = self.estimated_pose[1]

        q = quaternion_from_euler(0, 0, self.estimated_pose[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.transform_broadcaster.sendTransform(t)

        # Publish odometry messages so other nodes can use it (e.g. for navigation)
        odom = Odometry()

        odom.header.frame_id = 'world'
        odom._child_frame_id = 'base_link'
        odom.header.stamp = msg_timestamp
        
        # set the position
        odom.pose.pose.position.x = self.estimated_pose[0]
        odom.pose.pose.position.y = self.estimated_pose[1]

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # set the velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel

        self.odometry_publisher.publish(odom)


    def publish_god_odometry(self):
        # Update GOD values
        self.current_speed = self.god_gps.getSpeed()
        self.current_position = self.god_gps.getValues()
        self.current_orienation = self.god_imu.getRollPitchYaw()
        self.current_ang_vel = self.god_gyro.getValues()

        # CHEAT - read the inital position of the robot in the world from the GOD sensors
        if not self.INIT_VARS:
            # Set the initial position in the world
            self.estimated_pose = [self.current_position[2], self.current_position[0], self.current_orienation[2]]

            self.last_encoder_readings = [encoder.getValue() for encoder in self.motor_encoders]
            self.encoder_readings = self.encoder_readings[:]
            
            self.INIT_VARS = True


        # Robot node publishes the current transform to its main link in addition to its odometry
        # not sure if it is necessary, even more for god values
        br = tf2_ros.TransformBroadcaster(node=self)
        t = TransformStamped()

        current_time = self.get_clock().now().to_msg()

        t.header.stamp = current_time
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link_god'
        t.transform.translation.x = self.current_position[2]
        t.transform.translation.y = self.current_position[0]
        t.transform.translation.z = self.current_position[1]

        q = quaternion_from_euler(0, 0, self.current_orienation[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.transform_broadcaster.sendTransform(t)

        # Publish odometry messages so other nodes can use it (e.g. for navigation)
        odom = Odometry()

        # set headers
        odom.header.frame_id = 'world'
        odom._child_frame_id = 'base_link_god'
        odom.header.stamp = current_time
        
        # set the position
        odom.pose.pose.position.x = self.current_position[2]
        odom.pose.pose.position.y = self.current_position[0]
        odom.pose.pose.position.z = self.current_position[1]
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # set the velocity
        odom.twist.twist.linear.x = self.current_speed
        odom.twist.twist.angular.z = self.current_ang_vel[1]

        self.god_odometry_publisher.publish(odom)

    
    def merge_aruco_odometry(self, msg):
        self.estimated_pose[0] = msg.x
        self.estimated_pose[1] = msg.y
        self.estimated_pose[2] = msg.z


    # Robot should be controlled using cmd_vel Twist messages with only two values set:
    # linear.x - v - linear velocity
    # angular.z - w - angular velocity around z-axis (yaw)
    def cmd_vel_callback(self, msg):
        # Update odometry to flush the encoder readings
        self.update_odometry()

        if not self.are_motors_enabled:
            return

        # Code from some tutorial to convert from v, w to velocities of individual wheels (might be wrong)
        left_speed = ((2.0 * msg.linear.x - msg.angular.z * self.wheel_separation) / (2.0 * self.wheel_radius))
        right_speed = ((2.0 * msg.linear.x + msg.angular.z * self.wheel_separation) / (2.0 * self.wheel_radius))
        
        # Cap the speeds at motor's maximum velocity 
        # Note that in such case it may no longer follow desired radius of the turn 
        left_speed = min(self.motor_max_speed, max(-self.motor_max_speed, left_speed))
        right_speed = min(self.motor_max_speed, max(-self.motor_max_speed, right_speed))

        print(left_speed)

        self.motor_front_left.setVelocity(left_speed)
        self.motor_rear_left.setVelocity(left_speed)
        self.motor_front_right.setVelocity(right_speed)
        self.motor_rear_right.setVelocity(right_speed)


def main(args=None):
    rclpy.init(args=args)

    controller = SlaveController(args=args)
    
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()