import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import cv2
import cv2.aruco as aruco
# from cv_bridge import CvBridge
from time import sleep

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor_node')

        # NEEDED FOR THE Webots to work
        # self.get_logger().info('Before sleep')
        # sleep(20)
        # self.get_logger().info('After sleep')

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.speed = .1
        self.angle_correction = -10

        self.delta = 0
        self.cmd = Twist()
        self.stop = False
        self.count = 0
        self.number_of_pixels = 512
        self.reach_threshold = self.number_of_pixels * .4 # 40% of the image
        self.reached = False

        self.camera_subscriber = self.create_subscription(Image, '/camera_0/image_raw', self.temp_image_processing_callback, 10)
        # self.bridge = CvBridge()
        # self.processed_image_publisher = self.create_publisher(Image, '/camera/processed', 1)
        self.translation_x = 0

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_detector_parameters = aruco.DetectorParameters_create()

        self.subscription = self.create_subscription(
            String,
            '/topic/talker',
            self.listener_callback,
            10)
        
        self.get_logger().info('INIT COMPLETE')
    
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def temp_image_processing_callback(self, msg):
        self.get_logger().info('GOT IMAGE')
        self.correct_position()


    # def image_processing_callback(self, msg):

    #     self.get_logger().info('Got IMAGE')

    #     frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
    #     height = msg.height
    #     width = msg.width
    #     self.number_of_pixels = height

    #     # camera and distortion matrices
    #     camera_matrix = np.mat([[1, 0, height/2], [0, 1, width/2], [0, 0, 1]])
    #     distorion_matrix = np.mat([0, 0, 0, 0])

    #     gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Changes to grayscale

    #     corners, ids, rejected_img_points = aruco.detectMarkers(gray_frame, self.aruco_dict, parameters=self.aruco_detector_parameters, cameraMatrix=camera_matrix, distCoeff=distorion_matrix)

    #     # check if the ids list is not empty
    #     # if no check is added the code will crash
    #     if np.all(ids is not None):

    #         # draw a square around the markers
    #         aruco.drawDetectedMarkers(frame, corners)

    #         for i in range(0, ids.size):
    #              # estimate pose of each marker and return the values
    #             # rvet and tvec-different from camera coefficients
    #             rvec, tvec, marker_points = aruco.estimatePoseSingleMarkers(corners[i], .1, camera_matrix, distorion_matrix)

    #             # draw axis for the aruco markers
    #             aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1)

    #             # Goal reaaching criteria
    #             if abs(corners[i][0][0][0] - corners[i][0][2][0]) > self.reach_threshold:
    #                 self.reached = True
    #                 self.get_logger().info(f'Reached the aruco tag #{ids[i]}')
    #             else:
    #                 self.reached = False

    #             self.translation_x = tvec[0][0][0]
    #             self.stop = False
    #     else:
    #         self.translation_x = 0
    #         self.stop = True

    #     self.processed_image_publisher(self.bridge.cv2_to_imgmsg(frame, 'rgb8'))
    #     self.correct_position()

    def correct_position(self):
        # self.cmd.linear.x = self.speed

        # self.cmd.angular.z = self.angle_correction * self.translation_x

        self.cmd.linear.x = .1
        self.cmd.angular.z = 1.0
        self.get_logger().info('Sending command')
        self.cmd_vel_publisher.publish(self.cmd)

        return

        # if self.stop:
        #     self.cmd.linear.x = .01
        #     self.cmd.angular.z = 1.2

        # if self.reached:
        #     self.cmd.linear.x = 0
        #     self.cmd.angular.z = 0

        # self.cmd_vel_publisher.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)

    image_processor = ImageProcessor()
    rclpy.spin(image_processor)

    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()