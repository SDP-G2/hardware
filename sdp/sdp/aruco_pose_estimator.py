import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from time import sleep
import tf2_ros
from .transformations import compose_htm, quaternion_matrix, quaternion_from_euler
import functools
import math

# Size of the marker in the environment 
MARKER_SIZE = 0.4

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor_node')

        self.declare_parameter('camera_name', 'camera_0')
        self.declare_parameter('max_range', '9')
        self.declare_parameter('camera_x_res', '1280')

        ## Node's subscribers / publishers
        self.camera_subscriber = self.create_subscription(Image, '/camera_0/image_raw', self.image_processing_callback, 1)
        self.processed_image_publisher = self.create_publisher(Image, '/camera_0/processed', 10)
        
        # publishes pose estiamted from detected aruco markers
        self.pose_publisher = self.create_publisher(Vector3, '/odom_aruco', 10) 

        # ROS transform listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)


        ## OpenCV properties
        # Tool for transforming from ROS Image msg to OpenCV's frame
        self.bridge = CvBridge()
        
        # Aruco things
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        self.aruco_detector_parameters = aruco.DetectorParameters_create()

        self.get_logger().info('INIT COMPLETE')

        self.camera_x_res = int(self.get_parameter('camera_x_res').get_parameter_value().string_value)
        self.max_range = float(self.get_parameter('max_range').get_parameter_value().string_value)
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.get_logger().info(f'{self.camera_name}')


    def euclidean_distance(self, x, y, z):
        return math.sqrt(x**2+y**2+z**2)

    def euclidean_distance(self, x, y, z):
        return math.sqrt(x**2+y**2+z**2)

    def image_processing_callback(self, msg):
        # Get timestamp for the operation
        time = self.get_clock().now()

        # Open CV frame from Camera sensor topic
        frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        height = msg.height
        width = msg.width        

        # focal length calculation for ideal intrisic camera matrix
        # f = res_x/(2*tan(fov/2))
        # for fov = 0.785 # Webots' default where no LENS param is set
        # source: comment in https://stackoverflow.com/questions/61555182/webot-camera-default-parameters-like-pixel-size-and-focus
        f = 1533.88925565 if self.camera_x_res == 1280 else 2318.956708175
        # f = 2318.956708175 # 1080p

        # ideal camera and distortion matrices
        camera_matrix = np.mat([[f, 0.0, width/2.0], [0.0, f, height/2.0], [0.0, 0.0, 1.0]])
        distorion_matrix = np.mat([0.0, 0.0, 0.0, 0.0])

        # convert frame into grayscale for better marker detection
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Changes to grayscale

        corners, ids, _ = aruco.detectMarkers(gray_frame, self.aruco_dict, parameters=self.aruco_detector_parameters, cameraMatrix=camera_matrix, distCoeff=distorion_matrix)

        # check if the ids list is not empty
        # if no check is added the code will crash
        if np.all(ids is not None):

            ## draw a square around the markers
            aruco.drawDetectedMarkers(frame, corners)

            camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
            
            # Get the transform from the camera_0 to robot's base_link
            try:
                T_c_b = self.get_htm(camera_name, 'base_link', time)
            except:
                self.get_logger().warn(f'Cannot get {camera_name}->base_link transform at time={time}')
                return

            # list contating the pose estimates from all detected markers
            pose_estimates = []

            # self.get_logger().warn(f'{ids.size}')

            for i in range(0, ids.size):
                # ID of the aruco that is currently being processed
                aruco_id = ids[i][0]

                # estimate pose of each marker and return the values
                # rvec and tvec transformations from camera_0->aruco
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE, camera_matrix, distorion_matrix)

                dist_to_marker = self.euclidean_distance(tvec[0][0][0], tvec[0][0][1], tvec[0][0][2])

                if dist_to_marker > self.max_range:
                    # self.get_logger().info(f'Dist to marker = {dist_to_marker}')
                    continue

                ## draw axis on the aruco markers
                aruco.drawAxis(frame, camera_matrix, distorion_matrix, rvec, tvec, MARKER_SIZE / 2.0)

                # Get the transform from the world to aruco_<id> 
                try:
                    T_w_a = self.get_htm('world', f'aruco_{aruco_id}', time)
                except:
                    self.get_logger().warn(f'Cannot get world->aruco_{aruco_id} transform at time={time}')
                    continue

                # Estimate the robot's pose using the pose of the detected marker 
                estimated_pose = self.estimate_robot_pose(rvec=rvec[0][0], tvec=tvec[0][0], T_w_a=T_w_a, T_c_b=T_c_b)
                
                pose_estimates.append(estimated_pose)
                self.get_logger().info(f'Pose estimated from marker {ids[i]}: {estimated_pose}') # DEBUG
        else:
            return

        # Quit if there are no estimates
        if not pose_estimates:
            return
        
        # Pose as a mean of the estimated poses
        mean_pose = np.mean(np.array(pose_estimates), axis=0)

        # rotate yaw (theta) by pi/2 as ROS2 publishes wrong transform
        # mean_pose[2] -= math.pi / 2.0

        # fit into [-PI, PI] range
        if mean_pose[2] > math.pi:
            mean_pose[2] -= 2 * math.pi
        elif mean_pose[2] < -math.pi:
            mean_pose[2] += 2 * math.pi

        # Publish the estimate
        self.send_estimated_odometry(mean_pose, time)
        self.processed_image_publisher.publish(self.bridge.cv2_to_imgmsg(frame, 'rgb8'))


    def send_estimated_odometry(self, estimated_pose, time):
        # print(f'Sending mean pose: {estimated_pose}') # DEBUG
        
        ## Odometry message version

        # current_time = time.to_msg()

        # odom = Odometry()

        # # set headers
        # odom.header.frame_id = 'world'
        # odom._child_frame_id = 'base_link'
        # odom.header.stamp = current_time
        
        # # set the position
        # odom.pose.pose.position.x = estimated_pose[0]
        # odom.pose.pose.position.y = estimated_pose[1]
        # q = quaternion_from_euler(0, 0, estimated_pose[2])
        # odom.pose.pose.orientation.x = q[0]
        # odom.pose.pose.orientation.y = q[1]
        # odom.pose.pose.orientation.z = q[2]
        # odom.pose.pose.orientation.w = q[3]

        # Vector3 message version
        # x, y - position
        # z - yaw

        vec = Vector3()
        vec.x = estimated_pose[0]
        vec.y = estimated_pose[1]
        vec.z = estimated_pose[2]

        self.pose_publisher.publish(vec)


    def estimate_robot_pose(self, rvec, tvec, T_w_a, T_c_b):
        # Invert the transformation returned by OpenCV,
        # as rvec and tvec describe transform camera->aruco
        R_c_a, _ = cv2.Rodrigues(rvec)
        R_a_c = R_c_a.T
        t_a_c = -np.matmul(R_a_c, tvec) 
        T_a_c = compose_htm(rotation=R_a_c, translation=t_a_c)

        # Combine the transformations to obtain world->base_link transform
        # note that resulting HTM descibe the position and the orientation 
        # of the base_link in the world coordinate frame
        T_w_b = functools.reduce(lambda T,Q: np.matmul(T, Q), [T_w_a, T_a_c, T_c_b])

        # Extracting the robot's pose information from base_link's HTM
        x = T_w_b[0,3]
        y = T_w_b[1,3]

        # ROS performs Yaw(Z), Pitch(Y), Roll(X) Euler-Rotation
        # getting the yaw from the R part of HTM as described here:
        # https://stackoverflow.com/questions/11514063/extract-yaw-pitch-and-roll-from-a-rotationmatrix
        theta = math.atan2(T_w_b[1,0], T_w_b[0,0])

        # Return estimated pose
        return [x, y, theta]


    def get_htm(self, target_frame, source_frame, time):
        tf_stamped = self.tfBuffer.lookup_transform(target_frame, source_frame, time)
        
        # Get rotation
        tf_q = tf_stamped.transform.rotation
        R = quaternion_matrix([tf_q.x, tf_q.y, tf_q.z, tf_q.w])
        
        # Get translation
        tf_t = tf_stamped.transform.translation
        t = np.array([tf_t.x, tf_t.y, tf_t.z])

        # Compose the HTM from the retrieved rotation and translation
        return compose_htm(rotation=R, translation=t)


def main(args=None):
    rclpy.init(args=args)

    image_processor = ImageProcessor()
    rclpy.spin(image_processor)

    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()