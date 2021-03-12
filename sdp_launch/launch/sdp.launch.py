import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # Useful directories
    sdp_dir = get_package_share_directory('sdp')
    core_dir = get_package_share_directory('webots_ros2_core')
    
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_dir, 'launch', 'robot_launch.py')
        ),
        launch_arguments=[
            ('package', 'sdp'),
            ('executable', 'start_robot'),
            ('world', PathJoinSubstitution(
                [sdp_dir, 'worlds', 'tennis_court_aruco.wbt'])),
        ]
    )

    ## Throws some strange error when launched together with the main controller
    #  TODO: Resolve it with Webots expert
    # aruco_navigator = Node(
    #     package="sdp",
    #     executable="aruco_navigator"
    # )

    # TRANSFORMS FROM THE WORLD COORDINATES TO ARUCO MARKERS

    aruco_0_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_0_transform",
        arguments=["-0.7", "-0.05", "1", "0", "0", "1.5708", "world", "aruco_0"]
    )

    aruco_1_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_1_transform",
        arguments=["0.7", "-0.05", "1", "0", "0", "1.5708", "world", "aruco_1"]
    )

    # ROS publishes that by default (btw. it publishes it in a wrong way)
    # TODO: Resolve it with Webots expert
    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_transform",
        arguments=["0.25", "0", "0.11", "-1.5708", "0", "0", "base_link", "camera_0"]
    )
    
    return LaunchDescription([
        webots,
        aruco_0_static_tf, 
        aruco_1_static_tf,
        # aruco_navigator, 
        camera_static_tf,
    ])