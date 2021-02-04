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
                [sdp_dir, 'worlds', 'tennis_court.wbt'])),
        ]
    )

    # talker_node = Node(
    #     package="sdp",
    #     executable="talker",
    #     output='screen',
    #     emulate_tty=True,
    # )
    # listener_node = Node(
    #     package="sdp",
    #     executable="listener",
    #     output='screen',
    #     emulate_tty=True,
    # )
    
    
    return LaunchDescription([
        webots
    ])