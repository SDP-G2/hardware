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
                [sdp_dir, 'worlds', 'tennis_court_env.wbt'])),
        ]
    )

    ## Throws some strange error when launched together with the main controller
    #  TODO: Resolve it with Webots expert
    # aruco_navigator = Node(
    #     package="sdp",
    #     executable="aruco_navigator"
    # )

    # TRANSFORMS FROM THE WORLD COORDINATES TO ARUCO MARKERS

    # aruco_0_static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="aruco_0_transform",
    #     arguments=["-0.7", "-0.05", "1", "0", "0", "1.5708", "world", "aruco_0"]
    # )

    # aruco_1_static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="aruco_1_transform",
    #     arguments=["0.7", "-0.05", "1", "0", "0", "1.5708", "world", "aruco_1"]
    # )

    aruco_100_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_100_transform",
        arguments=["8.50", "-18.25", "0.60", "3.1416", "0", "1.5708", "world", "aruco_100"]
    )

    aruco_101_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_101_transform",
        arguments=["6.50", "-18.25", "0.60", "3.1416", "0", "1.5708", "world", "aruco_101"]
    )

    aruco_102_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_102_transform",
        arguments=["4.50", "-18.25", "0.60", "3.1416", "0", "1.5708", "world", "aruco_102"]
    )

    aruco_103_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_103_transform",
        arguments=["2.50", "-18.25", "0.60", "3.1416", "0", "1.5708", "world", "aruco_103"]
    )

    aruco_104_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_104_transform",
        arguments=["0.50", "-18.25", "0.60", "3.1416", "0", "1.5708", "world", "aruco_104"]
    )

    aruco_105_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_105_transform",
        arguments=["-1.50", "-18.25", "0.60", "3.1416", "0", "1.5708", "world", "aruco_105"]
    )

    aruco_106_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_106_transform",
        arguments=["-3.50", "-18.25", "0.60", "3.1416", "0", "1.5708", "world", "aruco_106"]
    )

    aruco_107_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_107_transform",
        arguments=["-5.50", "-18.25", "0.60", "3.1416", "0", "1.5708", "world", "aruco_107"]
    )

    aruco_108_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_108_transform",
        arguments=["-7.50", "-18.25", "0.60", "3.1416", "0", "1.5708", "world", "aruco_108"]
    )


    aruco_109_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_109_transform",
        arguments=["-8.5", "-18.25", "0.60", "3.1416", "0", "1.5708", "world", "aruco_109"]
    )
    
    aruco_200_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_200_transform",
        arguments=["-9.10", "-15.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_200"]
    )

    aruco_201_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_201_transform",
        arguments=["-9.10", "-13.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_201"]
    )

    aruco_202_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_202_transform",
        arguments=["-9.10", "-11.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_202"]
    )

    aruco_203_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_203_transform",
        arguments=["-9.10", "-9.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_203"]
    )

    aruco_204_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_204_transform",
        arguments=["-9.10", "-7.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_204"]
    )

    aruco_205_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_205_transform",
        arguments=["-9.10", "-5.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_205"]
    )

    aruco_206_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_206_transform",
        arguments=["-9.10", "-3.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_206"]
    )

    aruco_207_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_207_transform",
        arguments=["-9.10", "-1.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_207"]
    )

    aruco_208_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_208_transform",
        arguments=["-9.10", "1.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_208"]
    )

    aruco_209_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_209_transform",
        arguments=["-9.10", "3.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_209"]
    )

    aruco_210_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_210_transform",
        arguments=["-9.10", "5.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_210"]
    )

    aruco_211_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_211_transform",
        arguments=["-9.10", "7.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_211"]
    )

    aruco_212_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_212_transform",
        arguments=["-9.10", "9.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_212"]
    )

    aruco_213_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_213_transform",
        arguments=["-9.10", "11.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_213"]
    )

    aruco_214_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_214_transform",
        arguments=["-9.10", "13.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_214"]
    )

    aruco_215_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_215_transform",
        arguments=["-9.10", "15.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_215"]
    )

    aruco_216_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_216_transform",
        arguments=["-9.10", "17.00", "0.60", "1.5708", "0", "1.5708", "world", "aruco_216"]
    )

    aruco_300_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_300_transform",
        arguments=["-8.50", "18.25", "0.60", "0", "0", "1.5708", "world", "aruco_300"]
    )

    aruco_301_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_301_transform",
        arguments=["-6.50", "18.25", "0.60", "0", "0", "1.5708", "world", "aruco_301"]
    )

    aruco_302_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_302_transform",
        arguments=["-4.50", "18.25", "0.60", "0", "0", "1.5708", "world", "aruco_302"]
    )

    aruco_303_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_303_transform",
        arguments=["-2.50", "18.25", "0.60", "0", "0", "1.5708", "world", "aruco_303"]
    )

    aruco_304_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_304_transform",
        arguments=["-0.50", "18.25", "0.60", "0", "0", "1.5708", "world", "aruco_304"]
    )

    aruco_305_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_305_transform",
        arguments=["1.50", "18.25", "0.60", "0", "0", "1.5708", "world", "aruco_305"]
    )

    aruco_306_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_306_transform",
        arguments=["3.50", "18.25", "0.60", "0", "0", "1.5708", "world", "aruco_306"]
    )

    aruco_307_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_307_transform",
        arguments=["5.50", "18.25", "0.60", "0", "0", "1.5708", "world", "aruco_307"]
    )

    aruco_308_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_308_transform",
        arguments=["7.50", "18.25", "0.60", "0", "0", "1.5708", "world", "aruco_308"]
    )

    aruco_309_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_309_transform",
        arguments=["8.50", "18.25", "0.60", "0", "0", "1.5708", "world", "aruco_309"]
    )

    aruco_400_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_400_transform",
        arguments=["9.10", "17.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_400"]
    )

    aruco_401_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_401_transform",
        arguments=["9.10", "15.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_401"]
    )

    aruco_402_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_402_transform",
        arguments=["9.10", "13.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_402"]
    )

    aruco_403_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_403_transform",
        arguments=["9.10", "11.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_403"]
    )

    aruco_404_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_404_transform",
        arguments=["9.10", "9.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_404"]
    )

    aruco_405_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_405_transform",
        arguments=["9.10", "7.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_405"]
    )

    aruco_406_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_406_transform",
        arguments=["9.10", "5.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_406"]
    )

    aruco_407_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_407_transform",
        arguments=["9.10", "3.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_407"]
    )

    aruco_408_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_408_transform",
        arguments=["9.10", "1.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_408"]
    )

    aruco_409_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_409_transform",
        arguments=["9.10", "-1.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_409"]
    )

    aruco_410_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_410_transform",
        arguments=["9.10", "-3.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_410"]
    )

    aruco_411_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_411_transform",
        arguments=["9.10", "-5.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_411"]
    )

    aruco_412_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_412_transform",
        arguments=["9.10", "-7.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_412"]
    )

    aruco_413_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_413_transform",
        arguments=["9.10", "-9.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_413"]
    )

    aruco_414_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_414_transform",
        arguments=["9.10", "-11.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_414"]
    )

    aruco_415_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_415_transform",
        arguments=["9.10", "-13.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_415"]
    )

    aruco_416_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_416_transform",
        arguments=["9.10", "-15.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_416"]
    )

    aruco_417_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_417_transform",
        arguments=["9.10", "-17.00", "0.60", "-1.5708", "0", "1.5708", "world", "aruco_417"]
    )



    # ROS publishes that by default (btw. it publishes it in a wrong way)
    # TODO: Resolve it with Webots expert
    camera_0_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_transform",
        arguments=["0.25", "0", "0.11", "-1.5708", "0", "0", "base_link", "camera_0"]
    )

    camera_1_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_transform",
        arguments=["0", "0", "0.11", "-1.5708", "0", "0", "base_link", "camera_1"]
    )

    camera_2_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_transform",
        arguments=["0.25", "0", "0.11", "-1.5708", "0", "0", "base_link", "camera_2"]
    )


    
    return LaunchDescription([
        webots,
        # aruco_0_static_tf, 
        # aruco_1_static_tf,
        # aruco_navigator,
        aruco_100_static_tf,
        aruco_101_static_tf,
        aruco_102_static_tf,
        aruco_103_static_tf,
        aruco_104_static_tf,
        aruco_105_static_tf,
        aruco_106_static_tf,
        aruco_107_static_tf,
        aruco_108_static_tf,
        aruco_109_static_tf,
        aruco_200_static_tf,
        aruco_201_static_tf,
        aruco_202_static_tf,
        aruco_203_static_tf,
        aruco_204_static_tf,
        aruco_205_static_tf,
        aruco_206_static_tf,
        aruco_207_static_tf,
        aruco_208_static_tf,
        aruco_209_static_tf,
        aruco_210_static_tf,
        aruco_211_static_tf,
        aruco_212_static_tf,
        aruco_213_static_tf,
        aruco_214_static_tf,
        aruco_215_static_tf,
        aruco_216_static_tf,
        aruco_300_static_tf,
        aruco_301_static_tf,
        aruco_302_static_tf,
        aruco_303_static_tf,
        aruco_304_static_tf,
        aruco_305_static_tf,
        aruco_306_static_tf,
        aruco_307_static_tf,
        aruco_308_static_tf,
        aruco_309_static_tf,
        aruco_400_static_tf,
        aruco_401_static_tf,
        aruco_402_static_tf,
        aruco_403_static_tf,
        aruco_404_static_tf,
        aruco_405_static_tf,
        aruco_406_static_tf,
        aruco_407_static_tf,
        aruco_408_static_tf,
        aruco_409_static_tf,
        aruco_410_static_tf,
        aruco_411_static_tf,
        aruco_412_static_tf,
        aruco_413_static_tf,
        aruco_414_static_tf,
        aruco_415_static_tf,
        aruco_416_static_tf,
        aruco_417_static_tf,
        camera_0_static_tf,
        camera_1_static_tf,
        camera_2_static_tf,
    ])