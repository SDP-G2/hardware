# webots_ros2 integration

ROS2 packages for SDP Group 2's robot development and simulation

Packages:

* sdp - the main package that contains: 
  * ROS2 Nodes to control the robot
  * tennis court Webots environment used for the simulation
    * must be manually updated with the latest version of the environment
  * Robot PROTO file used in the simulation
* sdp_launch - the launcher package for the simulation



## Installation

#### Requirements 

* [ROS2 Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/)
* [webots_ros2 ](https://github.com/cyberbotics/webots_ros2) workspace 



#### Installation process (Windows):

**Remember to source Foxy and webots_ros2 workspace first!!!** 

(all paths are relative to your webots_ros2 workspace directory)

1. Put both packages in ./src/webots_ros2/ directory

2. Build the packages

   ```powershell
   colcon build --packages-select sdp
   ```

   ```powershell
   colcon build --packages-select sdp_launch
   ```

3. Source the workspace after the build



## Usage (Windows)

**Remember to source Foxy and webots_ros2 workspace first!!!** 

### Running simulation

```powershell
ros2 launch sdp_launch sdp.launch.py
```

![image-20210204144834856](C:\Users\kpija\AppData\Roaming\Typora\typora-user-images\image-20210204144834856.png)

### Running specific nodes

```powershell
ros2 run sdp <entry_point_name>
```