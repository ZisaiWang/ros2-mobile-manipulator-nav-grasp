# ROS2 Mobile Manipulator Navigation and Grasping Simulation

## 中文简介

本项目基于 **ROS2 Jazzy、Gazebo Sim、Nav2、slam_toolbox、AMCL、ros2_control 和 OpenCV** 搭建了一个移动操作机器人仿真系统。

系统主要实现了：

- 基于 2D LiDAR 的 SLAM 建图
- 地图保存与加载
- 基于 AMCL 的定位
- 基于 Nav2 的目标点自主导航
- RGB-D 相机目标检测
- 红色目标物体三维定位
- 基于简化逆运动学的机械臂控制
- 视觉引导的小车靠近与机械臂抓取演示

本项目主要用于 ROS2 移动机器人与移动操作机器人方向的学习和实践，覆盖了机器人建模、传感器仿真、SLAM、导航、视觉检测、坐标变换和机械臂控制等内容。

---

## English Introduction

This project implements a **ROS2-based mobile manipulator simulation system** using **ROS2 Jazzy, Gazebo Sim, Nav2, slam_toolbox, AMCL, ros2_control, and OpenCV**.

The system supports:

- 2D LiDAR-based SLAM mapping
- Map saving and loading
- AMCL localization
- Nav2 autonomous navigation
- RGB-D camera simulation
- Red object detection using OpenCV
- Depth-based 3D object localization
- Simplified inverse-kinematics-based arm control
- Vision-guided mobile base approaching and grasping demo

This project is designed as a ROS2 learning and robotics simulation demo. It integrates mobile robot modeling, SLAM, navigation, perception, coordinate transformation, and basic manipulation control.

---

## Demo

### SLAM Mapping Demo

[SLAM Mapping Demo Video](media/mapping.webm)

This demo shows the robot building a 2D occupancy grid map in Gazebo using `slam_toolbox` and a simulated 2D LiDAR.

### Navigation and Vision-Guided Grasping Demo

[Navigation and Grasping Demo Video](media/working.webm)

This demo shows the robot navigating in the saved map and using RGB-D perception with simplified inverse kinematics to approach and grasp a red object.

---

## Project Features

- Mobile robot model built with URDF/Xacro
- Custom Gazebo warehouse world
- 2D LiDAR simulation
- RGB-D camera simulation
- SLAM mapping using `slam_toolbox`
- Map saving using `nav2_map_server`
- Saved-map localization using AMCL
- Autonomous navigation using Nav2
- Local and global costmap configuration
- Red object detection using OpenCV
- Depth image and camera intrinsic based 3D localization
- TF transformation from camera frame to `base_link`
- Simplified IK calculation for a 3-DOF arm
- Gripper control using `ros2_control`
- Vision-guided grasping demo

---

## Project Structure

```text
ros2_mobile_manipulator_project/
├── my_robot_description/
│   ├── urdf/
│   ├── rviz/
│   └── ...
│
├── my_robot_bringup/
│   ├── launch/
│   ├── config/
│   ├── worlds/
│   ├── map/
│   └── ...
│
├── my_py_pkg/
│   ├── my_py_pkg/
│   │   ├── __init__.py
│   │   └── vision_pick_demo.py
│   ├── resource/
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── media/
│   ├── mapping.webm
│   └── working.webm
│
├── README.md
└── .gitignore
```

---

## Environment

Tested environment:

```text
Ubuntu 24.04
ROS2 Jazzy
Gazebo Sim
Nav2
slam_toolbox
ros_gz_bridge
ros2_control
OpenCV
```

Install required dependencies:

```bash
sudo apt update

sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-ros-gz-bridge
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-cv-bridge
sudo apt install ros-jazzy-tf2-geometry-msgs
sudo apt install python3-opencv
```

---

## Build

Create or enter a ROS2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Put the following packages into `~/ros2_ws/src`:

```text
my_robot_description
my_robot_bringup
my_py_pkg
```

Build all packages:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Or build only this project:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description my_robot_bringup my_py_pkg
source install/setup.bash
```

---

## 1. SLAM Mapping

Launch the robot, Gazebo world, RViz, LiDAR, RGB-D camera, bridge, controllers, and `slam_toolbox`:

```bash
ros2 launch my_robot_bringup robot_with_slam.launch.xml
```

In this launch file, the arm is automatically moved to a folded pose after about 10 seconds to reduce interference during mapping.

Use keyboard teleoperation to move the robot:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Move the robot slowly around the environment to build a clean map.

If the arm needs to be moved manually to a folded pose:

```bash
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 1.414, 1.414, 0.03, 0.03]}"
```

If the arm needs to be moved to an upright pose:

```bash
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}"
```

---

## 2. Save the Map

After mapping is complete, keep the SLAM launch running and open a new terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/my_robot_bringup/map/warehouse_map
```

If the map saver times out, use a longer timeout:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/src/my_robot_bringup/map/warehouse_map \
  --ros-args -p save_map_timeout:=30.0
```

The saved map files are:

```text
warehouse_map.yaml
warehouse_map.pgm
```

---

## 3. Launch Robot with Saved Map

After saving the map, launch the robot with the saved map and AMCL localization:

```bash
ros2 launch my_robot_bringup robot_with_map.launch.xml
```

If the map is not displayed in RViz, set the Map display QoS:

```text
Map -> QoS -> Durability -> Transient Local
```

If the robot pose is not aligned with the map, use the following tool in RViz:

```text
2D Pose Estimate
```

---

## 4. Start Nav2 Navigation

Open a new terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
```

Launch Nav2:

```bash
ros2 launch my_robot_bringup nav2.launch.py
```

In RViz, use:

```text
Nav2 Goal
```

to send a navigation target.

If you need to check the robot pose estimated by AMCL:

```bash
ros2 topic echo /amcl_pose --once
```

If you need to check a clicked map coordinate from RViz:

```bash
ros2 topic echo /clicked_point
```

Then use the RViz tool:

```text
Publish Point
```

to click on the map.

---

## 5. Vision-Guided Grasping Demo

Make sure the robot, RGB-D camera, TF tree, arm controller, and Gazebo simulation are running.

Open a new terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
```

Run the vision-guided grasping node:

```bash
ros2 run my_py_pkg vision_pick_demo
```

The node performs the following steps:

```text
1. Subscribe to RGB image
2. Detect red object using OpenCV
3. Subscribe to depth image
4. Estimate the 3D position of the red object
5. Transform object position to base_link using TF
6. Use /cmd_vel to approach the object
7. Stop the robot at a graspable distance
8. Solve simplified inverse kinematics
9. Control the arm and gripper through /arm_controller/commands
10. Close the gripper and lift the object
```

---

## 6. Useful Commands

### Check camera topics

```bash
ros2 topic list | grep camera
```

### View RGB or depth image

```bash
ros2 run rqt_image_view rqt_image_view
```

### Check LiDAR topic

```bash
ros2 topic echo /scan --once
```

### Check map topic

```bash
ros2 topic echo /map --once --qos-durability transient_local | head -20
```

### Check TF from map to base_link

```bash
ros2 run tf2_ros tf2_echo map base_link
```

### Check TF from base_link to camera frame

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

### Check TF from base_link to lidar_link

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
```

### Check Nav2 lifecycle states

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /global_costmap/global_costmap
ros2 lifecycle get /local_costmap/local_costmap
```

### Check costmap topics

```bash
ros2 topic list | grep costmap
```

### Check velocity command

```bash
ros2 topic echo /cmd_vel
```

### Check arm controller command topic

```bash
ros2 topic info /arm_controller/commands
```

### Move arm to a folded pose

```bash
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 1.414, 1.414, 0.03, 0.03]}"
```

### Move arm to an upright pose

```bash
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}"
```

---

## 7. Notes on RViz

If the saved map is not shown in RViz, set:

```text
Map -> QoS -> Durability -> Transient Local
```

If navigation fails, check:

```text
1. Whether the map is clean
2. Whether the robot initial pose is correct
3. Whether the target point is in a free area
4. Whether /scan is valid
5. Whether global and local costmaps are active
6. Whether /cmd_vel is being published
```

If the robot only rotates or backs up, possible reasons include:

```text
- The target point is inside an obstacle or inflated area
- The map contains black noise points
- The local costmap detects obstacles in front of the robot
- The robot initial pose is not aligned with the map
- Multiple nodes are publishing /cmd_vel at the same time
```

---

## 8. Technical Details

### SLAM

The project uses `slam_toolbox` for 2D LiDAR-based mapping.  
The generated map is saved by `nav2_map_server`.

### Localization

The saved map is loaded by `map_server`, and AMCL is used for localization.

### Navigation

Nav2 is used for autonomous navigation.  
The global planner uses a grid-based planner, while the local controller follows the generated path using a local planner.

### Perception

The RGB-D camera is used for object detection and localization.  
OpenCV is used to segment red objects in the RGB image.  
The corresponding depth value is used to estimate the 3D position of the object.

### Manipulation

The arm is controlled through `ros2_control`.  
A simplified inverse kinematics method is used instead of MoveIt.  
The calculated joint positions are published to:

```text
/arm_controller/commands
```

The command format is:

```text
[base_arm_base_joint, base_arm_arm1_joint, arm1_arm2_joint, left_finger, right_finger]
```

---



## License

This project is for learning and research demonstration purposes.
