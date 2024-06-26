This project contains all the code written for our coursework for the second year course Introduction to Robotics at Heriot-Watt University. We implemented a tic-tac-toe game played with a human against a robotic manipulator. 

We used a [Kinova Gen3 Lite](https://www.kinovarobotics.com/product/gen3-lite-robots) and [Coppeliasim](https://www.coppeliarobotics.com/) for simulation, along with [ROS](https://ros.org/) (Humble) as the main part of the software stack.

This code in the repository is licensed under an MIT license, with the exception of files provided by Kinova inc. See [license](https://github.com/Abban-Fahim/b38ro-project?tab=License-1-ov-file) for more info.

## Instructions

To run the example coppelia simulation, in the first terminal source, source ROS

```bash
source /opt/ros/humble/setup.bash
```

and then open the coppeliaSim file: `coppelia/exampleSubPub.ttt`

In a second terminal, `cd` into `b38ro_ws`, and build + source the workspace with

```bash
colcon build
source install/setup.bash
```

Then run the example ROS node with

```bash
ros2 launch joints robot.launch.py
```

Now starting the simulation should show start logging whatever the node publishes.

This launch file should also start the rosbridge websocket server if you have the package `ros-humble-rosbridge-suite` installed. If not started, run

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Then open up UI by opening the file `gui/index.html` in the browser.

## Adding custom messages to Coppeliasim's ROS interface

As `Float32MultiArray` does not come with the included messages for coppelia. We need to add it and recompile the build.

Clone the repository for [simROS](https://github.com/CoppeliaRobotics/simROS2/) and add the line `export COPPELIASIM_ROOT_DIR=(coppelia directory)` to your `~/.bashrc`.

Add the following in the `interfaces.txt`: 

```
std_msgs/msg/Float32MultiArray
std_msgs/msg/MultiArrayLayout
std_msgs/msg/MultiArrayDimension
trajectory_msgs/msg/JointTrajectory
trajectory_msgs/msg/JointTrajectoryPoint
sensor_msgs/msg/JointState
geometry_msgs/msg/Twist
```

Change the cloned directory's name to `sim_ros2_interface` and run 

```bash
colcon build
```

It should about take 5 mins to complete the build and then you're good to go :)

## Running the physical robot

To simulate the moveit commander, run 

```bash
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config robot.launch.py robot_type:=gen3_lite gripper:=gen3_lite_2f robot_ip:=yyy.yyy.yy.yy use_fake_hardware:=True
```
 
To start the real robot, run 
 
```bash
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config robot.launch.py robot_model:=gen3_lite gripper:=gen3_lite_2f robot_ip:=192.168.1.10 use_fake_hardware:=False
```

To move to a cartesian position:

```bash
ros2 topic pub /cart_pose geometry_msgs/msg/Pose "{position:{x: 0.3, y: 0.425, z: 0.3}, orientation: {x: 3.142, y: 0, z: 3.142}}" -1
```
 
To send joint commands,
 
```bash
  ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [
      { positions: [2, 2, 2, 2, 2, 2], time_from_start: { sec: 10 } },
    ]
  }" -1
```

