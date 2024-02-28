# Planning

Primary tasks (required):

- inverse kinematics (move end effector to a desired Cartesian pose)
- movement with desired translational and rotational velocity
- plan and make robot follow Cartesian trajectories
- read, write and visualise joint angles
- read, compute, write and plot Cartesian positions (FK) and Euler angles of robot links
- read, write and compute Jacobian matrices (taught fi week 8)
- be able to move the robot with controllers/keyboard or other interesting stuff

Report (5-10 pages):

- what our work simulates
- what and how was theoretical material used for developing our simulation
- what's good bout our simulation and any real applications
- data, graphs, numbers, results etc about or simulations

Our tasks and project outline:

- streamlining ROS and version control to work on the software
- research... lots of it, including but not limited to
  - python programming w/ numpy, ROS 2 and OOP
  - FK and IK libraries compatible with python
  - tools to easily for plotting and visualising stuff
  - computer vision for our game application (openCV, aruco markers)
- develop generic classes that fulfil the required tasks **(highest priority)**
- develop on our game solution
- plan and develop our integration with the real kinova gen3 lite

# Instructions

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
ros2 run joints talker
```

Now starting the simulation should show start logging whatever the node publishes.


# Before Getting Started

As `UInt16MultiArray` and `Int16MultiArray` do not come with the standard `ROS2 std_msgs` library for coppelia.
<br><br>
We need to add it and recompile the build.
<br><br>
Go to the directory where Coppelia has been installed and edit the `interfaces.txt` file.
<br>

The path to that is: 
```bash 
(coppeliasim dir)/programming/ros2_packages/sim_ros2_interface/meta/interfaces.txt
```

Add the following in the `interfaces.txt`: 

```bash
std_msgs/msg/UInt16MultiArray
std_msgs/msg/Int16MultiArray
std_msgs/msg/MultiArrayLayout
std_msgs/msg/MultiArrayDimension
```

Change directory to `sim_ros2_interface` and run 
```bash
colcon build
```
It should take 5 mins to complete the build and then you're good to go :)

## Notes
Video feed through fone (termux/web)



