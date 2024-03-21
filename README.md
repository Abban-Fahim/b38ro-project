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

## Run the web-based GUI

Install the package `ros-humble-rosbridge-suite` and start the rosbridge websocket server.

## Adding custom messages to Coppeliasim's ROS interface

As `Float32MultiArray` does not come with the included messages for coppelia. We need to add it and recompile the build.

Clone the repository for (simROS)[https://github.com/CoppeliaRobotics/simROS2/] and add the line `export COPPELIASIM_ROOT_DIR=(coppelia directory)` to your `~/.bashrc`.

Add the following in the `interfaces.txt`: 

```bash
std_msgs/msg/Float32MultiArray
std_msgs/msg/MultiArrayLayout
std_msgs/msg/MultiArrayDimension
```

Change the cloned directory's name to `sim_ros2_interface` and run 

```bash
colcon build
```

It should about take 5 mins to complete the build and then you're good to go :)

## Notes

Video feed through fone (termux/web)

## NUFTFY

- how to pick

