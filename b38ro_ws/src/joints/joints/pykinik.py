# Importing necessary libraries
import numpy as np
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils import plot_utils as p_utils

# Define the file path to the URDF file
file_path = '../assets/urdf/KR7108-URDF/KR7108-URDF.urdf'

# Initialize the SingleArm robot object with the URDF file path
robot = SingleArm(file_path, Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0]))

# Chain the robot links from BASE to DUMMY
robot.setup_link_name("BASE", "DUMMY")

# Define the target end-effector pose (position and orientation)
target_pose = [0.8, 0.8, 0.8, 1, 0, 0, 0]

# Initial joint angles (thetas) for the robot
init_thetas = [0, 0, 0, 0, 0, 0]

# Compute forward kinematics to get the initial end-effector pose
fk = robot.forward_kin(init_thetas)

# Perform inverse kinematics to compute joint angles for the given target pose
joints = robot.inverse_kin(init_thetas, target_pose)

# Set the robot's joint angles to the computed joint angles
robot.set_transform(joints)

# Print the computed joint angles
print(joints)

# Initialize a 3D figure for visualization
_, ax = p_utils.init_3d_figure("IK")
p_utils.plot_robot(ax=ax, robot=robot, geom="visual", only_visible_geom=True)
p_utils.show_figure()


