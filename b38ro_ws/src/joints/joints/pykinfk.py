# Forward kinematics with Pykin

# Import necessary libraries
import numpy as np
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils import plot_utils as p_utils

# Define the file path to the URDF file
file_path = '../assets/urdf/KR7108-URDF/KR7108-URDF.urdf'

# Initialize a SingleArm robot object with the URDF file and set its initial pose
robot = SingleArm(file_path, Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0]))

# Set up the links
robot.setup_link_name("BASE", "END_EFFECTOR")

# Define the target joint angles for forward kinematics
target_thetas = [-0.470, -1.735, 2.480, -2.275, -1.590, -1.991]

# Set the robot's configuration to the target joint angles
robot.set_transform(target_thetas)

# Initialize a 3D figure for visualization
_, ax = p_utils.init_3d_figure("FK")
p_utils.plot_robot(ax=ax, robot=robot, geom="visual", only_visible_geom=True, alpha=1)
p_utils.show_figure()

