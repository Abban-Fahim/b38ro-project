from pykin.robots.single_arm import SingleArm
from pykin.robots.bimanual import Bimanual
from pykin.kinematics.transform import (
    Transform,
)
import numpy as np
from pykin.kinematics.jacobian import calc_jacobian
import struct

robot = SingleArm(
    "../assets/urdf/KR7108-URDF/KR7108-URDF.urdf",
    Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0]),
)
robot.setup_link_name("BASE", "END_EFFECTOR")

print(robot.forward_kin(np.zeros((6, 1)))["END_EFFECTOR"])
# print(robot.forward_kin(np.zeros((6, 1)))["DUMMY"])
# print(robot.compute_eef_pose())

fk = robot.forward_kin(np.zeros((6, 1)))
# print(robot.generate_desired_frame_recursive("BASE", "END_EFFECTOR"))
print(robot.desired_frames)
J = calc_jacobian(robot.desired_frames, fk, 6)
print(J)
a = struct.pack(b=struct.pack("=%sf" % J.size, *J.flatten("F")))
print(a)
