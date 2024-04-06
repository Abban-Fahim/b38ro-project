from scipy.spatial.transform.rotation import Rotation
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils import plot_utils as p_utils
import numpy as np

file_path = "../assets/urdf/KR7108-URDF/KR7108-URDF.urdf"
robot = SingleArm(file_path, Transform(pos=[0, 0, 0], rot=[0.0, 0.0, 0.0]))
robot.setup_link_name("BASE", "END_EFFECTOR")

target_thetas = [0, 0, 0, 0, 0, 0]

robot.set_transform(target_thetas)
fk = robot.forward_kin(target_thetas)
print(fk["END_EFFECTOR"])
pointing_down = Rotation.from_matrix(
    np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
).as_quat()
print(pointing_down)
fk["END_EFFECTOR"]
T = Transform(pos=np.identity(3), rot=pointing_down).h_mat()
print(T)
# Initialize a 3D figure for visualization
# _, ax = p_utils.init_3d_figure("FK")
# p_utils.plot_robot(ax=ax, robot=robot, geom="visual", only_visible_geom=True, alpha=1)
# p_utils.show_figure()
