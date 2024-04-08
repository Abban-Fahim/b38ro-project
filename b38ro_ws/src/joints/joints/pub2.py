import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import (
    Transform,
)
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.time import Duration
import scipy.spatial.transform as sp

jointNames = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("joints")

        # Create a publisher for target joint angles
        self.target_angles = self.create_publisher(
            JointTrajectory, "joint_trajectory_controller/joint_trajectory", 10
        )

        # Publisher for FK computation
        self.robot_position = self.create_publisher(Pose, "robot_position", 10)

        # Action client for moving the gripper
        self.gripper_client = ActionClient(
            self, GripperCommand, "robotiq_gripper_controller/gripper_cmd"
        )

        # receive Cartesian poses to move to
        self.create_subscription(Pose, "cart_pose", self.move_to_angles, 10)
        self.create_subscription(Float32, "gripper_pose", self.move_gripper, 10)

        # recieve robot's current joint angles
        self.create_subscription(JointState, "joint_states", self.set_robot_state, 10)

        # Load robot from URDF file
        file_path = "../assets/urdf/KR7108-URDF/KR7108-URDF.urdf"
        self.robot = SingleArm(file_path, Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0]))
        self.robot.setup_link_name("BASE", "END_EFFECTOR")

        self.robot_state = [0, 0, 0, 0, 0, 0]

    def set_robot_state(self, msg: JointState):
        self.robot_state = msg.position  # set robot's joint config from message
        self.robot_state.pop(1)  # remove the second element containing gripper position
        fk = self.robot.forward_kin(self.robot_state)[
            "END_EFFECTOR"
        ]  # forward kinematics to find end-effector Pose
        pos = Pose()
        pos.position.x, pos.position.y, pos.position.z = fk.pos
        # convert from quaternion to ZYX Euler angles
        quat = sp.Rotation.from_quat(fk.rot)
        eulers = quat.as_euler("ZYX")
        (
            pos.orientation.x,
            pos.orientation.y,
            pos.orientation.z,
        ) = eulers

        self.robot_position.publish(pos)

    def move_to_angles(self, msg: Pose):

        print(msg)

        # Calculate forward kinematics to get the initial end-effector pose
        self.robot.set_transform(self.robot_state)

        # Calculate inverse kinematics for the received Cartesian pose
        # Convert Euler angles recived to a quaternion
        eulers = [msg.orientation.x, msg.orientation.y, msg.orientation.z]
        quat = sp.Rotation.from_euler("ZYX", eulers).as_quat()
        # quat = [0, 0, 1, 1]
        angles = self.robot.inverse_kin(
            self.robot_state,
            [
                msg.position.x,
                msg.position.y,
                msg.position.z,
                quat[0],
                quat[1],
                quat[2],
                quat[3],
            ],
        )

        # Publish only one point to JointTrajectory
        traj = JointTrajectory()
        traj.joint_names = jointNames
        point = JointTrajectoryPoint()
        point.positions = angles.tolist()
        point.time_from_start = Duration(seconds=0).to_msg()
        traj.points.append(point)

        # Publish the calculated joint angles
        # print(eulers, angles)
        self.target_angles.publish(traj)

    def move_gripper(self, msg: Float32):
        print("Recived gripper goal", msg.data)
        gripper_msg = GripperCommand.Goal()
        gripper_msg.command.position = msg.data
        gripper_msg.command.max_effort = 100.0
        print(gripper_msg)
        self.gripper_client.send_goal_async(gripper_msg)


def main():
    rclpy.init()
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
