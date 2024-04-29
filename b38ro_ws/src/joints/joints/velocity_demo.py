import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from controller_manager.controller_manager_services import SwitchController
from example_interfaces.srv import Trigger

from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import (
    Transform,
)
from pykin.kinematics.jacobian import calc_jacobian

from rclpy.time import Duration
import scipy.spatial.transform as sp
import numpy as np

jointNames = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class JointsPublisher(Node):

    def __init__(self):
        super().__init__("joints")

        # Create a publisher for target joint angles
        self.target_angles = self.create_publisher(
            JointTrajectory, "joint_trajectory_controller/joint_trajectory", 10
        )

        # Publisher for FK computation
        self.robot_position = self.create_publisher(Pose, "robot_position", 10)
        self.robot_jacobians = self.create_publisher(
            Float32MultiArray, "robot_jacobian", 10
        )

        # Action clients for restting the arm in case of a fault
        # self.create_subscription(Bool, "reset_arm", self.reset_arm, 10)
        # self.reset_fault = ActionClient(self, Trigger, "fault_controller/reset_fault")
        # self.controller_switcher = ActionClient(
        #     self, SwitchController, "controller_manager/switch_controller"
        # )

        # receive Cartesian poses to move to
        self.create_subscription(Pose, "cart_pose", self.move_to_angles, 10)

        # Reci
        self.create_subscription(Float32MultiArray, "target_vel", self.target_vel, 10)
        self.eef_vel = np.ones(6)
        print(self.eef_vel)

        # recieve robot's current joint angles
        self.create_subscription(JointState, "joint_states", self.set_robot_state, 10)

        # Load robot from URDF file
        file_path = "../assets/urdf/KR7108-URDF/KR7108-URDF.urdf"
        self.robot = SingleArm(file_path, Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0]))
        self.robot.setup_link_name("BASE", "END_EFFECTOR")

        self.robot_angles = [0, 0, 0, 0, 0, 0]
        self.robot_velocities = [0, 0, 0, 0, 0, 0]
        self.jacobian = np.zeros((6, 6))

    def target_vel(self, msg: Float32MultiArray):
        self.eef_vel = np.array(msg.data)
        print(self.eef_vel, self.eef_vel.shape)

    def set_robot_state(self, msg: JointState):
        # set robot's joint config from message
        # Since this message is published by the low-level code on the arm driver,
        # the order in which the joint values are published are is psuedo-random.
        # The name attribute of the msg contains an array in which these joints are published
        # Explanation: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/619
        for i in range(len(msg.name)):
            # Is a joint angle
            joint_name = msg.name[i]
            if joint_name.startswith("joint_"):
                # Get last character in the name, containing number
                num = int(joint_name[len(joint_name) - 1]) - 1
                self.robot_angles[num] = msg.position[i]
                self.robot_velocities[num] = msg.velocity[i]
                # print(num, joint_name, i)

        # forward kinematics to find end-effector Pose
        fk = self.robot.forward_kin(self.robot_angles)

        # Calculate and publish jacobian
        self.jacobian = calc_jacobian(self.robot.desired_frames, fk, 6)
        jacMsg = Float32MultiArray()
        jacMsg.data = self.jacobian.flatten().tolist()
        self.robot_jacobians.publish(jacMsg)

        fk = fk["END_EFFECTOR"]
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

        # Publish FK of robot
        self.robot_position.publish(pos)

    def move_to_angles(self, msg: Pose):

        # Calculate forward kinematics to get the initial end-effector pose
        self.robot.set_transform(self.robot_angles)

        # Calculate inverse kinematics for the received Cartesian pose
        # Convert Euler angles recived to a quaternion
        eulers = [msg.orientation.x, msg.orientation.y, msg.orientation.z]
        quat = sp.Rotation.from_euler("ZYX", eulers).as_quat()
        # quat = [0, 0, 1, 1]
        angles = self.robot.inverse_kin(
            self.robot_angles,
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

        # Calculate joint velocities based on eef_vel
        try:
            angleVel = np.matmul(np.linalg.inv(self.jacobian), self.eef_vel)

            print(np.linalg.inv(self.jacobian), self.eef_vel)
            print(angleVel)
        except np.linalg.LinAlgError:
            print("cant find inverse of jacobian")
            angleVel = np.zeros(0)

        # Publish only one point to JointTrajectory
        traj = JointTrajectory()
        traj.joint_names = jointNames
        point = JointTrajectoryPoint()
        point.positions = angles.tolist()
        point.velocities = angleVel.tolist()  # add velocities to joints commander
        point.time_from_start = Duration(seconds=3).to_msg()
        traj.points.append(point)

        # Publish the calculated joint angles
        # print(eulers, angles)
        self.target_angles.publish(traj)


def main():
    rclpy.init()
    joints_publisher = JointsPublisher()
    rclpy.spin(joints_publisher)
    joints_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
