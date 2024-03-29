import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pykin.robots.single_arm import SingleArm  # Import SingleArm robot class from Pykin
from pykin.kinematics.transform import Transform  # Import Transform class from Pykin for defining transformations
import numpy as np

jointNames = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


# Define a class for our ROS node
class MinimalPublisher(Node):

    # Constructor method
    def __init__(self):
        super().__init__("joints")  # Initialize the node with name "joints"

        # Create a publisher for target angles
        self.target_angles = self.create_publisher(
            JointTrajectory, "joint_trajectory_controller/joint_trajectory", 10
        )
        # Create a subscription to recieve cartesian target positions
        self.create_subscription(Pose, "cart_pose", self.move_to_angles, 10)

        # Define the file path to the URDF file
        file_path = '../assets/urdf/KR7108-URDF/KR7108-URDF.urdf'
        
        # Load the robot model from the URDF file
        self.robot = SingleArm(file_path, Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0]))
        self.robot.setup_link_name("BASE", "DUMMY")

    # Method to calculate joint angles and publish target angles
    def move_to_angles(self, msg: Pose):

        # Initial joint angles
        init_thetas = [0, 0, 0, 0, 0, 0]

        # NOTE
        # Change the line of code above to a correct one.
        # init_thetas = [0, 0, 0, 0, 0, 0]
        # I have a feeling it's wrong.

        # Calculate forward kinematics to get the initial end-effector pose
        self.robot.forward_kin(init_thetas)

        cart_pose = np.concatenate((msg.position, msg.orientation))
        # Calculate inverse kinematics for the recived cartesian positions
        angles = self.robot.inverse_kinematics(
            init_thetas, cart_pose
        )

        # Create a new message for publishing
        traj = JointTrajectory()
        traj.joint_names = jointNames
        point = JointTrajectoryPoint()
        point.positions = angles.tolist()
        point.time_from_start = Duration(seconds=10).to_msg()
        traj.points.append(point)

        print(angles, len(jointNames), traj.points[0])
        print(traj)
        self.target_angles.publish(traj)


# Boilerplate ROS code for making a node
def main():
    rclpy.init()
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
