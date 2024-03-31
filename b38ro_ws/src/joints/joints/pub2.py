# Import necessary libraries for ROS 2
import rclpy  # Import the ROS 2 Python library
from rclpy.node import Node  # Import the Node class for creating ROS nodes
from std_msgs.msg import (
    String,
    Float32MultiArray,
)  # Import message types for communication
from pykin.robots.single_arm import SingleArm  # Import SingleArm robot class from Pykin
from pykin.kinematics.transform import (
    Transform,
)  # Import Transform class from Pykin for defining transformations
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.time import Duration

jointNames = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


# Define a class for our ROS node
class MinimalPublisher(Node):

    # Constructor method
    def __init__(self):
        super().__init__("joints")  # Initialize the node with name "joints"

        # Create a publisher for target joint angles
        self.target_angles = self.create_publisher(
            JointTrajectory, "joint_trajectory_controller/joint_trajectory", 10
        )
        self.robot_position = self.create_publisher(Pose, "robot_position", 10)

        # receive Cartesian poses to move to
        self.create_subscription(Pose, "cart_pose", self.move_to_angles, 10)

        self.create_subscription(
            Float32MultiArray, "robot_state", self.set_robot_state, 10
        )

        # Load robot from URDF file
        file_path = "../assets/urdf/KR7108-URDF/KR7108-URDF.urdf"
        self.robot = SingleArm(file_path, Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0]))
        self.robot.setup_link_name("BASE", "DUMMY")

        self.robot_state = [0, 0, 0, 0, 0, 0]

    def set_robot_state(self, msg):
        self.robot_state = msg.data
        fk = self.robot.forward_kin(self.robot_state)["END_EFFECTOR"]
        pos = Pose()
        pos.position.x, pos.position.y, pos.position.z = fk.pos
        (
            pos.orientation.x,
            pos.orientation.y,
            pos.orientation.z,
            pos.orientation.w,
        ) = fk.rot
        self.robot_position.publish(pos)

    # Method to calculate joint angles and publish target angles
    def move_to_angles(self, msg: Pose):

        # Calculate forward kinematics to get the initial end-effector pose
        self.robot.forward_kin(self.robot_state)
        print(self.robot_state)

        # Calculate inverse kinematics for the received Cartesian pose
        angles = self.robot.inverse_kin(
            self.robot_state,
            [
                msg.position.x,
                msg.position.y,
                msg.position.z,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ],
        )

        # Create a new message for publishing
        traj = JointTrajectory()
        traj.joint_names = jointNames
        point = JointTrajectoryPoint()
        point.positions = angles.tolist()
        point.time_from_start = Duration(seconds=10).to_msg()
        traj.points.append(point)

        # Publish the calculated joint angles
        self.target_angles.publish(traj)


# Main function to initialize the node and start the event loop
def main():
    rclpy.init()  # Initialize the ROS 2 client library
    minimal_publisher = MinimalPublisher()  # Create an instance of MinimalPublisher
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()  # Explicitly destroy the node
    rclpy.shutdown()  # Shutdown the ROS 2 client library


# Entry point of the script
if __name__ == "__main__":
    main()  # Call the main function to start the node
