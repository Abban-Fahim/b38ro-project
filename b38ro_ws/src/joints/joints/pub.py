import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Duration
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from ament_index_python import get_package_prefix
import ikpy.chain

jointNames = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


# Define a class for our ROS node
class RobotCommander(Node):

    # Constructor method
    def __init__(self):
        super().__init__("robot_commander")  # Initialize the node with name "joints"

        # Create a publisher for target angles
        self.target_angles = self.create_publisher(
            JointTrajectory, "joint_trajectory_controller/joint_trajectory", 10
        )

        # Action client for moving the gripper
        self.gripper_client = ActionClient(
            self, GripperCommand, "robotiq_gripper_controller/gripper_cmd"
        )

        # Create a subscription to recieve arm and gripper target positions
        self.create_subscription(Pose, "cart_pose", self.move_to_angles, 10)
        self.create_subscription(Float32, "gripper_pose", self.move_gripper, 10)

        # Get the package directory path
        pkg_dir = get_package_prefix("joints")

        # Load robot chain from URDF file
        self.robot = ikpy.chain.Chain.from_urdf_file(
            pkg_dir + "/../../../coppelia/GEN3-LITE.urdf",
            ["BASE"],
            active_links_mask=[True, True, True, True, True, True, True, False],
        )

    # Method to calculate joint angles and publish target angles
    def move_to_angles(self, msg: Pose):
        # Calculate inverse kinematics for the recived cartesian positions
        angles = self.robot.inverse_kinematics(
            [msg.position.x, msg.position.y, msg.position.z]
        )

        # Create a new message for publishing
        traj = JointTrajectory()
        traj.joint_names = jointNames
        point = JointTrajectoryPoint()
        point.positions = angles[1:7].tolist()
        point.time_from_start = Duration(seconds=10).to_msg()
        traj.points.append(point)
        print(angles)

        self.target_angles.publish(traj)

    def move_gripper(self, msg: Float32):
        gripper_msg = GripperCommand.Goal()
        gripper_msg.command.position = msg.data
        gripper_msg.command.max_effort = 100.0
        print(gripper_msg)
        self.gripper_client.send_goal_async(gripper_msg)


# Boilerplate ROS code for making a node
def main():
    rclpy.init()
    robot_commander = RobotCommander()
    rclpy.spin(robot_commander)
    robot_commander.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
