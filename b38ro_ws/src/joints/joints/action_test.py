import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool ,Int32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

from moveit_msgs.action import ExecuteTrajectory

class JointsPublisher(Node):

    def __init__(self):
        super().__init__("joints")

        self.client = ActionClient(self,ExecuteTrajectory,"/execute_trajectory")
        
        req = ExecuteTrajectory.Goal()
        req.trajectory.joint_trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        p = JointTrajectoryPoint()
        p.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        req.trajectory.joint_trajectory.points.append(p)
        p.positions = [-91.0* 0.0174533, 155.0* 0.0174533, 162.0* 0.0174533, -90.0* 0.0174533, 156.0* 0.0174533, 0.0]
        req.trajectory.joint_trajectory.points.append(p)

        self.client.send_goal(req)
    


def main():
    rclpy.init()
    joints_publisher = JointsPublisher()
    rclpy.spin(joints_publisher)
    joints_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
