import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32, Int32, Int32MultiArray, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


jointNames = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class JointsPublisher(Node):

    def __init__(self):
        super().__init__("joints")

        # Create a publisher for target joint angles
        self.create_subscription(JointTrajectory,'/joint_trajectory_controller/joint_trajectory',self.targ,10)

        # recieve robot's current joint angles
        self.create_subscription(JointState, "joint_states", self.curstate, 10)

        # create feedback publisher 
        self.feedback = self.create_publisher(Int32, "/feedback", 10)
        
        self.targstate = [0]
    def targ(self ,msg :JointTrajectory):
        
        self.targstate = list(msg.points[0].positions)

    def curstate(self, msg: JointState ):
        self.curstate = list(msg.position)

        print(self.curstate)

        print(self.targstate)

        tem = 1 # idle / ~ at target

        for a,b in zip(self.curstate,self.targstate):
            tol = 0.05*max(abs(a),abs(b))
            if abs(a-b) > tol :
                tem = 0
        self.feedbackval = Int32()
        self.feedbackval.data = tem
        print(self.feedbackval)
        # Publish FK of robot
        self.feedback.publish(self.feedbackval)






def main():
    rclpy.init()
    joints_publisher = JointsPublisher()
    rclpy.spin(joints_publisher)
    joints_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
