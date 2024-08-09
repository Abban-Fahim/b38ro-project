import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32, Int32, Int32MultiArray, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import time 



class JointsPublisher(Node):

    def __init__(self):
        super().__init__("joints")
    
        #subscribe to feedback 
        self.create_subscription(Int32, "/feedback", self.feedba, 10)
        #subscribe to list of moves 
        self.create_subscription(Int32MultiArray, "/moveset",self.moveset, 10)
        #create move publisher 
        self.position_topic = self.create_publisher(Pose, "/cart_pose", 10)
        #create gripper publisher 
        self.gripper_topic = self.create_publisher(Float32, "/gripper_pose", 10)
        #create position tracker 
        self.n = 0 
        #create looper
        self.looper = self.create_timer(
            1,
            lambda:(
                self.pubnext() if (self.feedba == 1 ) else print("waiting")
            )
        )
    
    def pubnext(self):
        


def main():
    rclpy.init()
    joints_publisher = JointsPublisher()
    rclpy.spin(joints_publisher)
    joints_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
