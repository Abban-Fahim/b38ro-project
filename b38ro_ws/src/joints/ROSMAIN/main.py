# import ROS
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import Pose
import math
import time

# import libs
# import picplace.py as p
# import tttai.py as t
# import waitformove.py as w
from .libs.move import m

# import firstmove.py as fm


# initalisation
# r.setup()
# m.def_pos()#move robot arm to default position
# p_s=m.det_bord_cart()#calibrate the position of each sqaure
# b_s=[0,0,0,0,0,0,0,0,0]

# # determine who goes first
# if (fm.h_first()):
# 	md = w.wait()#wait for user to make move
# 	#current implement also has user input move they made
# 	b_s[md]=1
# 	#if we go with comp vision we'll have 3 nodes ,one for move detection ,one for board position and one for main logic


class Game(Node):
    def __init__(self):
        super().__init__("game_logic")

        # self.create_subscription(Joy, "/joy", self.controller_cb, 10)

        self.coords = m()

        self.position_topic = self.create_publisher(Pose, "/cart_pose", 10)

        for coord in self.coords:
            newMsg = Pose()
            newMsg.position.x = coord[0]
            newMsg.position.y = coord[1]
            newMsg.position.z = 0.2

            newMsg.orientation.x = math.pi
            newMsg.orientation.y = 0
            newMsg.orientation.z = 0

            self.position_topic.publish()
            time.sleep(5)


def main():
    rclpy.init()
    node = Game()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
