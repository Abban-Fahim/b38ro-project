# import ROS
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
import math
import time

# import libs
# import picplace.py as p
# import tttai.py as t
# import waitformove.py as w
from .move import det_bord_cart
from .tttai import dec
from .tttai import win

# import firstmove.py as fm


class Game(Node):
    def __init__(self):
        super().__init__("game_logic")

        # self.create_subscription(Joy, "/joy", self.controller_cb, 10)

        # self.coords = det_bord_cart()

        self.position_topic = self.create_publisher(Pose, "/cart_pose", 10)
        self.gripper_topic = self.create_publisher(Float32, "gripper_pose", 10)
        self.gripper_value = Float32()

        # store board state 0=empty ,2=ai taken ,1=player taken
        self.b_s = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        # store current player and winner if any
        self.win = 0

        # store board carteisan position and calculate cp of each point
        self.p1 = [0.3, -0.15]  # future implement method to
        self.p2 = [0.6, 0.15]  # find positions and calibrate with cp ,do in move.py

        self.b_cp = det_bord_cart(self.p1, self.p2)

        # store location of cuboid location to pickup from
        self.pp = [0.3,0.425] 


        # define resting pose
        self.retract = Pose()
        self.retract.position.x = 0.45
        self.retract.position.y = 0.0
        self.retract.position.z = 0.4

        # main game logic

        # det if shuman / ai go first
        self.turn = int(input("Who go first ,AI =2,HUMAN = 1"))
        self.playing = True
        print("1st")
        self.position_topic.publish(self.retract)
        # Main loop
        while self.playing:
            print("2d")
            while self.win == 0:
                print("3r")
                if self.turn == 2:
                    print("4d")
                    self.mov_to_make = dec(self.b_s)
                    print("5d")
                    self.move_make(self.b_cp[self.mov_to_make])
                    print("6t")
                    self.b_s[self.mov_to_make] = 2
                    print("7t")
                    if win(2, self.b_s):
                        self.win = 2

                    self.turn = 1
                elif self.turn == 1:
                    print(self.b_s)
                    self.b_s[int(input("What move was made"))] = 1
                    if win(1, self.b_s):
                        self.win = 1
                    self.turn = 2

            if self.win == 2:
                self.rob_celeb()

            elif self.win == 1:
                self.rob_rage()

            if int(input("Press 1 after reseting the board to play again")):
                self.b_s = [0, 0, 0, 0, 0, 0, 0, 0, 0]
                self.win = 0
                self.turn = int(input("Who go first ,AI =2 ,HUMAN = 1"))

    def move_to_position(self, x, y, z):
        newMsg = Pose()
        newMsg.position.x = x
        newMsg.position.y = y
        newMsg.position.z = z

        newMsg.orientation.x = math.pi
        newMsg.orientation.y = 0.0
        newMsg.orientation.z = 0.0
        self.position_topic.publish(newMsg)
        time.sleep(2)

    def move_make(self, msg):
        # Determine using ai where to place
        # pick up a block
        # move to the board position
        # drop it
        # go back to retract
#


        self.move_to_position(self.pp[0],self.pp[1],0.45)
        self.gripper_value.data = 0.2
        self.gripper_topic.publish(self.gripper_value)
        time.sleep(10)
        self.move_to_position(self.pp[0],self.pp[1],0.25)
        time.sleep(10)
        self.gripper_value.data = 0.8
        self.gripper_topic.publish(self.gripper_value)
        time.sleep(10)
        self.move_to_position(self.pp[0],self.pp[1],0.45)
        


        self.position_topic.publish(self.retract)
        time.sleep(5)
        self.move_to_position(msg[0], msg[1], 0.45)
        time.sleep(8)
        self.move_to_position(msg[0], msg[1], 0.25)
        time.sleep(3)
        self.gripper_value.data = 0.2
        self.gripper_topic.publish(self.gripper_value)
        time.sleep(5)
        self.move_to_position(msg[0], msg[1], 0.45)
        time.sleep(5)
        self.position_topic.publish(self.retract)       


    def rob_celeb(self):
        # idk have the robot do something when it wins ?
        # make it twerk or sthm :>
        print("HAHAHAHAHAHA THE AI WON U SUCH A NOOB")

    def rob_rage(self):
        # make the robot sweep the game peices off the board when it louses :)
        print("comon man ill win next time")


def main():
    rclpy.init()
    node = Game()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
