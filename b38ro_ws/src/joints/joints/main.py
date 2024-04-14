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
from .move import det_bord_cart
from .tttai import dec 
from .tttai import win
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

        #self.create_subscription(Joy, "/joy", self.controller_cb, 10)

        #self.coords = det_bord_cart()

        self.position_topic = self.create_publisher(Pose, "/cart_pose", 10)

        #store board state 0=empty ,2=ai taken ,1=player taken
        self.b_s = [0,0,0,0,0,0,0,0,0]
        #store current player and winner if any 
        self.win = 0

        #store board carteisan position and calculate cp of each point
        self.p1 = [1,1] #future implement method to 
        self.p2 = [0,0] #find positions and calibrate with cp ,do in move.py


        self.b_cp = det_bord_cart(self.p1,self.p2)     

        #define resting pose 
        self.retract = Pose()
        self.retract.position.x = 0.0
        self.retract.position.y = 0.0
        self.retract.position.z = 1.0

        #main game logic    
        
        #det if human / ai go first 
        self.turn = int(input('Who go first ,AI =0 ,HUMAN = 1'))
        self.playing = True
        # Main loop 
        while self.playing :

            while self.win == 0 :
                if self.turn == 2:
                    self.mov_to_make = dec(self.b_s)
                    self.move_made(self.b_cp[self.mov_to_make])
                    self.b_s[self.mov_to_make]=2
                    if win(2,self.b_s):
                        self.win = 2

                    self.turn = 1
                elif self.mov == 1 :
                    self.b_s[int(input("What move was made"))] = 1
                    if win(1,self.b_s):
                        self.win = 1
                    self.turn  = 2

            if self.win == 1 :
                self.rob_celeb()

            elif self.win == 2:
                self.rage()

            if int(input('Press 1 after reseting the board to play again')):
                self.b_s = [0,0,0,0,0,0,0,0,0]
                self.win = 0
                self.turn = int(input('Who go first ,AI =0 ,HUMAN = 1'))



        

    

    def move_to_position(self, x, y, z):
        newMsg = Pose()
        newMsg.position.x = x
        newMsg.position.y = y
        newMsg.position.z = z

        newMsg.orientation.x = math.pi
        newMsg.orientation.y = 0.0
        newMsg.orientation.z = 0.0
        self.position_topic.publish(newMsg)

    def move_made(self, msg):
        # Determine using ai where to place
        # pick up a block
        # move to the board position
        # drop it
        # go back to retract
        
        self.position_topic.publish(self.retract)
    
    def rob_celeb():
        # idk have the robot do something when it wins ?
        # make it twerk or sthm :>
        print('HAHAHAHAHAHA THE AI WON U SUCH A NOOB')
    
    def rob_rage():
        #make the robot sweep the game peices off the board when it louses :)
        print('comon man ill win next time')



def main():
    rclpy.init()
    node = Game()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()