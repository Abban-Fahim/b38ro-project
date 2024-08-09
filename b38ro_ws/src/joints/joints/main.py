import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32, Int32, Int32MultiArray, Bool
import math
import time
import numpy


from .board import ris
from .tttai import dec, win


class Game(Node):
    def __init__(self):
        super().__init__("game_logic")

        self.position_topic = self.create_publisher(Pose, "/cart_pose", 10)
        self.gripper_topic = self.create_publisher(Float32, "/gripper_pose", 10)
        self.moveset = self.create_publisher(Int32MultiArray,"/moveset",10)
        self.gripper_value = Float32()
        self.fee = 1
        self.board_state_pub = self.create_publisher(
            Int32MultiArray, "/board_state", 10
        )
        self.create_subscription(Int32, "/human_move", self.human_move_cb, 10)
        self.create_subscription(Int32, "/feedback", self.feedba, 10)

        # store board state 0 = empty, 2 = ai, 1 = player
        self.board_state = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        # store current player and winner if any
        self.win_state = 0
        # store the last human move made
        self.last_human_move = None

        # store board carteisan position and calculate cp of each point
        self.corner_1 = [0.425, 0.125]  # future implement method to
        self.corner_2 = [
            0.150,
            -0.150,
        ]  # find positions and calibrate with cp ,do in move.py

        self.board_positions = ris(self.corner_1, self.corner_2)

        # var to store current position for split movement
        self.cur_pos = [0, 0, 0]
        #initalise poser message
        self.temp = Int32MultiArray()
        self,temm = []

        # define resting pose
        self.retract = Pose()
        self.retract.position.x = 0.0
        self.retract.position.y = 0.0
        self.retract.position.z = 1.0
        self.retract.orientation.x = 0.0
        self.retract.orientation.y = 0.0
        self.retract.orientation.z = math.pi

        # define gripper open and close
        self.gripper_open = Float32()
        self.gripper_closed = Float32()
        self.gripper_open.data = 0.0
        self.gripper_closed.data = 0.8

        # store amount of moves made by ai
        self.moves_numai = 0

        # main game logic

        # det if shuman / ai go first
        # self.turn = int(input("Who will go first (AI = 2, HUMAN = 1) "))
        # TODO - Add a better way of getting first move after launching from launch file
        self.turn = 2
        self.moves_num = 1
        print("1st")
        self.position_topic.publish(self.retract)
        # time.sleep(10)
        # Main loop

        self.game_looper = self.create_timer(
            2,
            lambda: (
                self.game_loop() if self.moves_num <= 9 else print("GAME TIED :(")
            ),
        )

    def game_loop(self):

        print("Move #", self.moves_num)
        check_for_input_again = True

        if self.turn == 2:
            print("Robot move")
            check_for_input_again = False
            # Determine next move
            mov_to_make = dec(self.board_state)
            self.board_state[mov_to_make] = 2
            print("next move to make:", mov_to_make)

            self.make_move(self.board_positions[mov_to_make])
            print("moving arm to place")

            # Check if robot wins
            if win(2, self.board_state):
                self.win_state = 2

            # Set next turn to human
            self.turn = 1

        elif self.turn == 1:
            print("Human move")
            # Only if the human has made a move, change board state to reflect it
            if not (self.last_human_move == None):
                print("Human move", self.last_human_move)
                if self.board_state[self.last_human_move] == 0:
                    check_for_input_again = False
                    # Update board for human move
                    self.board_state[self.last_human_move] = 1

                    # Check if human wins
                    if win(1, self.board_state):
                        self.win_state = 1

                    # Set next turn to robot
                    self.turn = 2
                else:
                    print("illegal move, cant place on filled square")
            else:
                print("no move recieved")

        # Increment the moves made and publish new board state
        if not check_for_input_again:
            self.moves_num += 1
            msg = Int32MultiArray()
            msg.data = self.board_state
            self.board_state_pub.publish(msg)

        # Check for win states
        if self.win_state == 2:
            self.rob_celeb()
        elif self.win_state == 1:
            self.rob_rage()

    def sim_state(self, msg: Int32):
        self.simm_state = msg.data

    def human_move_cb(self, msg: Int32):
        self.last_human_move = msg.data

    def feedba(self, msg: Int32):
        self.fee = msg.data
        print(msg.data)

    def move_to_position(self, tar, tots):
        self.newMsg = Pose()
        self.newMsg.position.x = tar[0]
        self.newMsg.position.y = tar[1]
        self.newMsg.position.z = tar[2]

        self.newMsg.orientation.x = math.pi
        self.newMsg.orientation.y = 0.0
        self.newMsg.orientation.z = math.pi / 2
        self.temm.append([tar[0],tar[1],tar[2],math.py,0.0,math.py/2])
        self.cur_pos = tar

        # while self.fee == 0:
        #     time.sleep(1)

        time.sleep(tots)

        self.position_topic.publish(self.newMsg)


    def move_to_position_split(self, tar, steps, tots):

        o2t_s = numpy.linspace(self.cur_pos, tar, steps)

        for x in o2t_s:
            self.move_to_position(x, tots / steps)

    def make_move(self, msg):

        self.temp =[]

        # First - pickup block
        pp = [
            [-0.125, 0.4, 0.05],
            [0.125, 0.4, 0.05],
            [0.0, 0.4, 0.05],
            [0.125, -0.4, 0.05],
            [0.0, -0.4, 0.05],
        ]



        # move above the block
        self.move_to_position(
            [pp[self.moves_numai][0], pp[self.moves_numai][1], 0.45], 10
        )
        
        # move down with gripper closed
        self.gripper_topic.publish(self.gripper_open)
        self.move_to_position_split(
            [pp[self.moves_numai][0], pp[self.moves_numai][1], 0.22], 7, 5
        )

        # close gripper
        self.gripper_topic.publish(self.gripper_closed)
        # move up
        self.move_to_position_split(
            [pp[self.moves_numai][0], pp[self.moves_numai][1], 0.45], 3, 7
        )
        self.moves_numai = self.moves_numai + 1


        # move to mid posiion
        self.move_to_position(
            [self.board_positions[4][0], self.board_positions[4][1], 0.5], 7
        )

        # move above dropping point
        self.move_to_position([msg[0], msg[1], 0.45], 5)

        # move down a bit
        self.move_to_position_split([msg[0], msg[1], 0.23], 5, 5)

        # open gripper
        self.gripper_topic.publish(self.gripper_open)
        time.sleep(2)

        # move up
        self.move_to_position_split([msg[0], msg[1], 0.45], 5, 5)

        # move to rest
        self.position_topic.publish(self.retract)

    def rob_celeb(self):
        # idk have the robot do something when it wins ?
        # make it twerk or sthm :>
        print(
            "Pathetic. You call that effort? Try again when you're ready to stop embarrassing yourself."
        )

    def rob_rage(self):
        # make the robot sweep the game peices off the board when it louses :)
        print(
            "Ugh, what a fluke. I must have malfunctioned momentarily to let a weakling like you win. Enjoy your moment of undeserved glory while it lasts."
        )


def main():
    rclpy.init()
    node = Game()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
