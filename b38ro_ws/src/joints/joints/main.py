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
        self.gripper_value = Float32()

        self.board_state_pub = self.create_publisher(
            Int32MultiArray, "/board_state", 10
        )
        self.create_subscription(Int32, "/human_move", self.human_move_cb, 10)

        # restart copelia sim

        # self.start_sim = self.create_publisher(Bool, "/startSimulaion", 10)
        # self.stop_sim = self.create_publisher(Bool, "/stopSimulation", 10)
        # self.temT = Bool()
        # self.temT.data = False

        # print("tt")
        # self.stop_sim.publish(self.temT)
        # print("t")
        # time.sleep(2)
        # self.start_sim.publish(self.temT)
        # print("f")

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
        self.move_to_position([0.2, 0.2, 0.25], 10)
        self.gripper_topic.publish(self.gripper_open)
        self.move_to_position_split([0.2, 0.2, 0.14], 3, 5)
        self.gripper_topic.publish(self.gripper_closed)
        self.move_to_position_split([0.2, 0.2, 0.25], 3, 2)
        self.move_to_position([0.2, -0.2, 0.25], 3)
        self.move_to_position_split([0.2, -0.2, 0.14], 3, 5)
        self.gripper_topic.publish(self.gripper_open)

        # det if shuman / ai go first
        self.turn = int(input("Who will go first (AI = 2, HUMAN = 1) "))
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

    def move_to_position(self, tar, tots):
        newMsg = Pose()
        newMsg.position.x = tar[0]
        newMsg.position.y = tar[1]
        newMsg.position.z = tar[2]

        newMsg.orientation.x = math.pi
        newMsg.orientation.y = 0.0
        newMsg.orientation.z = math.pi / 2
        self.position_topic.publish(newMsg)
        time.sleep(tots)
        self.cur_pos = tar

    #     def bezier_curve(self,org, P1, xyz, t):
    #     xb = (1 - t) ** 2 * org[0] + 2 * (1 - t) * t * P1[0] + t ** 2 * xyz[0]
    #     yb = (1 - t) ** 2 * org[1] + 2 * (1 - t) * t * P1[1] + t ** 2 * xyz[1]
    #     zb = (1 - t) ** 2 * org[2] + 2 * (1 - t) * t * P1[2] + t ** 2 * xyz[2]
    #     return xb, yb, zb

    # def generate_curve_points(self,org, P1, xyz, step=0.1):
    #     points = []
    #     t = 0
    #     while t <= 1:
    #         points.append(self.bezier_curve(org, P1, xyz, t))
    #         t += step
    #     return points

    # def move_to_position_split(self,org ,x, y, z ,steps,tots):
    #     # initial_pose = [org[0], org[1], org[2]]
    #     # final_pose = [x, y, z]
    #     # intermediate_pose = [(org[0]+x)/2, (org[1]+y)/2, (org[2]+z)/2 + 0.6]
    #     points = self.generate_curve_points(org, [(org[0]+x)/2, (org[1]+y)/2, (org[2]+z)/2], [x, y, z])  # Generate curve points
    #     step_size = len(points) // steps  # Calculate step size
    #     for i in range(0, len(points), step_size):
    #         self.move_to_position(points[i][0], points[i][1], points[i][2], tots/steps)

    def move_to_position_split(self, tar, steps, tots):

        o2t_s = numpy.linspace(self.cur_pos, tar, steps)

        for x in o2t_s:
            self.move_to_position(x, tots / steps)

    def make_move(self, msg):
        # Determine using ai where to place
        # pick up a block
        # move to the board position
        # drop it
        # go back to retract

        # First - pickup block
        pp = [
            [-0.125, 0.4, 0.05],
            [0.125, 0.4, 0.05],
            [0.0, 0.4, 0.05],
            [0.125, -0.4, 0.05],
            [0.0, -0.4, 0.05],
            [-0.125, -0.4, 0.05],
        ]

        # steps to pickup block :
        # move above block
        # move down with gripper open
        # close gripper
        # move up

        # move above the block
        self.move_to_position(
            [pp[self.moves_numai][0], pp[self.moves_numai][1], 0.45], 15
        )

        # move down with gripper closed
        self.gripper_topic.publish(self.gripper_open)
        self.move_to_position_split(
            [pp[self.moves_numai][0], pp[self.moves_numai][1], 0.25], 10, 30
        )

        # close gripper
        self.gripper_topic.publish(self.gripper_closed)
        # move up
        self.move_to_position_split(
            [pp[self.moves_numai][0], pp[self.moves_numai][1], 0.45], 3, 7
        )
        self.moves_numai = self.moves_numai + 1
        # steps to move and drop block  :
        # move to centeral resting position
        # move above the dropping point
        # move down a bit --SKIP? yes skip!
        # drop / open gripper
        # move above the dropping point
        # move back to centeral

        # move to mid posiion
        self.move_to_position(
            [self.board_positions[4][0], self.board_positions[4][1], 0.5], 7
        )

        # move above dropping point
        self.move_to_position([msg[0], msg[1], 0.45], 5)

        # move down a bit
        self.move_to_position_split([msg[0], msg[1], 0.255], 20, 30)

        # open gripper
        self.gripper_topic.publish(self.gripper_open)
        time.sleep(2)

        # move up
        self.move_to_position_split([msg[0], msg[1], 0.45], 5, 10)

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
