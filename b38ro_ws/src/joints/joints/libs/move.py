# imports
import graphtests as gra


# functions

# def def_pos():
# 	#code to move to a default position for waiting for next move
# 	#

# def det_move(camfot):
# 	#code to either ;
# 	# analise camera footage for when a move is done ,and when its done
# 	# or just by physical command (Read Readme in ROSMAIN)


def det_bord_cart():
    # code that asks for user to calibrate board position
    c1cp = [
        0.1,
        0.25,
    ]  # create a function to give control and get the cartesian position
    # of corner 1
    c2cp = [
        0.325,
        0.525,
    ]  # create a function to give control and get the cartesian position
    # of corner 2

    return gra.ris(c1cp, c2cp)
