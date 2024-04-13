# imports
from .graphtests import ris


# functions

# def def_pos():
# 	#code to move to a default position for waiting for next move
# 	#

# def det_move(camfot):
# 	#code to either ;
# 	# analise camera footage for when a move is done ,and when its done
# 	# or just by physical command (Read Readme in ROSMAIN)


def det_bord_cart(c1cp,c2cp):
    # code that asks for user to calibrate board position
    c1cp[0] = input("carteasian pos of first pos x")
    c1cp[1] = input("carteasian pos of first pos y")
    c2cp[0] = input("cartesian positions of second pos x")
    c2cp[1] = input("cartesian positions of second pos y")
    
    # replace above with vision based in future 
    return ris(c1cp, c2cp)
