import rclpy
from std_msgs.msg import Int16MultiArray

def def_pos():
	#code to move to a default position for waiting for next move
	n = Int16MultiArray()
	n.data=[0,0,0,0,0,0}
	return n
	
def det_move(camfot):
	#code to either ;
	# analise camera footage for when a move is done ,and when its done 
	# or just by physical command (Read Readme in ROSMAIN)
