#import ROS 
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import Pose




#import libs
import picplace.py as p
import tttai.py as t
import waitformove.py as w
import move.py as m
import firstmove.py as fm




#initalisation 
r.setup()
m.def_pos()#move robot arm to default position 
p_s=m.det_bord_cart()#calibrate the position of each sqaure 
b_s=[0,0,0,0,0,0,0,0,0]

# determine who goes first 
if (fm.h_first())
	md = w.wait()#wait for user to make move 
	#current implement also has user input move they made 
	b_s[md]=1
	#if we go with comp vision we'll have 3 nodes ,one for move detection ,one for board position and one for main logic 




	
	
	


