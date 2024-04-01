#testing equations ect to try and derrive method +equations to get cartesian positions given 
# two opposite points in board ,ie;
# x |   |     given the x'es cartesian positions (where x is where the robot would place)\
#   |   |     
#   |   | x   deduce each of the 9 places cartesian positions 


#  (0,0) | (1,0) | (2,0)
# 
# 

import math as m


def TwoPointToEquation(p1,p2):
    a = (p1[1]-p2[1])
    b = (p2[0]-p1[0])
    c = (((p2[1]-p1[1])*p2[0])-((p2[0]-p1[0])*p2[1]))
    
    return [a,b,c]

def TwoPointDist(p1,p2):
    return m.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
  
def LinePerp(eq,p1):
    return [eq[1],-eq[0],(-eq[1]*p1[0]+eq[0]*p1[1])]

def Intersect(eq1,eq2):
    return [((eq1[1]*eq2[1]-eq1[2]*eq2[0])/(eq1[0]*eq2[1]-eq1[1]*eq2[0])),((eq1[2]*eq2[0]-eq1[0]*eq2[2])/(eq1[0]*eq2[1]-eq[1]*eq[0]))]

def PointbelowLine(eq1,p1,d):
    return [(p1[0]-eq1[1]/m.sqrt(eq1[0]**2+(eq1[1]**2)*d)),(p1[1]+eq1[0]/m.sqrt(eq1[0]**2+(eq1[1]**2)*d))]
    
    
    
# derrived method :
# need to make 8 lines ,line joining the two points ,4 parralell and 3 perpendicular 
# the 4 parralell lines ,two to the left and 2 right .
# factor of 1/2 and 1/4 of distance bitween the two points left/right
# the 3 perpendicular lines ,one on each of the known points
# and one on the midpoint bitween them 

#check desmos graph for more detail https://www.desmos.com/calculator/lmcito3cnm
