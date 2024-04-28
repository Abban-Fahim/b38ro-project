#testing equations ect to try and derrive method +equations to get cartesian positions given 
# two opposite points in board ,ie;
# x |   |     given the x'es cartesian positions (where x is where the robot would place)\
#   |   |     
#   |   | x   deduce each of the 9 places cartesian positions 


#  (0,0) | (1,0) | (2,0)
# 
# 

import math as m


def tpte(p1,p2):
    a = (p1[1]-p2[1])
    b = (p2[0]-p1[0])
    c = (((p2[1]-p1[1])*p2[0])-((p2[0]-p1[0])*p2[1]))
    
    return [a,b,c]

def tpd(p1,p2):
    return m.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
  
def pl(eq,p1):
    return [eq[1],-eq[0],(-eq[1]*p1[0]+eq[0]*p1[1])]

def sec(eq1,eq2):
    return [((eq1[1]*eq2[2]-eq1[2]*eq2[1])/(eq1[0]*eq2[1]-eq1[1]*eq2[0])),((eq1[2]*eq2[0]-eq1[0]*eq2[2])/(eq1[0]*eq2[1]-eq1[1]*eq2[0]))]

 
def lbl(eq1,p1,d):
    return [eq1[0],eq1[1],(d*m.sqrt(eq1[0]**2+eq1[1]**2)-eq1[0]*p1[0]-eq1[1]*p1[1])]
    
def ris(p1,p2):
    d = tpd(p1,p2)
    
    l1 = tpte(p1,p2)#
    l1p = pl(l1,p1)#tem
    lp1 = lbl(l1p,p1,-d/4)#
    lp2 = lbl(l1p,p1,-d/2)#
    lp3 = lbl(l1p,p1,-3*d/4)
    l2 = lbl(l1,p1,d/2)
    l3 = lbl(l1,p1,d/4)
    l4 = lbl(l1,p1,-d/2)
    l5 = lbl(l1,p1,-d/4)

    
    q0 = p1
    q1 = sec(lp1,l5)
    q2 = sec(lp2,l4)
    
    
    q3 = sec(lp1,l3)
    q4 = sec(l1,lp2)
    q5 = sec(lp3,l5)
    
    q6 = sec(lp2,l2)
    q7 = sec(lp3,l3)
    q8 = p2
    

    
    return [q0,q1,q2,q3,q4,q5,q6,q7,q8]
# derrived method :
# need to make 8 lines ,line joining the two points ,4 parralell and 3 perpendicular 
# the 4 parralell lines ,two to the left and 2 right .
# factor of 1/2 and 1/4 of distance bitween the two points left/right
# the 3 perpendicular lines ,one on each of the known points
# and one on the midpoint bitween them 

#check desmos graph for more detail https://www.desmos.com/calculator/lmcito3cnm


