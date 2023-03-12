# This is the main python file for Path Planning Project 3
# by Levi Butler and Julia MacOn Kim

import numpy as np
from queue import PriorityQueue
import matplotlib.pyplot as plt
from matplotlib import animation
from sympy import *

##################################### Functions #####################################
# pos_act = [x, y, theta], act is which one of the five actions
def action_m(pos_act, L_act, act):
    x_act = pos_act[0]
    y_act = pos_act[1]
    th_act = pos_act[2]
    # move forward
    if act == 1:
         ang = th_act * pi / 180
         x_add = L_act * cos(ang)
         y_add = L_act * sin(ang)
         x_new = float(x_act + x_add)
         y_new = float(y_act + y_add)
         th_new = th_act
    # move counter-clockwise 30 
    elif act == 2:
         ang = (th_act + 30) * pi / 180
         x_add = float(L_act * cos(ang))
         y_add = float(L_act * sin(ang))
         x_new = x_act + x_add
         y_new = y_act + y_add
         th_new = th_act + 30
    # move counter-clockwise 60 
    elif act == 3:
         ang = (th_act + 60) * pi / 180
         x_add = float(L_act * cos(ang))
         y_add = float(L_act * sin(ang))
         x_new = x_act + x_add
         y_new = y_act + y_add
         th_new = th_act + 60
    # move clockwise 30 
    elif act == 4:
         ang = (th_act - 30) * pi / 180
         x_add = float(L_act * cos(ang))
         y_add = float(L_act * sin(ang))
         x_new = x_act + x_add
         y_new = y_act + y_add
         th_new = th_act - 30
    # move clockwise 60 
    else:
         ang = (th_act - 60) * pi / 180
         x_add = float(L_act * cos(ang))
         y_add = float(L_act * sin(ang))
         x_new = x_act + x_add
         y_new = y_act + y_add
         th_new = th_act - 60
         
     ############################################ Main code ###############################################