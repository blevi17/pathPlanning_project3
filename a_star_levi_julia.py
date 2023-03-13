# This is the main python file for Path Planning Project 3
# by Levi Butler and Julia MacOn Kim
# GitHub repository: https://github.com/blevi17/pathPlanning_project3

import numpy as np
from queue import PriorityQueue
import matplotlib.pyplot as plt
from matplotlib import animation
from sympy import *
import time

## used to time how long code takes to execute
start_time = time.time()

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
     
     new_no = [x_new, y_new, th_new]

     return new_no 

     
# This function converts the x, y, and theta list into the format needed for checking the tree (instructions: page 14, Second method) 
def mat_expl(node_exp):
    x_exp = round(2 * node_exp[0])-1 #subtract 1 to convert from coord (1-600) to index (0-599)
    y_exp = round(2 * node_exp[1])-1 #subtract 1 to convert from coord (1-250) to index (0-249)
    th_exp = round(node_exp[2]/30) #updated to use 30 degree threshold for theta
    if th_exp == 12: #added this to fix an issue so 360 and 0 are treated as the same angle when indexing check matrices. There might be a better way to fix it than this
        th_exp = 0

    return x_exp, y_exp, th_exp
         
         
############################################ Main code ###############################################

# I liked Julia's function for prompting the user for inputs, could go here

# I used Priority queue (I am surpised you did not use that or heap)
# Initialize priority q
open_l = PriorityQueue()
closed_l = PriorityQueue()
q3 = PriorityQueue()  # this is just for the while loop
uu = q3.empty()

# Create the first element in the list
node_i = []  # This needs to be changed to the customer input
# [Cost, index, parent, [x, y, theta]]
el1 = [0, 0, 0, node_i]

# check if the initial node or goal nodes are in obstacle space


# Starting the search
res_g = 0  # becomes 1 or something else when we reach the goal
id = el1[1]
mat_exp_ol = np.zeros((1200, 400, 361))  # empty matrix to record where we have explored in the open list
mat_exp_cl = np.zeros((1200, 400, 361))  # empty matrix to record where we have explored in the closed list
mat_cost = np.zeros((1200, 400, 361)) # saving the cost in a matrix
# may want to move mat_exp earlier as we can designate obstacle space as 0
while open_l.empty() != uu and res_g == 0:
     # pull out a node and add it to the closed list
     x = open_l.get()
     closed_l.put(x)
     # pull out useful elements
     cur_cost = x[0]
     cur_ind = x[1]
     cur_par = x[2]
     cur_pos = x[4]
    
     # check if we have reached the goal
     if cur_pos == node_g:  # Need to add the goal node input
          # run the backtrack function
          #node_path = trace_back(close_l, cur_par, cur_ind)
          print("success")
          res_g = 1
     # perform all possible movements
     else:
          for i in range(0, 5):
               new_pos = action_m(cur_pos, L, i)  # L is the user input for the step size of the robot
               # Check if the new location is in the closed list or obstacle space
               i_e, j_e, th_e = mat_expl(new_pos)
               check_cl = mat_exp_cl[i_e][j_e][th_e]
               check_ol = mat_exp_ol[i_e][j_e][th_e]
               #check_ob = p2_coll(new_l)
               # need to add lxu = cost2come + cost2go
               if check_cl == 0 and check_ob == 1:
                    mat_exp_cl[i_e][j_e][th_e] = 1  # now we have explored this in the closed list
                    if check_ol == 0:
                        mat_exp_ol[i_e][j_e][th_e] = 1
                        id = id + 1
                        # lxu is cost to come plus cost to go
                        new_node = [cur_cost + lxu, id, cur_ind, new_pos]
                        mat_cost[i_e][j_e][th_e] = cur_cost + lxu  #update the cost matrix
                        open_l.put(new_node)
               elif check_cl == 1 and check_ob == 1:
                    m_co = mat_cost[i_e][j_e][th_e]  # this is to get the cost of the repeat, needs to be remade
                    lenj = closed_l.qsize()
                    if m_co > (cur_cost + lxu):
                        for j1 in range(0, lenj):
                            if closed_l.queue[j1][3] == new_pos:
                                m_i = closed_l.queue[j1][1]
                                closed_l.queue[j1] = [cur_cost + lxu, m_i, cur_ind, new_pos]
                                mat_cost[i_e][j_e][th_e] = cur_cost + lxu  # update the cost matrix
                                
## used to time how long code takes to execute
end_time = time.time()
print('Total time (s):', end_time-start_time)