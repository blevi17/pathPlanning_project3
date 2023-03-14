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
         
# define function to convert user input into node format
def input2node(input):
    output = []
    for num in input.split(', '):
        output.append(float(num))
    return output

# This reverses a list
def reverse_list(listio):
    new_list = []
    for i_l in range(1, len(listio) + 1):
        new_list.append(listio[-1*i_l])
    return new_list

# This searches through a queue, finding the parents of indexes until it reaches the initial node
def trace_back(q_cl, par, cur_ind):
    path = [cur_ind, par]
    while par != 0:
        len = q_cl.qsize()
        for it in range(0, len):
            if q_cl.queue[it][1] == par:
                path.append(q_cl.queue[it][2])
                par = q_cl.queue[it][2]
    trace_res = reverse_list(path)
  
############################################ Main code ###############################################

### I got it to run without errors but I think it is getting stuck in a loop without finding the goal node and I'm stuck trying to figure out where/why

# define obstacle space with 5mm clearance; 0=free space, 1=obstacle space
obs = np.zeros((1200,500))
for i in range(1200):
    # print('i',i)
    for j in range(500):
        # print('j',j)
        if j<=210 and 190<=i<=310: #bottom rectangle definition w/ margin
            obs[i,j]=1
        elif j>=290 and 190<=i<=310: #top rectangle definition w/ margin
            obs[i,j]=1
        elif i>=910 and j>=2*i-1812 and j<=-2*i+2312 and 40<=j<=460: #triangle definition w/ margin
            obs[i,j]=1
        elif 460<=i<=740 and j<=0.5774*i+65.5898 and j<=-0.5774*i+758.4102 and j>=-0.5774*i+434.4102 and j>=0.5774*i-258.4102: #hexagon definiton w/ margin
            obs[i,j]=1
        elif 0<=i<=9 or 1190<=i<=1199: #vertical wall margin
            obs[i,j]=1
        elif 0<=j<=9 or 490<=j<=499: #horizonal wall margin
            obs[i,j]=1

# I liked Julia's function for prompting the user for inputs, could go here
# Thanks! Updated input code below for new node format, and matched the variable names from your code below
while 1:
    try:
        start_input = input("Start State:")
        node_i = input2node(start_input)
        if obs[int(2*node_i[0]),int(2*node_i[1])]==1:
            print('Start State inside an obstacle. Try again...')
        else:
            break
    except:
        print('Input must be three integers separated by a comma and space (ex: 10, 10, 30). Acceptable range for first value: 1 to 600. Acceptable range for second value: 1 to 250. Acceptable range for third value: 0 to 359. Try again...')
while 1:
    try:
        goal_input = input("Goal State:")
        node_g = input2node(goal_input)
        if obs[int(2*node_g[0]),int(2*node_g[1])]==1:
            print('Goal State inside an obstacle. Try again...')
        else:
            break
    except:
        print('Input must be three integers separated by a comma and space (ex: 10, 10, 30). Acceptable range for first value: 1 to 600. Acceptable range for second value: 1 to 250. Acceptable range for third value: 0 to 359. Try again...')
while 1:
    try:
        L = float(input("Step Size:"))
        if 1<=L<=10:
            print('Inputs accepted! Calculating...')
            break
        else:
            print('Step size must be less than 10 and greater than 1. Try again...')
    except:
        print('Input must be a number between 1 and 10. Try again...')


# I used Priority queue (I am surpised you did not use that or heap)
# I will remove the comments above when I do a comment sweep for cleanliness
# Initialize priority q
open_l = PriorityQueue()
closed_l = PriorityQueue()
#q3 = PriorityQueue()  # this is just for the while loop
#uu = q3.empty()

# Create the first element in the list
#node_i = []  # This needs to be changed to the customer input #done
cost_go = np.sqrt((node_g[0]-node_i[0])**2 + (node_g[1]-node_i[1])**2)
# [Total cost, cost to go, index, parent, [x, y, theta]] #will be easier to compute below if we track cost to go for each node
el1 = [0, 0, 0, node_i]
open_l.put(el1) # starting the open list
# check if the initial node or goal nodes are in obstacle space

# Starting the search
res_g = 0  # becomes 1 or something else when we reach the goal
id = el1[1]
mat_exp_ol = np.zeros((1200, 500, 12))  # empty matrix to record where we have explored in the open list
mat_exp_cl = np.zeros((1200, 400, 12))  # empty matrix to record where we have explored in the closed list
mat_cost = np.zeros((1200, 500, 12)) # saving the cost in a matrix
# may want to move mat_exp earlier as we can designate obstacle space as 0
# I think the shape of mat_exp should be 1200, 500, 12 using the thresholds from slide 14
while not open_l.empty():
     # pull out a node and add it to the closed list
     x = open_l.get()
     closed_l.put(x)
     # pull out useful elements
     cur_cost = x[0]
     cost_go = x[1]
     cur_ind = x[2]
     cur_par = x[3]
     cur_pos = x[4]

     ## we should update closed record outside of the loop with node we are currently exploring
     i_e, j_e, th_e = mat_expl(cur_pos)
     # print(i_e, j_e, th_e) #used for a bug check
     mat_exp_cl[i_e][j_e][th_e] = 1  # now we have explored this in the closed list
    
     # check if we have reached the goal
     thresh = np.sqrt((cur_pos[0] - node_g[0])**2 + (cur_pos[1] - node_g[1])**2)
     if thresh <= 1.5:  
          # run the backtrack function
          node_path = trace_back(closed_l, cur_par, cur_ind)
          ## added to the print line below to verify that final state is correct
          print("Success! Confirm final state:",'('+str(cur_pos[0])+', '+str(cur_pos[1])+', '+str(cur_pos[2])+')')
          # res_g = 1
          break
     # perform all possible movements
     else:
          for i in range(0, 5):
               new_pos = action_m(cur_pos, L, i)  # L is the user input for the step size of the robot
               # keep theta value between 0 and 359
               if new_pos[2]>359:
                   new_pos[2] = new_pos[2]-360
               elif new_pos[2]<0:
                   new_pos[2] = new_pos[2]+360
               # Check if the new location is in the closed list or obstacle space
               i_e, j_e, th_e = mat_expl(new_pos)
               check_cl = mat_exp_cl[i_e][j_e][th_e]
               check_ol = mat_exp_ol[i_e][j_e][th_e]
               check_ob = obs[i_e][j_e] #no theta value for obstacles ##The obstacle matrix needs to be updated to match the new resolution
               new_pos_round = ((i_e+1)/2, (j_e+1)/2, (th_e)*30) ##added for now, so the if statement to update open list works. Need to continue troubleshooting why that check produces error if new_pos is not rounded to the threshold grid
               #check_ob = p2_coll(new_l)
               # need to add lxu = cost2come + cost2go
               cost_come = cur_cost - cost_go.copy() + L #.copy() is because I had a bug earlier with updating variables in a loop and using a copy fixed it
               cost_go = np.sqrt((node_g[0]-new_pos_round[0])**2 + (node_g[1]-new_pos_round[1])**2)
               lxu = cost_come+cost_go

               if check_ob == 1: #decoupled the checks into separate statements
                    continue #we can skip all the updating code if the new node is in the obstacle space
               elif check_cl == 0:
                    # mat_exp_cl[i_e][j_e][th_e] = 1  # now we have explored this in the closed list
                    if check_ol == 0:
                        mat_exp_ol[i_e][j_e][th_e] = 1
                        id = id + 1
                        # lxu is cost to come plus cost to go
                        new_node = [lxu, cost_go, id, cur_ind, new_pos]
                        mat_cost[i_e][j_e][th_e] = lxu  #update the cost matrix
                        open_l.put(new_node)
                    ## below if statement moved from check_cl==1 section below, and updated for open list
                    elif check_cl == 1:
                        m_co = mat_cost[i_e][j_e][th_e]  # this is to get the cost of the repeat, needs to be remade
                        #lenj = closed_l.qsize()
                        check_idx = [item[4] for item in open_l.queue]
                        idx=check_idx.index(new_pos_round) #this should eliminate need for a for loop
                        if m_co >  lxu:
                            rep_pos = open_l.queue[idx][4]
                            rep_ind = open_l.queue[idx][2]
                            rep_node = open_l.queue[idx]
                            open_l.queue.remove(rep_node)
                            imp_q = [lxu, cost_go, rep_ind, cur_ind, rep_pos]
                            open_l.put(imp_q)
                        #for j1 in range(0, lenj):
                         #   if closed_l.queue[j1][3] == new_pos:
                          #      m_i = closed_l.queue[j1][1]
                           #     closed_l.queue[j1] = [cur_cost + lxu, m_i, cur_ind, new_pos]
                            #    mat_cost[i_e][j_e][th_e] = cur_cost + lxu  # update the cost matrix
               elif check_cl == 1:
                   continue
                                
## used to time how long code takes to execute
end_time = time.time()
print('Total time (s):', end_time-start_time)
