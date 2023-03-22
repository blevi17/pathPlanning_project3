# This is the main python file for Path Planning Project 3 Phase 2
# It says to use the map from phase 1, but the turtlebot won't fit in that 
# by Levi Butler and Julia Macon Kim
# GitHub repository: https://github.com/blevi17/pathPlanning_project3

import numpy as np
from queue import PriorityQueue
import matplotlib.pyplot as plt
from matplotlib import animation
from sympy import *
import time

## used to time how long code takes to execute
start_time = time.time()

# Variables from the datasheet of the turtlebot
R = 66  # wheel radius in mm
r = 105 # necessary buffer around the robot in mm
L = 160  # wheel center to center distance in mm
v_mr = 2.84  # maximum rotational velocity in rad/s
v_mrev = float(v_mr * 60 / (2 * pi))  # maximum rotational velocity in revolutions/minute
v_mt = 220  # maximum translational velocity in mm/s according to the spec sheet
dt = 0.2  # Time step that we get to define  ###############################################################################

################################## Functions ###############################################
# define function to convert user input into node format
def input2node(input):
    output = []
    for num in input.split(', '):
        output.append(float(num))
    return output

def move_step(u_l, u_r, theta):
    dx1 = float((r/2) * (u_l + u_r) * cos(theta) * dt)
    dy1 = float((r/2) * (u_l + u_r) * sin(theta) * dt)
    dth1 = float((r/L) * (u_r + u_l) * dt)

    return dx1, dy1, dth1

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
            if q_cl.queue[it][2] == par:
                path.append(q_cl.queue[it][3])
                par = q_cl.queue[it][3]
    trace_res = reverse_list(path)

    return trace_res

################################### Main Code #############################################
# Taking in user input
while 1:
    try:
        c = float(input("Clearance:"))
        if L>190 or L<0:
            print('Clearance is too large to guarantee the goal is reachabel! Try Again...')
        else:
            break
    except:
        print('Input must be a number between 0 and 190. Try again...')

# Check the obstacle space
# I want to make the bottom left corner the obstacle space of the matrix we are checking
obs = np.zeros(6000, 2000)  # might have to change this depending on step sizes
for i in range(6000):
    # print('i',i)
    for j in range(2000):
        # print('j',j)
        if j<=(1250 + r + c) and (2395 - r - c)<=i<=(2650 + r + c): #bottom rectangle definition w/ robot radius and clearance
            obs[i,j]=1
        elif j>=(750 - r - c) and (1395 - r - c)<=i<=(1650 + r + c): #top rectangle definition w/ robot radius and clearance
            obs[i,j]=1
        elif (i - 4000)**2 + (j - 1000)**2 <= (500 + r + c)**2: #circle definition w/ margin
            obs[i,j]=1
        elif i<=(r+c) or i>=(6000 - r - c):  #vertical wall definition
            obs[i,j]=1
        elif i<=(r+c) or i>=(4000 - r - c):  #horizontal wall definition
            obs[i,j]=1

while 1:
    try:
        start_input = input("Start State:")
        node_i = input2node(start_input)
        if obs[int(node_i[0] + 500),int(2*node_i[1] + 1000)]==1:
            print('Start State inside an obstacle. Try again...')
        else:
            break
    except:
        print('Input must be three integers separated by a comma and space (ex: 10, 10, 30). Acceptable range for first value: -499 to 5500. Acceptable range for second value: -999 to 1000. Acceptable range for third value: 0 to 359. Try again...')
while 1:
    try:
        goal_input = input("Goal State:")
        node_g = input2node(goal_input)
        if obs[int(2*node_g[0]),int(2*node_g[1])]==1:
            print('Goal State inside an obstacle. Try again...')
        else:
            break
    except:
        print('Input must be two integers separated by a comma and space (ex: 10, 10). Acceptable range for first value: -499 to 5500. Acceptable range for second value: -999 to 1000. Try again...')
while 1:
    try:
        wheel_rpm = input("Wheel RPMs:")
        if 0 < wheel_rpm[0] < v_mrev and 0 < wheel_rpm[1] < v_mrev:
            print('Inputs accepted! Calculating...')
            break
        else:
            print('The wheel rpms are beyond the ability of the burger turtlebot. Try again...')
            print('Input must be two integers separated by a comma and space (ex: 10, 10). Acceptable range for wheel rpms is 0 to 27.12 rpm, non-inclusive.Try again...')
    except:
        print('Input must be two integers separated by a comma and space (ex: 10, 10). Acceptable range for wheel rpms is 0 to 27.12 rpm, non-inclusive.Try again...')

# Action Set
rpm1 = wheel_rpm[0]
rpm2 = wheel_rpm[1]
act = [[0, rpm1], [rpm1, 0], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]

# Initialize priority q
open_l = PriorityQueue()
closed_l = PriorityQueue()

# Create the first element in the list
cost_go = np.sqrt((node_g[0]-node_i[0])**2 + (node_g[1]-node_i[1])**2)
# Determine number of points per frame and the weighting of the cost to go
if cost_go < 250:
    c_w = 1.5
    v_w = 200
elif cost_go < 375:
    c_w = 1.5
    v_w = 250
else:
    c_w = 3.75
    v_w = 550
# [Total cost, cost to go (not based on goal location), index, parent, [x, y, theta]] #will be easier to compute below if we track cost to go for each node
el1 = [0, 0, 0, 0, node_i]
open_l.put(el1) # starting the open list

# Starting the search
id = el1[1]  # index of new nodes
mat_exp_ol = np.zeros((6007, 407))  # empty matrix to record where we have explored in the open list (we don't care about angle at the goal)
mat_exp_cl = np.zeros((6007, 407))  # empty matrix to record where we have explored in the closed list
mat_cost = np.zeros((6007, 407)) # saving the cost in a matrix
while not open_l.empty():
    # pull out a node and add it to the closed list
    x = open_l.get()
    closed_l.put(x)
    # pull out useful elements
    cur_cost = x[0]
    cur_go = x[1]
    cur_ind = x[2]
    cur_par = x[3]
    cur_pos = x[4]

    mat_exp_cl[x[0] - 1][x[1] - 1] = 1  # now we have explored this in the closed list
    # check if we have reached the goal
    thresh = np.sqrt((cur_pos[0] - node_g[0])**2 + (cur_pos[1] - node_g[1])**2)
    if thresh <= 1.5:
        # run the backtrack function
        node_path = trace_back(closed_l, cur_par, cur_ind)
        ## added to the print line below to verify that final state is correct
        print("Success! Confirm final state:",'('+str(cur_pos[0])+', '+str(cur_pos[1])+', '+str(cur_pos[2])+')')
        break
    
    else:
        for i in range(0,8):
            rpm = act(i)
            # no slip condition
            ul = float(rpm[0] * 2 * pi * R)
            ur = float(rpm[1] * 2 * pi * R)
            dx, dy, dth = move_step(ul, ur, cur_pos[2])
            new_pos = [cur_pos[0] + dx, cur_pos[1] + dy, cur_pos[2] + dth]

            # keep theta value between 0 and 359
            if new_pos[2]>359:
                new_pos[2] = new_pos[2]-360
            elif new_pos[2]<0:
                new_pos[2] = new_pos[2]+360
            # Check if the new location is in the closed list or obstacle space
            check_cl = mat_exp_cl[new_pos[0]][new_pos[1]]
            check_ol = mat_exp_ol[new_pos[0]][new_pos[1]]
            check_ob = obs[new_pos[0]][new_pos[1]] #no theta value for obstacles
            cg = float(np.sqrt(dx**2 + dy**2))
            cost_come = cur_go + cg
            cost_go = c_w * np.sqrt((node_g[0]-new_pos[0])**2 + (node_g[1]-new_pos[1])**2)  # weighted cost to go
            lxu = cost_come+cost_go  # combined cost

## used to time how long code takes to execute
end_time = time.time()
print('Total time (s):', end_time-start_time)
