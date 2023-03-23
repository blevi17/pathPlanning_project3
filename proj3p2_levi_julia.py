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
    dx1 = float((R/2) * (u_l + u_r) * cos(theta) * dt)
    dy1 = float((R/2) * (u_l + u_r) * sin(theta) * dt)
    dth1 = float((R/L) * (u_r + u_l) * dt)

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
obs = np.zeros((6000, 2000))  # might have to change this depending on step sizes
for i in range(6000):
    # print('i',i)
    for j in range(2000):
        # print('j',j)
        if j<=(1250 + r + c) and (2395 - r - c)<=i<=(2650 + r + c): #bottom rectangle definition w/ robot radius and clearance
            obs[i,j]=1
        elif j>=(750 - r - c) and (1395 - r - c)<=i<=(1650 + r + c): #top rectangle definition w/ robot radius and clearance
            obs[i,j]=1
        elif (i - 4000)**2 + (j - 1100)**2 <= (500 + r + c)**2: #circle definition w/ margin
            obs[i,j]=1
        elif i<=(r+c) or i>=(6000 - r - c):  #vertical wall definition
            obs[i,j]=1
        elif j<=(r+c) or j>=(2000 - r - c):  #horizontal wall definition
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
        if obs[int(node_g[0]+500),int(node_g[1]+1000)]==1:
            print('Goal State inside an obstacle. Try again...')
        else:
            break
    except:
        print('Input must be two integers separated by a comma and space (ex: 10, 10). Acceptable range for first value: -499 to 5500. Acceptable range for second value: -999 to 1000. Try again...')
while 1:
    try:
        w_rpm = input("Wheel RPMs:")
        wheel_rpm = input2node(w_rpm)
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
act = np.array([[0, rpm1], [rpm1, 0], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]])

# Initialize priority q
open_l = PriorityQueue()
closed_l = PriorityQueue()

# Create the first element in the list
cost_go = np.sqrt((node_g[0]-node_i[0])**2 + (node_g[1]-node_i[1])**2)
# Determine number of points per frame and the weighting of the cost to go
if cost_go < 250:
    c_w = 1.5
    v_w = 1
    #v_w = 200
elif cost_go < 375:
    c_w = 1.5
    v_w = 1
    #v_w = 250
else:
    c_w = 3.75
    v_w = 1
    #v_w = 550
# [Total cost, cost to go (not based on goal location), index, parent, [x, y, theta], ditance traveled to reach the point] #will be easier to compute below if we track cost to go for each node
el1 = [0, 0, 0, 0, node_i, 0]
open_l.put(el1) # starting the open list

# Starting the search
id = el1[1]  # index of new nodes
mat_exp_ol = np.zeros((6007, 2007))  # empty matrix to record where we have explored in the open list (we don't care about angle at the goal)
mat_exp_cl = np.zeros((6007, 2007))  # empty matrix to record where we have explored in the closed list
mat_cost = np.zeros((6007, 2007)) # saving the cost in a matrix
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

    mat_exp_cl[round(cur_pos[0])+500][round(cur_pos[1])+1000] = 1  # now we have explored this in the closed list
    # check if we have reached the goal
    thresh = np.sqrt((cur_pos[0] - node_g[0])**2 + (cur_pos[1] - node_g[1])**2)
    if thresh <= 30:
        # run the backtrack function
        node_path = trace_back(closed_l, cur_par, cur_ind)
        ## added to the print line below to verify that final state is correct
        print("Success! Confirm final state:",'('+str(cur_pos[0])+', '+str(cur_pos[1])+', '+str(cur_pos[2])+')')
        break
    
    else:
        for i in range(0,8):
            rpm = act[i, :]
            # no slip condition
            ul = float(rpm[0] * pi / 30)
            ur = float(rpm[1] * pi / 30)
            dx, dy, dth = move_step(ul, ur, cur_pos[2])
            L = float(np.sqrt(dx**2 + dy**2))
            new_pos = [cur_pos[0] + dx, cur_pos[1] + dy, cur_pos[2] + dth]

            # keep theta value between 0 and 359
            if new_pos[2]>359:
                new_pos[2] = new_pos[2]-360
            elif new_pos[2]<0:
                new_pos[2] = new_pos[2]+360
            # Check if the new location is in the closed list or obstacle space
            check_cl = mat_exp_cl[round(new_pos[0])+500][round(new_pos[1])+1000]
            check_ol = mat_exp_ol[round(new_pos[0])+500][round(new_pos[1])+1000]
            check_ob = obs[round(new_pos[0])+500][round(new_pos[1])+1000] #no theta value for obstacles
            cg = float(np.sqrt(dx**2 + dy**2))
            cost_come = cur_go + cg
            cost_go = c_w * np.sqrt((node_g[0]-new_pos[0])**2 + (node_g[1]-new_pos[1])**2)  # weighted cost to go
            lxu = cost_come+cost_go  # combined cost

            if check_ob == 1: #decoupled the checks into separate statements
                continue #we can skip all the updating code if the new node is in the obstacle space
            elif check_cl == 0:
                if check_ol == 0:
                    mat_exp_ol[round(new_pos[0])+500][round(new_pos[1])+1000] = 1
                    id = id + 1
                    # lxu is cost to come plus cost to go
                    new_node = [lxu, cost_come, id, cur_ind, new_pos, L]
                    mat_cost[round(new_pos[0])+500][round(new_pos[1])+1000] = lxu  #update the cost matrix
                    open_l.put(new_node)
                # Finding a repeat in the open loop
                elif check_ol == 1:
                    m_co = mat_cost[round(new_pos[0])+500][round(new_pos[1])+1000]
                    # If the cost of the old node is larger
                    if m_co >  lxu:
                        for j in range(0, open_l.qsize()):
                            check_pos = open_l.queue[j][4]
                            i_c = check_pos[0]
                            j_c = check_pos[1]
                            if i_c == (round(new_pos[0]+500)) and j_c == (round(new_pos[1]+1000)):
                                idx = j
                                rep_pos = open_l.queue[idx][4]
                                rep_ind = open_l.queue[idx][2]
                                rep_node = open_l.queue[idx]
                                open_l.queue.remove(rep_node)
                                imp_q = [lxu, cost_come, rep_ind, cur_ind, rep_pos, L]
                                open_l.put(imp_q)
            elif check_cl == 1:
                continue

# get points in the obstacle space
x_obs = []
y_obs = []
for i in range(-499, 5500, 4):
    for j in range(-999, 1000, 4):
        if obs[i, j] == 1:
            x_obs.append(i+500)
            y_obs.append(j+1000)

# save closed explored x and y points
plt1_size = closed_l.qsize()
x_exp1 = []
y_exp1 = []
xth_exp1 = []
yth_exp1 = []
for i1_plot in range(0, plt1_size):
    L = closed_l.queue[i1_plot][5]
    x_exp1.append(closed_l.queue[i1_plot][4][0])
    y_exp1.append(closed_l.queue[i1_plot][4][1])
    xth_exp1.append(float(L * cos(closed_l.queue[i1_plot][4][2] * pi / 180)))
    yth_exp1.append(float(L * sin(closed_l.queue[i1_plot][4][2] * pi / 180)))
x_exp1 = reverse_list(x_exp1)
y_exp1 = reverse_list(y_exp1)
xth_exp1 = reverse_list(xth_exp1)
yth_exp1 = reverse_list(yth_exp1)

# record the path
len_pa = len(node_path)
x_pa = []
y_pa = []
xth_pa = []
yth_pa = []
for i_pa in range(0, len_pa):
    ind_pa = node_path[i_pa]
    for j_pa in range(0, plt1_size):
        if closed_l.queue[j_pa][2] == ind_pa:
            L = closed_l.queue[j_pa][5]
            x_pa.append(closed_l.queue[j_pa][4][0])
            y_pa.append(closed_l.queue[j_pa][4][1])
            xth_pa.append(float(L * cos(closed_l.queue[j_pa][4][2] * pi / 180)))
            yth_pa.append(float(L * sin(closed_l.queue[j_pa][4][2] * pi / 180)))

#plotting the obstacle space
fig, ax = plt.subplots(figsize=(12, 7))
plt.plot(x_obs, y_obs, 'b.', markersize=1)
plt.xlim((-500, 5500))
plt.ylim((-1000, 1000))

# plot all of the explored points in a test
len_cl = len(x_exp1)
len_pa = len(x_pa)
print(len_cl)
print(len_pa)
def animate(fr):
    i_a = fr * v_w # range of points displayed in each frame
    if i_a < len_cl:
        ax.quiver(x_exp1[0:i_a], y_exp1[0:i_a], xth_exp1[0:i_a], yth_exp1[0:i_a], color="red", angles='xy', scale_units='xy', scale=1, width=0.005)
    elif i_a < (len_cl + len_pa):
        i_b = i_a - len_cl
        ax.quiver(x_pa[0:i_b], y_pa[0:i_b], xth_pa[0:i_b], yth_pa[0:i_b], color="green", angles='xy', scale_units='xy', scale=1, width=0.005)
    else:
        i_c = len_cl + len_pa
        ax.quiver(x_pa[0:i_c], y_pa[0:i_c], xth_pa[0:i_c], yth_pa[0:i_c], color="green", angles='xy', scale_units='xy', scale=1, width=0.005)

anim = animation.FuncAnimation(fig, animate,frames=(len_cl + len_pa), interval=1)

plt.show()

## used to time how long code takes to execute
end_time = time.time()
print('Total time (s):', end_time-start_time)
