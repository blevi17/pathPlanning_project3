# This is the main python file for Path Planning Project 3
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
            if q_cl.queue[it][2] == par:
                path.append(q_cl.queue[it][3])
                par = q_cl.queue[it][3]
    trace_res = reverse_list(path)

    return trace_res

############################################ Main code ###############################################

### I got it to run without errors but I think it is getting stuck in a loop without finding the goal node and I'm stuck trying to figure out where/why

# define obstacle space with 5mm clearance; 0=free space, 1=obstacle space
obs = np.zeros((1207,507))
for i in range(1207):
    # print('i',i)
    for j in range(507):
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
el1 = [0, 0, 0, 0, node_i]
open_l.put(el1) # starting the open list
# check if the initial node or goal nodes are in obstacle space

# Starting the search
res_g = 0  # becomes 1 or something else when we reach the goal
id = el1[1]
mat_exp_ol = np.zeros((1207, 507, 12))  # empty matrix to record where we have explored in the open list
mat_exp_cl = np.zeros((1207, 507, 12))  # empty matrix to record where we have explored in the closed list
mat_cost = np.zeros((1207, 507, 12)) # saving the cost in a matrix
# may want to move mat_exp earlier as we can designate obstacle space as 0
# I think the shape of mat_exp should be 1200, 500, 12 using the thresholds from slide 14
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

    ## we should update closed record outside of the loop with node we are currently exploring
    i_e, j_e, th_e = mat_expl(cur_pos)
    # print(i_e, j_e, th_e) #used for a bug check
    mat_exp_cl[i_e][j_e][th_e] = 1  # now we have explored this in the closed list

    # check if we have reached the goal
    thresh = np.sqrt((cur_pos[0] - node_g[0])**2 + (cur_pos[1] - node_g[1])**2)
    if thresh <= 1.5:
        if cur_pos[2] == node_g[2]:  
            # run the backtrack function
            node_path = trace_back(closed_l, cur_par, cur_ind)
            ## added to the print line below to verify that final state is correct
            print("Success! Confirm final state:",'('+str(cur_pos[0])+', '+str(cur_pos[1])+', '+str(cur_pos[2])+')')
            break
        else:
            while cur_pos[2] != node_g[2]:
                if ((cur_pos[2] - node_g[2])**2 % 3600) == 0:
                    first_act = 4
                else:
                    first_act = 2
                # Move forward once or 30 degrees
                m1 = action_m(cur_pos, L, 4)
                i_e, j_e, th_e = mat_expl(m1)
                check_ob = obs[i_e][j_e]
                if check_ob == 0:
                    id = id + 1
                    m_add = [0, 0, id, cur_par, m1]
                    closed_l.put(m_add)
                else:
                    print("Near Success (not enough room to correct orientation)! Confirm final state:",'('+str(cur_pos[0])+', '+str(cur_pos[1])+', '+str(cur_pos[2])+')')
                    break
                # Move 60 degrees to the right 5 times
                # First turn
                m2 = action_m(m1, L, 5)
                i_e, j_e, th_e = mat_expl(m2)
                check_ob = obs[i_e][j_e]
                if check_ob == 0:
                    id = id + 1
                    m_add = [0, 0, id, id - 1, m2]
                    closed_l.put(m_add)
                else:
                    print("Near Success (not enough room to correct orientation)! Confirm final state:",'('+str(cur_pos[0])+', '+str(cur_pos[1])+', '+str(cur_pos[2])+')')
                    break
                # Second turn
                m3 = action_m(m2, L, 5)
                i_e, j_e, th_e = mat_expl(m3)
                check_ob = obs[i_e][j_e]
                if check_ob == 0:
                    id = id + 1
                    m_add = [0, 0, id, id - 1, m3]
                    closed_l.put(m_add)
                else:
                    print("Near Success (not enough room to correct orientation)! Confirm final state:",'('+str(cur_pos[0])+', '+str(cur_pos[1])+', '+str(cur_pos[2])+')')
                    break
                # Third turn
                m4 = action_m(m3, L, 5)
                i_e, j_e, th_e = mat_expl(m4)
                check_ob = obs[i_e][j_e]
                if check_ob == 0:
                    id = id + 1
                    m_add = [0, 0, id, id - 1, m4]
                    closed_l.put(m_add)
                else:
                    print("Near Success (not enough room to correct orientation)! Confirm final state:",'('+str(cur_pos[0])+', '+str(cur_pos[1])+', '+str(cur_pos[2])+')')
                    break
                # Fourth turn
                m5 = action_m(m4, L, 5)
                i_e, j_e, th_e = mat_expl(m5)
                check_ob = obs[i_e][j_e]
                if check_ob == 0:
                    id = id + 1
                    m_add = [0, 0, id, id - 1, m5]
                    closed_l.put(m_add)
                else:
                    print("Near Success (not enough room to correct orientation)! Confirm final state:",'('+str(cur_pos[0])+', '+str(cur_pos[1])+', '+str(cur_pos[2])+')')
                    break
                # Fifth turn
                m6 = action_m(m5, L, 5)
                i_e, j_e, th_e = mat_expl(m6)
                check_ob = obs[i_e][j_e]
                if check_ob == 0:
                    id = id + 1
                    m_add = [0, 0, id, id - 1, m6]
                    closed_l.put(m_add)
                else:
                    print("Near Success (not enough room to correct orientation)! Confirm final state:",'('+str(cur_pos[0])+', '+str(cur_pos[1])+', '+str(cur_pos[2])+')')
                    break
                # Fix the angle after returning to the goal location
                if m6[2] > 359:
                    m6[2] = m6[2] - 360
                elif m6[2] < 0:
                    m6[2] = m6[2] + 360
                # update useful elements
                cur_cost = x[0] + 6 * L
                #cur_go = x[1]
                cur_ind = id
                cur_par = id - 1
                cur_pos = m6
            ## added to the print line below to verify that final state is correct
            node_path = trace_back(closed_l, cur_par, cur_ind)
            print("Success! Confirm final state:",'('+str(cur_pos[0])+', '+str(cur_pos[1])+', '+str(cur_pos[2])+')')
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
            #new_pos_round = ((i_e+1)/2, (j_e+1)/2, (th_e)*30) ##added for now, so the if statement to update open list works. Need to continue troubleshooting why that check produces error if new_pos is not rounded to the threshold grid
            #check_ob = p2_coll(new_l)
            # need to add lxu = cost2come + cost2go
            cost_come = cur_go + L #.copy() is because I had a bug earlier with updating variables in a loop and using a copy fixed it
            cost_go = 3.75 * np.sqrt((node_g[0]-cur_pos[0])**2 + (node_g[1]-cur_pos[1])**2)
            lxu = cost_come+cost_go

            if check_ob == 1: #decoupled the checks into separate statements
                continue #we can skip all the updating code if the new node is in the obstacle space
            elif check_cl == 0:
                # mat_exp_cl[i_e][j_e][th_e] = 1  # now we have explored this in the closed list
                if check_ol == 0:
                    mat_exp_ol[i_e][j_e][th_e] = 1
                    id = id + 1
                    # lxu is cost to come plus cost to go
                    new_node = [lxu, cost_come, id, cur_ind, new_pos]
                    mat_cost[i_e][j_e][th_e] = lxu  #update the cost matrix
                    open_l.put(new_node)
                ## below if statement moved from check_cl==1 section below, and updated for open list
                elif check_ol == 1:
                    m_co = mat_cost[i_e][j_e][th_e]  # this is to get the cost of the repeat, needs to be remade
                    #lenj = closed_l.qsize()
                    #check_idx = [item[4] for item in open_l.queue]
                    #idx=check_idx.index(new_pos_round) #this should eliminate need for a for loop
                    if m_co >  lxu:
                        # I added a for loop as I got errors from the code commented out above
                        for j in range(0, open_l.qsize()):
                            check_pos = open_l.queue[j][4]
                            i_c, j_c, th_c = mat_expl(check_pos)
                            if i_c == i_e and j_c == j_e and th_c == th_e:
                                idx = j
                        rep_pos = open_l.queue[idx][4]
                        rep_ind = open_l.queue[idx][2]
                        rep_node = open_l.queue[idx]
                        open_l.queue.remove(rep_node)
                        imp_q = [lxu, cost_come, rep_ind, cur_ind, rep_pos]
                        open_l.put(imp_q)
                    #for j1 in range(0, lenj):
                        #   if closed_l.queue[j1][3] == new_pos:
                        #      m_i = closed_l.queue[j1][1]
                        #     closed_l.queue[j1] = [cur_cost + lxu, m_i, cur_ind, new_pos]
                        #    mat_cost[i_e][j_e][th_e] = cur_cost + lxu  # update the cost matrix
            elif check_cl == 1:
                continue
                                
# get points in the obstacle space
x_obs = []
y_obs = []
for i in range(600):
    for j in range(250):
        xi = 2 * i - 1
        yi = 2 * j - 1
        if obs[xi, yi] == 1:
            x_obs.append(i)
            y_obs.append(j)

# save closed explored x and y points
plt1_size = closed_l.qsize()
x_exp1 = []
y_exp1 = []
xth_exp1 = []
yth_exp1 = []
for i1_plot in range(0, plt1_size):
    x_exp1.append(closed_l.queue[i1_plot][4][0])
    y_exp1.append(closed_l.queue[i1_plot][4][1])
    xth_exp1.append(float(L * cos(closed_l.queue[i1_plot][4][2] * pi / 180)))
    yth_exp1.append(float(L * sin(closed_l.queue[i1_plot][4][2] * pi / 180)))
x_exp1 = reverse_list(x_exp1)
y_exp1 = reverse_list(y_exp1)
xth_exp1 = reverse_list(xth_exp1)
yth_exp1 = reverse_list(yth_exp1)

# save the open explored points
#plt2_size = open_l.qsize()
#x_exp2 = []
#y_exp2 = []
#xth_exp2 = []
#yth_exp2 = []
#for i2_plot in range(0, plt2_size):
#    x_exp2.append(open_l.queue[i2_plot][4][0])
#    y_exp2.append(open_l.queue[i2_plot][4][1])
#    xth_exp2.append(float(L * cos(open_l.queue[i2_plot][4][2] * pi / 180)))
#    yth_exp2.append(float(L * sin(open_l.queue[i2_plot][4][2] * pi / 180)))

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
            x_pa.append(closed_l.queue[j_pa][4][0])
            y_pa.append(closed_l.queue[j_pa][4][1])
            xth_pa.append(float(L * cos(closed_l.queue[j_pa][4][2] * pi / 180)))
            yth_pa.append(float(L * sin(closed_l.queue[j_pa][4][2] * pi / 180)))

#plotting the obstacle space
fig, ax = plt.subplots(figsize=(12, 7))
#fig = plt.figure()
plt.plot(x_obs, y_obs, 'b.', markersize=1)
plt.xlim((0, 600))
plt.ylim((0, 250))

# plot all of the explored points in a test
len_cl = len(x_exp1)
len_pa = len(x_pa)
def animate(fr):
    i_a = fr * 500
    if i_a < len_cl:
        ax.quiver(x_exp1[0:i_a], y_exp1[0:i_a], xth_exp1[0:i_a], yth_exp1[0:i_a], color="red", angles='xy', scale_units='xy', scale=1, width=0.005)
    elif i_a < (len_cl + len_pa):
        i_b = i_a - len_cl
        ax.quiver(x_pa[0:i_b], y_pa[0:i_b], xth_pa[0:i_b], yth_pa[0:i_b], color="blue", angles='xy', scale_units='xy', scale=1, width=0.005)
    else:
        i_c = len_cl + len_pa
        ax.quiver(x_pa[0:i_c], y_pa[0:i_c], xth_pa[0:i_c], yth_pa[0:i_c], color="blue", angles='xy', scale_units='xy', scale=1, width=0.005)

anim = animation.FuncAnimation(fig, animate,frames=(len_cl + len_pa), interval=1)

plt.show()

## used to time how long code takes to execute
end_time = time.time()
print('Total time (s):', end_time-start_time)