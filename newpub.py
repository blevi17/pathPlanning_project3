import numpy as np
from queue import PriorityQueue
import rospy
from geometry_msgs.msg import Twist, Pose
import math
import time

## used to time how long code takes to execute
start_time = time.time()

# Variables from the datasheet of the turtlebot
# R = 33  # wheel radius in mm
b = 105 # necessary buffer around the robot in mm
# L = 160  # wheel center to center distance in mm
v_mr = 2.84  # maximum rotational velocity in rad/s
v_mrev = float(v_mr * 60 / (2 * math.pi))  # maximum rotational velocity in revolutions/minute
v_mt = 220  # maximum translational velocity in mm/s according to the spec sheet
# dt = 1  # Time step that we get to define  ###############################################################################

################################## Functions ###############################################
# define function to convert user input into node format
def input2node(input):
    output = []
    for num in input.split(', '):
        output.append(float(num))
    return output

# def move_step(u_l, u_r, theta):
#     dx1 = (R/2) * (u_l + u_r) * math.cos(theta) * dt
#     dy1 = (R/2) * (u_l + u_r) * math.sin(theta) * dt
#     dth1 = (R/L) * (u_r - u_l) * dt
#     dth1_deg = float(dth1 * 180 / pi)

#     return dx1, dy1, dth1_deg

def cost(Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 33
    L = 160
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = math.pi * Thetai / 180
    Thetai = math.pi * Thetai / 180
    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes
    D=0
    Dth=0
    # Dth_list = []
    # D_list = []
    while t<1:
        t = t + dt
        # Xs = Xn
        # Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
        Dth=Dth+ (Thetan-Thetai)
        # Dth_list.append(Dth)
        # D_list.append(D)
    Thetan = 180 * (Thetan) / math.pi
    # Dth = 180 * (Dth) / pi
    return Xn, Yn, Thetan, D, Dth

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

# Creating a publisher for the initial pose
#def start_pose(x1, y1, theta1):
#    msg1 = Pose()
#    pub1 = rospy.Publisher('cmd_vel', Pose, queue_size=10)
#    rospy.init_node('start_pose', anonymous=True)
#    for i in range(5):
#        if not rospy.is_shutdown():
#        # if not rospy.is_shutdown():
#            msg1.position.x = float(x1)
#            msg1.position.y = float(y1)
#            msg1.position.z = 0
#            msg1.orientation.x = 0
#            msg1.orientation.y = 0
#            msg1.orientation.z = 1
#            msg1.orientation.w = float(theta1 * math.pi / 180)
#            #buffer is based on the dt value
#            pub1.publish(msg1)
#            rospy.sleep(dt)


################################### Main Code #############################################
# Taking in user input
while 1:
    try:
        c = float(input("Clearance (mm):"))
        if c>190 or c<0:
            print('Clearance is too large to guarantee the goal is reachable! Must be less than 190. Try Again...')
        else:
            break
    except:
        print('Input must be a number between 0 and 190. Try again...')

# Check the obstacle space
# I want to make the bottom left corner the obstacle space of the matrix we are checking
## For part 1 we are supposed to use the same map as we did in phase 1 (see 2nd bullet on slide 15 of instructions) not the Gazebo map provided for part 2
obs = np.zeros((600, 200))  # might have to change this depending on step sizes
for i in range(600):
    # print('i',i)
    for j in range(200):
        # print('j',j)
        if j<=(1250 + b + c)/10 and (2500 - b - c)/10<=i<=(2650 + b + c)/10: #bottom rectangle definition w/ robot radius and clearance
            obs[i,j]=1
        elif j>=(750 - b - c)/10 and (1500 - b - c)/10<=i<=(1650 + b + c)/10: #top rectangle definition w/ robot radius and clearance
            obs[i,j]=1
        elif (i - 400)**2 + (j - 110)**2 <= ((500 + b + c)/10)**2: #circle definition w/ margin
            obs[i,j]=1
        elif i<=(b+c)/10 or i>=(6000 - b - c)/10:  #vertical wall definition
            obs[i,j]=1
        elif j<=(b+c)/10 or j>=(2000 - b - c)/10:  #horizontal wall definition
            obs[i,j]=1

## how are you getting the acceptable ranges below? Should it be x is 0 to 600 and y is 0 to 250 like before?
while 1:
    try:
        start_input = input("Start State (mm):")
        node_i = input2node(start_input)
        if obs[int(node_i[0]/10 + 50),int(node_i[1]/10 + 100)]==1:
            print('Start State inside an obstacle. Try again...')
        else:
            break
    except:
        print('Input must be three integers separated by a comma and space (ex: 10, 10, 30). Acceptable range for first value: -499 to 5500. Acceptable range for second value: -999 to 1000. Acceptable range for third value: 0 to 359. Try again...')
while 1:
    try:
        goal_input = input("Goal State (mm):")
        node_g = input2node(goal_input)
        if obs[int(node_g[0]/10+50),int(node_g[1]/10+100)]==1:
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
v_w = int(cost_go/10)
c_w = 1 #(1 / 60) * (cost_go - 180)
# 35 worked for the c_w for 50, 50 to 1500, 60

# [Total cost, cost to come, index, parent, [x, y, theta], distance traveled to reach the point, change in angle]
el1 = [cost_go, 0, 0, 0, node_i, 0, 0]
open_l.put(el1) # starting the open list

# Starting the search
id = el1[2]  # index of new nodes
mat_exp_ol = np.zeros((607, 207))  # empty matrix to record where we have explored in the open list (we don't care about angle at the goal)
mat_exp_cl = np.zeros((607, 207))  # empty matrix to record where we have explored in the closed list
mat_cost = np.zeros((607, 207)) # saving the cost in a matrix
# node_path = [0, 0]  # creating this to test a bug
while not open_l.empty():
    # pull out a node and add it to the closed list
    x = open_l.get()
    closed_l.put(x)
    # print(x)
    # pull out useful elements
    cur_cost = x[0]
    cur_come = x[1]
    cur_ind = x[2]
    cur_par = x[3]
    cur_pos = x[4]

    mat_exp_cl[round(cur_pos[0]/10)+50][round(cur_pos[1]/10)+100] = 1  # now we have explored this in the closed list
    # check if we have reached the goal
    thresh = np.sqrt((cur_pos[0] - node_g[0])**2 + (cur_pos[1] - node_g[1])**2)
    cur_time = time.time()
    if (cur_time - start_time) > 240:
        print("Aborting attempt")
        thresh = 0
    if thresh <= 50:
        # run the backtrack function
        node_path = trace_back(closed_l, cur_par, cur_ind)
        ## added to the print line below to verify that final state is correct
        print("Success! Confirm final state with 50mm threshold:",'('+str(round(cur_pos[0]))+', '+str(round(cur_pos[1]))+', '+str(round(cur_pos[2]))+')')
        break
    
    else:
        # for i in range(0,8):
        #     rpm = act[i, :]
        #     # no slip condition
        #     ul = float(rpm[0] * pi / 30)
        #     ur = float(rpm[1] * pi / 30)
        #     dx, dy, dth = move_step(ul, ur, float(cur_pos[2]*pi/180))
        #     dist = float(np.sqrt(dx**2 + dy**2))
        #     new_pos = [cur_pos[0] + dx, cur_pos[1] + dy, cur_pos[2] + dth]
        for action in act:
            ul = float(action[0] * math.pi / 30)
            ur = float(action[1] * math.pi / 30)
            x_new, y_new, th_new, dist, dtheta = cost(cur_pos[0], cur_pos[1], cur_pos[2], ul, ur)
            new_pos = [x_new, y_new, th_new]
            # delta = [dist, dtheta]
            # print(new_pos)
            # keep theta value between 0 and 359
            if new_pos[2]>359:
                new_pos[2] = new_pos[2]-360
            elif new_pos[2]<0:
                new_pos[2] = new_pos[2]+360
            # Check if the new location is in the closed list or obstacle space
            if new_pos[0]>5500 or new_pos[0]<-500 or new_pos[1]<-1000 or new_pos[1]>1000:
                continue
            check_cl = mat_exp_cl[round(new_pos[0]/10)+50][round(new_pos[1]/10)+100]
            check_ol = mat_exp_ol[round(new_pos[0]/10)+50][round(new_pos[1]/10)+100]
            check_ob = obs[round(new_pos[0]/10)+50][round(new_pos[1]/10)+100] #no theta value for obstacles
            # cg = float(np.sqrt(dx**2 + dy**2))
            cost_come = cur_come + dist
            cost_go = c_w * np.sqrt((node_g[0]-new_pos[0])**2 + (node_g[1]-new_pos[1])**2)  # weighted cost to go
            lxu = cost_come+cost_go  # combined cost

            if check_ob == 1: #decoupled the checks into separate statements
                continue #we can skip all the updating code if the new node is in the obstacle space
            elif check_cl == 0:
                if check_ol == 0:
                    mat_exp_ol[round(new_pos[0]/10)+50][round(new_pos[1]/10)+100] = 1
                    id = id + 1
                    # lxu is cost to come plus cost to go
                    new_node = [lxu, cost_come, id, cur_ind, new_pos, dist, dtheta]
                    mat_cost[round(new_pos[0]/10)+50][round(new_pos[1]/10)+100] = lxu  #update the cost matrix
                    open_l.put(new_node)
                # Finding a repeat in the open loop
                elif check_ol == 1:
                    m_co = mat_cost[round(new_pos[0]/10)+50][round(new_pos[1]/10)+100]
                    # If the cost of the old node is larger
                    if m_co >  lxu:
                        for j in range(0, open_l.qsize()):
                            check_pos = open_l.queue[j][4]
                            i_c = round(check_pos[0]/10 + 50)
                            j_c = round(check_pos[1]/10+100)
                            if i_c == (round(new_pos[0]/10+50)) and j_c == (round(new_pos[1]/10+100)):
                                idx = j
                                rep_pos = open_l.queue[idx][4]
                                rep_ind = open_l.queue[idx][2]
                                rep_node = open_l.queue[idx]
                                open_l.queue.remove(rep_node)
                                imp_q = [lxu, cost_come, rep_ind, cur_ind, rep_pos, dist, dtheta]
                                open_l.put(imp_q)
                                break
            elif check_cl == 1:
                continue

# record the path
plt1_size = closed_l.qsize()
len_pa = len(node_path)
# x_v = []
# y_v = []
# th_v = []
vel = []
angvel = []
dt=1
for i_pa in range(0, len_pa):
    ind_pa = node_path[i_pa]
    for j_pa in range(0, plt1_size):
        if closed_l.queue[j_pa][2] == ind_pa:
            # for n in range(10):
            vel.append(closed_l.queue[j_pa][5]/(1000*dt))
            angvel.append(closed_l.queue[j_pa][6]/dt)
            # for v in closed_l.queue[j_pa][5]:
            #     vel.append(v/(1000*dt))
            # for a in closed_l.queue[j_pa][6]:
            #     angvel.append(a/dt)
            # x_v.append(closed_l.queue[j_pa][6][0] / (1000 * dt))
            # y_v.append(closed_l.queue[j_pa][6][1] / (1000 * dt))
            # th_v.append(closed_l.queue[j_pa][6][2] * pi / (180 * dt))
            
## used to time how long code takes to execute
end_time = time.time()
print('Total time (s):', end_time-start_time)

# Creating a talker function
def astar_path():
    msg = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('astar_path', anonymous=True)
    # rate = rospy.Rate(1/dt)
    vel_cmd = 0
    ang_cmd = 0
    for i in range(len(vel)):
        if not rospy.is_shutdown():
        # if not rospy.is_shutdown():
            vel_cmd = vel[i]
            ang_cmd = angvel[i]
            msg.linear.x = float(vel_cmd)
            msg.angular.z = float(ang_cmd)
            #buffer is based on the dt value
            pub.publish(msg)
            rospy.sleep(dt)

    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub.publish(msg)

if __name__ == '__main__':
    astar_path()