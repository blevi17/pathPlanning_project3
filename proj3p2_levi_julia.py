# This is the main python file for Path Planning Project 3 Phase 2
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

################################## Functions ###############################################
# define function to convert user input into node format
def input2node(input):
    output = []
    for num in input.split(', '):
        output.append(float(num))
    return output

# Variables from the datasheet of the turtlebot
R = 66  # wheel radius in mm
r = 105 # necessary buffer around the robot in mm
L = 160  # wheel center to center distance in mm
v_mr = 2.84  # maximum rotational velocity in rad/s
v_mrev = float(v_mr * 60 / (2 * pi))  # maximum rotational velocity in revolutions/minute
v_mt = 220  # maximum translational velocity in mm/s according to the spec sheet

# Taking in user input
while 1:
    try:
        C = float(input("Clearance:"))
        if L>190 or L<0:
            print('Clearance is too large to guarantee the goal is reachabel! Try Again...')
        else:
            break
    except:
        print('Input must be a number between 0 and 190. Try again...')
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
        print('Input must be two integers separated by a comma and space (ex: 10, 10). Acceptable range for first value: 1 to 600. Acceptable range for second value: 1 to 250. Try again...')
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

## used to time how long code takes to execute
end_time = time.time()
print('Total time (s):', end_time-start_time)
