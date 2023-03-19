# This is the main python file for Path Planning Project 3 Phase 2
# by Levi Butler and Julia Macon Kim
# GitHub repository: https://github.com/blevi17/pathPlanning_project3

import numpy as np
from queue import PriorityQueue
import matplotlib.pyplot as plt
from matplotlib import animation
from sympy import *
import time

# Variables from the datasheet of the turtlebot
R = 66  # wheel radius in mm
r = 105 # necessary buffer around the robot
L = 160  # wheel center to center distance in mm
v_mr = 2.84  # maximum rotational velocity in rad/s
v_mt = 220  # maximum translational velocity in mm/s according to the spec sheet
