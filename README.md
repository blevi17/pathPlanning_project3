# pathPlanning_project3
This is the Path Planning Project 3 Github Repository of Levi Butler and Julia Macon Kim
https://github.com/blevi17/pathPlanning_project3
The videos for this project are included in the Google Drive at the links below.  They has been shared with the grader's UMD email
    phase 1 video:
    phase 2 video:

Team member names and IDs:
    Levi Butler (lbutle11, 119444733)
    Julia Kim (jkim603, 119451717)

Libraries/dependencies:
    numpy
    queue.PriorityQueue
    matplotlib.pyplot
    matplotlib.animation
    math
    time
    rospy
    geometry_msgs.msg, Twist

Phase 01 Instructions:
    1. Run file: proj3p2_levi_julia.py
    2. Input desired clearance around obstacles when probmpted, in mm. Ex: 50
    3. Input start node when prompted, in format x, y, theta. Theta should be in degrees, x and y in mm. Ex: 0, 0, 0
    4. Input goal node when prompted, in format x, y in mm. Ex: 5000, 500
    5. Input 2 RPM values when prompted, in format RPM1, RPM2. Ex: 10, 15
    6. Wait for the code to calculate results. It will automatically time out after 5 minutes if calculation is taking too long.
    7. The code will output a confimration of the final node location and an animation of the path in the obstacle space. Explored nodes are in red and the final path is in green. The obstacles AND the clearance are both shown in blue.
    
Phase 02 Instructions:
    1. 
