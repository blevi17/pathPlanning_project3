# pathPlanning_project3
This is the Path Planning Project 3 Github Repository of Levi Butler and Julia Macon Kim
https://github.com/blevi17/pathPlanning_project3
The videos for this project are included in the Google Drive at the links below.  They has been shared with the grader's UMD email
https://drive.google.com/drive/folders/0APgw_6vLq5KGUk9PVA 

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
    1. Have the necessary Turtlebot files predownloaded in your catkin_ws
    2. Add the given map.world into the folder ~/turtlbot3_simulations/turtlebot3_gazebo/worlds
    3. Add turtlebot3_map.launch to the folder ~/turtlbot3_simulations/turtlebot3_gazebo/launch
    4. Add the publisher, newpub.py, to the src folder of your chosen catkin workspace
    5. Use catkin_make at your discretion
    6. Two terminal windows or two terminal tabs are necessary for the new few steps
    7. Launch the turtlebot using the following line of code
       roslaunch turtlebot3_gazebo turtlebot3_map.launch
    8. To change the default position from (0, 0, 0), add 'x_pos:=XX y_pos:=XX' to the above line of code in meters.  Ex: x_pos:=1.5 y_pos:=0
    9. To change the default orientation of the turtlbot the 'yaw' can be similarly changed in the terminal
    10. After the turtlbot is launched, use cd to reach the folder of the publisher code and run with python3
        Ex: cd ~/catkin_ws/src && python3 newpub.py
    11. The inputs as the same as those in proj3p2_levi_julia.py.  Note: rpms under 8 will cause an error as the dt = 0.1 seconds and the resolution of the code is 1 cm
    12. After the code outputs the word 'Success!' it will begin publshing to the x-linear velocity and z-angular velocity of the turtlebot
    13. The publisher will stop the turtlebot once it has finshed executing its main for loop
