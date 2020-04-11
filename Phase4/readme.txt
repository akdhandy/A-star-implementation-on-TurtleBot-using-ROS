Project 3 Phase 4 - Implementation of A*Star Algorithm for Non-Holonomic Robot using ROS-Gazebo

Dependencies:

    Ubuntu 16.04
    ROS kinetic
    Gazebo
    catkin
    Turtlebot3 packages

Libraries:

    math
    rospy
    numpy
    time
    matplotlib
    heapq
    argparse
    geometry_msgs-Twist
    std_msgs.msg-String

Initial Setup:

Install turtlebot3-gazebo package by running the below command:

$ sudo apt install ros-kinetic-turtlebot3-gazebo

In your .bashrc file, include the following statements and source it.

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger

To create a catkin workspace and install dependecies for turtlebot3 package, run the following commands:

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ cd ~/catkin_ws/
$ catkin_make


From the downloaded file,which is the package "searchbot" folder, copy to /catkin_ws/src/ and run the following commands:

$ cd ~/catkin_ws/
$ source ~/catkin_ws/devel/setup.bash

Instructions to run:

The inputs from the user are coordinates of start point, orientation of start point (in degrees), coordinates of goal point, two RPM values and clearance.

The orientation of goal point is taken by default as 0.

The clearance given by the user is 0.1 (minimum clearance).

In the terminal run the following command:

roslaunch searchbot astar.launch  x_pos:=4.5 y_pos:=3.0 yaw:=3.142

The x,y coordinates of start point in gazebo should be given in arguments 'x_pos' and 'y_pos' respectively in the terminal. The argument 'yaw' is (3.14 + orientation at start of the robot in radians). This ensures the turtlebot to spawn at correct position and orientation in gazebo.

The start coordinates and goal coordinates should be given as negative of the coordinates as observed in right-handed system because the map given has negative axes. For example, if the robot start point is bottom left (-4.5,-4.5) in right-handed coordinate system, it should be given as (4.5,4.5).

Inputs for the submitted videos:

Video 1:
For Manual input enter 1 else enter 2
1
Enter the clearance.1
Please enter the rpm1:3
Please enter the rpm2:4
Please enter start point x coordinate:4.5
Please enter start point y coordinate:3.0
Please enter start orientation in degrees:0
Please enter goal point x coordinate:-4.5
Please enter goal point y coordinate:-3.0

Video2:
For Manual input enter 1 else enter 2
1
Enter the clearance.1
Please enter the rpm1:3
Please enter the rpm2:4
Please enter start point x coordinate:4.5
Please enter start point y coordinate:3.0
Please enter start orientation in degrees:0
Please enter goal point x coordinate:0
Please enter goal point y coordinate:3.0

