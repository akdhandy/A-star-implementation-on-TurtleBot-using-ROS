# ENPM-661-Project-3
Implementation of A* Algorithm for rigid robot in an obstacle workspace

Dependencies:
	cv2
	time
	math
	heapq
	numpy
TO RUN:Navigate to the codes folder in the submission folder proj3p2_28_python and run
python3 Astar_rigid.py

DESCRIPTION - The python program  to perform A* algorithm for a rigid robot based on user's input.
The algorithms are run in an obstacle filled environment composed of a rectangle, circle, ellipse,rhombus and a polygon. The algorithm finds the shortest path to goal node from the start node(both user provided) and displays the explored nodes and the path.

	Upon the running the code the user is prompted to provide a number of inputs. The sample usecase has been provided below -

	Please enter radius of the robot:-->
	1(**then press Enter)
	
	Please enter the clearance:-->
	1(**then press Enter)
	
	PLease enter the step size from 1 to 10:-->
	1(**then press Enter)
	
	Please enter start point x coordinate -->
	50(**press Enter)

	Please enter start point y coordinate -->
	30(**press Enter)
	
	Please enter start orientation in degrees:-->
	60(**press Enter)
	
	Please enter goal point x coordinate -->
	150(**press Enter)
	
	Please enter goal point y coordinate -->
	150(**press Enter)
	
	Please enter goal orientation in degrees:-->
	0(**press Enter)
	
	
