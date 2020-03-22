import math
import cv2
import numpy as np
import time
from math import pi,sin,atan,cos
from heapq import heappush, heappop
#Normalizing the positions with thresholding
def normalize(Node,thresholdDistance=0.5, thresholdAngle=30):
    x,y,t = Node[0],Node[1],Node[2]
    x = round(x/thresholdDistance)* thresholdDistance
    y = round(y/thresholdDistance)* thresholdDistance
    t = round(t/thresholdAngle) * thresholdAngle
    n=t//360
    t=t-(360*n)
    t=(t/30)
    return [x,y,int(t)]
# Calculating the Euclidean distance
def distance(startCoordinate,goalCoordinate):
    sx,sy = startCoordinate[0],startCoordinate[1]
    gx,gy = goalCoordinate[0],goalCoordinate[1]
    return math.sqrt((gx-sx)**2 + (gy-sy)**2)#instead of using scipy.spatial importing distance
def listToString(s):  
        str1 = ""  
        for ele in s:  
             str1 += str(ele)  
        return str1  
   
#original obs
#Checking the boundary conditions for the obstacle space
def boundary_check(i,j):
    if (i<tot or j>199-tot or j<tot or i>299-tot):
        return 0
    else:
        return 1
def obs_map_pt(x,y):
    circle = ((np.square(x-225))+ (np.square(y-50)) <=np.square(25))
    ellipse = (((np.square(x-150))/np.square(40))+((np.square(y-100))/np.square(20)) -1 <=0)
    rhombus = (x*(-3/5)+y-55<0) and (x*(3/5)+y-325<0) and (x*(-3/5)+y-25>0) and (x*(3/5)+y-295 > 0)
    rectangle = ((200-y) - (1.73)*x + 135 > 0 and (200-y) + (0.58)*x - 96.35  <= 0 and (200-y) - (1.73)*x - 15.54 <= 0 and (200-y) + (0.58)*x - 84.81 >= 0)
    polygon1 = ((y+13*x-340>0) and x+y-100<0 and y+(-7/5)*x+20>0)#triangle1
    polygon2 = (y-15>0 and (7/5)*x+y-120<0 and y+(-7/5)*x+20<0)#triangle2
    polygon3 = ((7/5)*x+y-120>0 and (-6/5)*x+y+10<0 and (6/5)*x+y-170<0 and (-7/5)*x+y+90>0)#rhombus
   
    if circle or ellipse or rhombus or rectangle or polygon1 or polygon2 or polygon3 :
        obj_val = 0
    else:
        obj_val = 1
   
    return obj_val
#Obstacle Map with Rigid body clearence
def obs_map(x,y):
    circle = ((np.square(x-225))+ (np.square(y-50)) <=np.square(25+tot))
    ellipse = (((np.square(x-150))/np.square(40+tot))+((np.square(y-100))/np.square(20+tot)) -1 <=0)
    rhombus = (x*(-3/5)+y-55-(tot/sin(pi/2 -atan(-3/5)))<0) and (x*(3/5)+y-325-(tot/sin(pi/2 -atan(3/5)))<0) and (x*(-3/5)+y-25+(tot/sin(pi/2 -atan(-3/5)))>0) and (x*(3/5)+y-295+(tot/sin(pi/2 -atan(3/5))) > 0)
    rectangle = ((200-y) - (1.73)*x + 135+(tot/sin(pi/2 -atan(1.73))) > 0 and (200-y) + (0.58)*x - 96.35-(tot/sin(pi/2 -atan(0.58)))  <= 0 and (200-y) - (1.73)*x - 15.54-(tot/sin(pi/2 -atan(1.73)))<= 0 and (200-y) + (0.58)*x - 84.81+(tot/sin(pi/2 -atan(0.58)))>= 0)
    polygon1 = ((y+13*x-340+(tot/sin(pi/2 -atan(13)))>0) and x+y-100-(tot/sin(pi/2 -atan(1)))<0 and y+(-7/5)*x+20>0)#triangle
    polygon2 = (y-15+tot>0 and (7/5)*x+y-120<0 and y+(-7/5)*x+20<0)#triangle
    polygon3 = ((7/5)*x+y-120>0 and (-6/5)*x+y+10-(tot/sin(pi/2 -atan(-6/5)))<0 and (6/5)*x+y-170-(tot/sin(pi/2 -atan(6/5)))<0 and (-7/5)*x+y+90+(tot/sin(pi/2 -atan(-7/5)))>0)#rhombus
    if circle or ellipse or rhombus or rectangle or polygon1 or polygon2 or polygon3 :
        obj_val = 0
    else:
        obj_val = 1
   
    return obj_val
# Move Func
def move(i,t,s):
    t=math.radians(t)
    p=math.radians(i[2]*30)
    new_node=[i[0]+(s*cos(t+p)),i[1]+(s*sin(t+p)),math.degrees(t+p)]  
    return new_node
# Priority Queue Function
def  convergence(a,goal_node):
            if((np.square(a[0]-goal_node[0]))+ (np.square(a[1]-goal_node[1])) <=np.square(1.5)) :
                            return 0
            else:
                return 1
           
###############  TAKING INPUTS ################################
rad=int(input('Please enter radius of the robot:'))
clearance=int(input('Please enter the clearance:'))

tot=rad+clearance

#Step size
s=int(input("Please enter the step size from 1 to 10:"))
#Getting the start nodes from the user
x_start = int(input("Please enter start point x coordinate:"))
y_start = int(input("Please enter start point y coordinate:"))
theta_start = int(input("Please enter start orientation in degrees:"))

y_start = 199 - y_start

start_obs = obs_map(x_start,y_start)
start_boundary = boundary_check(x_start,y_start)

while(start_obs!=1 or start_boundary!=1):
    print("Incorrect start point! Please enter a valid start point:")
    x_start = int(input("Please enter start point x coordinate:"))
    y_start = int(input("Please enter start point y coordinate:"))
    y_start = 199 - y_start
    theta_start = int(input("Please enter start orientation in degrees:"))
    start_obs = obs_map(x_start,y_start)
    start_boundary = boundary_check(x_start,y_start)
 
start=normalize([x_start,y_start,theta_start])
start_node=[int(start[0]),int(start[1]),start[2]]
norm_current_node=start_node
#Geting the goal nodes from the user
x_goal=int(input("Please enter goal point x coordinate:"))
y_goal=int(input("Please enter goal point y coordinate:"))
y_goal=199 - y_goal
theta_goal = int(input("Please enter goal orientation in degrees:"))


goal_obs=obs_map(x_goal,y_goal)
goal_boundary=boundary_check(x_goal,y_goal)


while(goal_obs!=1 or goal_boundary!=1):
    print("Incorrect goal point! Please enter a valid goal point:")
    x_goal=int(input("Please enter another goal point x coordinate:"))
    y_goal=int(input("Please enter another goal point y coordinate:"))
    theta_goal = int(input("Please enter goal orientation in degrees:"))

    y_goal=199 - y_goal
    goal_obs=obs_map(x_goal,y_goal)
    goal_boundary=boundary_check(x_goal,y_goal)
goal=normalize([x_goal,y_goal,theta_goal])
goal_node=[int(goal[0]),int(goal[1]),goal[2]]

###################   INTIALISING COST,VISITED,PARENT AND QUEUE  ###############
#Initializing cost as infinity
cost_array=np.array(np.ones((600,400,12)) * np.inf)
#Initializing visited nodes as empty array
visited=np.array(np.zeros((600,400,12)))
#Heuristic distance-Eucledian
# euclidean_array=np.array(np.ones((600,400,12)) * np.inf)
euclidean_array=np.array(np.ones((600,400)) * np.inf)
#Initializing total cost with cost and distance
totalcost=np.array(np.ones((600,400,12)) * np.inf)
parent={}
Q=[]
# append start point and initialize it's cost to zero
heappush(Q,(0,start_node))
cost_array[int(2*start[0])][int(2*start[1])][start[2]]=0
totalcost[int(2*start[0])][int(2*start[1])][start[2]]=0
   
################PRE-PLOT OBSTACLE SPACE###################
x = 600
y = 400
image = np.ones((y,x,3),np.uint8)*255
# image = (np.full([y,x], 0, dtype='uint8'))

for i in range(400):
    for j in range(600):
            image[i][j] = (175,175,175)
           
#Outer edges mapping
for x in range(600):
    for y in range(400):
        if obs_map(x/2,y/2)==0:
                 image[y,x] = 0
                 
                 #Outer edges mapping
for x in range(600):
    for y in range(400):
        if obs_map_pt(x/2,y/2)==0:
                 image[y,x] = (255,255,255)
#cv2.imshow("A-star Search",image)
#cv2.waitKey(0)
###################### A-STAR##################################
start_time=time.time()    
breakflag=0
while convergence(norm_current_node,goal_node):
    if breakflag==1:
        break
    norm_current_node=heappop(Q)[1]
    if convergence(norm_current_node,goal_node)==0:
        goalfound=norm_current_node
        print("found")
        print(norm_current_node)
        break
    for t in [0,30,60,330,300]:
        curr_node=move(norm_current_node,t,s)
        norm_curr_node=normalize(curr_node)
        if convergence(norm_curr_node,goal_node)==0:
            print("found")
            parent[listToString(norm_curr_node)]=norm_current_node
            goalfound=norm_curr_node
            print(norm_curr_node)
            breakflag=1
            break
        status=boundary_check(norm_curr_node[0],norm_curr_node[1])
        flag=obs_map(norm_curr_node[0],norm_curr_node[1])
        if (status and flag == 1):
          if visited[int(2*norm_curr_node[0]),int(2*norm_curr_node[1]),norm_curr_node[2]]==0:
            visited[int(2*norm_curr_node[0]),int(2*norm_curr_node[1]),norm_curr_node[2]]=1
            parent[listToString(norm_curr_node)]=norm_current_node
            cv2.line(image,(int(2*norm_current_node[0]),int( (2*norm_current_node[1]))),(int(2*norm_curr_node[0]),int( (2*norm_curr_node[1]))),(255,0,0), 2)
            cv2.imshow("A-star Search",image)
            cv2.waitKey(1)
            cost_array[int(2*norm_curr_node[0]),int(2*norm_curr_node[1]),norm_curr_node[2]]=s+cost_array[int(2*norm_current_node[0]),int(2*norm_current_node[1]),norm_current_node[2]]
            euclidean_array[int(2*norm_curr_node[0]),int(2*norm_curr_node[1])]=distance(norm_curr_node, goal_node)
            totalcost[int(2*norm_curr_node[0]),int(2*norm_curr_node[1]),norm_curr_node[2]]=cost_array[int(2*norm_curr_node[0]),int(2*norm_curr_node[1]),norm_curr_node[2]] + euclidean_array[int(2*norm_curr_node[0]),int(2*norm_curr_node[1])]
            heappush(Q,( totalcost[int(2*norm_curr_node[0]),int(2*norm_curr_node[1]),norm_curr_node[2]] ,norm_curr_node ))
          else:
             if totalcost[int(2*norm_curr_node[0]),int(2*norm_curr_node[1]),norm_curr_node[2]]>(totalcost[int(2*norm_current_node[0]),int(2*norm_current_node[1]),norm_current_node[2]]+s):
                totalcost[int(2*norm_curr_node[0]),int(2*norm_curr_node[1]),norm_curr_node[2]]=(totalcost[int(2*norm_current_node[0]),int(2*norm_current_node[1]),norm_current_node[2]]+s)              
                parent[listToString(norm_curr_node)]=norm_current_node


################## BACKTRACK ##########################
path=[]
def path_find(goal,start):
    path.append(goal)
    GN=parent[listToString(goal)]
    path.append(GN)
    while (GN!=start):
        GN=parent[listToString(GN)]
        path.append(GN)
    return path


path=path_find(goalfound,start)
path.reverse()
for i in range(len(path)-1):
        cv2.line(image,(int(2*(path[i])[0]),int( (2*(path[i])[1]))),(int(2*(path[i+1])[0]),int( (2*(path[i+1])[1]))),(0,0,255), 3)
        cv2.imshow("A-star Search",image)
        cv2.waitKey(1)
print(time.time()-start_time)  
cv2.waitKey(0)

cv2.destroyAllWindows()

