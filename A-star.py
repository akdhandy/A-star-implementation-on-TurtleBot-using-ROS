import math
import numpy as np
import cv2 as cv
import time
from scipy.spatial import distance
from math import pi,sin,atan,cos


#############  BOUNDS FUNCTION ####################################
#Checking the boundary conditions for the obstacle space
def boundary_check(i,j):
    if (i<tot or j>199-tot or j<tot or i>299-tot):
        return 0
    else:
        return 1
#original obs
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
    theta_goal = int(input("Please enter goal orientation in degrees:"))
    y_start = 199 - y_start
    theta_start = int(input("Please enter start orientation in degrees:"))
    start_obs = obs_map(x_start,y_start)
    start_boundary = boundary_check(x_start,y_start)
 

start=[x_start,y_start]


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
 

goal=[x_goal,y_goal]


###################   INTIALISING COST,VISITED,PARENT AND QUEUE  ###############
#Initializing cost as infinity 
cost_array=np.array(np.ones((600,400,12)) * np.inf)
#Initializing visited nodes as empty array
visited=np.array(np.zeros((600,400,12)))
#Heuristic distance-Eucledian
euclidean_array=np.array(np.ones((600,400,12)) * np.inf)
#Initializing total cost with cost and distance
totalcost=np.array(np.ones((600,400,12)) * np.inf)
parent_list=np.zeros((600,400,12))
parent_list=list(parent_list)
Q=[]

# append start point and initialize it's cost to zero
Q.append([x_start,y_start,theta_start])
cost_array[x_start][y_start][theta_start]=0
totalcost[x_start][y_start][theta_start]=0

# Priority Queue Function
def pop(Q):
    minimum_index=0
    minimum_X = Q[0][0] 
    minimum_Y = Q[0][1]
    minimum_th=Q[0][2]
    for i in range(len(Q)):
        x = Q[i][0]
        y = Q[i][1]
        z=Q[i][2]
        if totalcost[x,y,z] < totalcost[minimum_X,minimum_Y,minimum_th]:
            minimum_index = i
            minimum_X = x 
            minimum_Y= y
            minimum_th=z
        
    current_node = Q[minimum_index]
    Q.remove(Q[minimum_index])
    return current_node


def move(i,j,t,s):
    t=math.radians(t)
    new_node=[i+s*cos(t),j+s*sin(t)]     
    return new_node

#Djikstra Algorithm
start_time=time.time()
goalpoint=[x_goal,y_goal,theta_goal]    
visited_node=[]
current_node=[x_start,y_start,theta_start]
while current_node[0:2]!=goal[0:2]:
    current_node=pop(Q)
    
    for t in range(0,360,30):
        curr_node=move(current_node[0],current_node[1],t,s)
        status=boundary_check(curr_node[0],curr_node[1])
        flag=obs_map(curr_node[0],curr_node[1])
        if (status and flag == 1):
          if visited[2*curr_node[0],2*curr_node[1]][t]==0:
            visited[curr_node[0],curr_node[1]]=1
            visited_node.append(curr_node)
            Q.append(curr_node)
            parent_list[curr_node[0]][curr_node[1]]=current_node
            cost_array[curr_node[0],curr_node[1]]=(cost_array[current_node[0],current_node[1]]+s)
            euclidean_array[curr_node[0],curr_node[1]]=distance.euclidean(curr_node, goalpoint)
            totalcost[curr_node[0],curr_node[1]]= cost_array[curr_node[0],curr_node[1]]+euclidean_array[curr_node[0],curr_node[1]]
          else:
            if totalcost[curr_node[0],curr_node[1]]>(totalcost[current_node[0],current_node[1]]+1):
                totalcost[curr_node[0],curr_node[1]]=(totalcost[current_node[0],current_node[1]]+1)
                parent_list[curr_node[0]][curr_node[1]]=current_node
  
print("Goal reached")

#Pathfinder function
goal=[x_goal,y_goal]
start=[x_start,y_start]
path=[]
def path_find(goal,start):
    GN=goal
    path.append(goal)
    while (GN!=start):
        a=parent_list[GN[0]][GN[1]]
        path.append(a)
        GN=a

path_find(goal,start)
print('The cost of the shortest path is',totalcost[x_goal,y_goal])







#############   PLOTTING STARTS ####################
plot_x=[]
plot_y=[]

for i in range(200):
    for j in range(300):
         if( obs_map(j, i)==0) :
            plot_x.append(j)
            plot_y.append(i)
plt.figure(figsize=(9,6))
plt.xlim(0,300)
plt.ylim(0,200)
plt.plot(plot_x,plot_y,".r") 
plt.show() 
plt.pause(5) 
plt.close()



