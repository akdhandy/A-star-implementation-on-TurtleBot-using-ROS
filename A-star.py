#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 18 19:25:54 2020

@author: arjun
"""

#!/usr/bin/env python
# coding: utf-8
"""
Created on Fri Mar  6 18:54:30 2020

@author: Arun
"""
import math
import numpy as np
import cv2 as cv
import time
from scipy.spatial import distance
from math import pi,sin,atan



rad=int(input('Please enter radius of the robot:'))
clearance=int(input('Please enter the clearance:'))

tot=rad+clearance


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

x = 300
y = 200
image = np.ones((y,x,3),np.uint8)*255

for i in range(200):
    for j in range(300):
            image[i][j] = (175,175,175)
            
#Outer edges mapping
for x in range(300):
    for y in range(200):
        if obs_map(x,y)==0:
                 image[y,x] = 0
                 
                 #Outer edges mapping
for x in range(300):
    for y in range(200):
        if obs_map_pt(x,y)==0:
                 image[y,x] = (255,255,255)


parent_list=[]
for j in range (300):
    column=[]
    for i in range (200):
        column.append(0)
    parent_list.append(column)

#Step size
s=int(input("Please enter the step size from 1 to 10:"))
#Getting the start nodes from the user
x_start = int(input("Please enter start point x coordinate:"))
y_start = int(input("Please enter start point y coordinate:"))
y_start = 199 - y_start

start_obs = obs_map(x_start,y_start)
start_boundary = boundary_check(x_start,y_start)

while(start_obs!=1 or start_boundary!=1):
    print("Incorrect start point! Please enter a valid start point:")
    x_start = int(input("Please enter start point x coordinate:"))
    y_start = int(input("Please enter start point y coordinate:"))
    y_start = 199 - y_start
    start_obs = obs_map(x_start,y_start)
    start_boundary = boundary_check(x_start,y_start)
 

start=[x_start,y_start]


#Geting the goal nodes from the user
x_goal=int(input("Please enter goal point x coordinate:"))
y_goal=int(input("Please enter goal point y coordinate:"))
y_goal=199 - y_goal

goal_obs=obs_map(x_goal,y_goal)
goal_boundary=boundary_check(x_goal,y_goal)


while(goal_obs!=1 or goal_boundary!=1):
    print("Incorrect goal point! Please enter a valid goal point:")
    x_goal=int(input("Please enter another goal point x coordinate:"))
    y_goal=int(input("Please enter another goal point y coordinate:"))
    y_goal=199 - y_goal
    goal_obs=obs_map(x_goal,y_goal)
    goal_boundary=boundary_check(x_goal,y_goal)
 

goal=[x_goal,y_goal]



#Initializing cost as infinity 
cost_array=np.array(np.ones((300,200)) * np.inf)
#Initializing visited nodes as empty array
visited=np.array(np.zeros((600,400,12)))
#Heuristic distance-Eucledian
euclidean_array=np.array(np.ones((300,200)) * np.inf)
#Initializing total cost with cost and distance
totalcost=np.array(np.ones((300,200)) * np.inf)

#Size of workspace
size=(200,300)

mapy=np.ones((size),np.uint8)*255


Q=[]

# append start point and initialize it's cost to zero

Q.append([x_start,y_start])
cost_array[x_start][y_start]=0
totalcost[x_start][y_start]=0
print(Q)
# Priority Queue Function

def pop(Q):
    minimum_index=0
    minimum_X = Q[0][0] 
    minimum_Y = Q[0][1]
    for i in range(len(Q)):
        x = Q[i][0]
        y = Q[i][1]
        if cost_array[x,y] < cost_array[minimum_X,minimum_Y]:
            minimum_index = i
            minimum_X = x 
            minimum_Y= y
        
    current_node = Q[minimum_index]
    Q.remove(Q[minimum_index])
    return current_node



# # All possible movements
# def north(i,j):
#     new_node=[i,j+1]     
#     return new_node


def move(i,j,t,s):
    t=math.radians(t)
    new_node=[i+s*cos(t),j+s*sin(t)]     
    return new_node

#Djikstra Algorithm
start_time=time.time()
goalpoint=[x_goal,y_goal]    
visited_node=[]
current_node=[x_start,y_start]
while current_node!=goal:
    current_node=pop(Q)
    
    for t in range(0,360,30):
        new_node=move(current_node[0],current_node[1],t,s)
    # new_north=north(current_node[0],current_node[1])
    # status=boundary_check(new_north[0],new_north[1])
    # flag=obs_map(new_north[0],new_north[1])
    # if (status and flag == 1):
    #     if visited[new_north[0],new_north[1]]==0:
    #         visited[new_north[0],new_north[1]]=1
    #         visited_node.append(new_north)
    #         Q.append(new_north)
    #         parent_list[new_north[0]][new_north[1]]=current_node
    #         cost_array[new_north[0],new_north[1]]=(cost_array[current_node[0],current_node[1]]+1)
    #         euclidean_array[new_north[0],new_north[1]]=distance.euclidean(new_north, goalpoint)
    #         totalcost[new_north[0],new_north[1]]= cost_array[new_north[0],new_north[1]]+euclidean_array[new_north[0],new_north[1]]
    #     else:
    #         if cost_array[new_north[0],new_north[1]]>(cost_array[current_node[0],current_node[1]]+1):
    #             cost_array[new_north[0],new_north[1]]=(cost_array[current_node[0],current_node[1]]+1)
    #             parent_list[new_north[0]][new_north[1]]=current_node
    #             euclidean_array[new_north[0],new_north[1]]=distance.euclidean(new_north, goalpoint)
    #             totalcost[new_north[0],new_north[1]]= cost_array[new_north[0],new_north[1]]+euclidean_array[new_north[0],new_north[1]]
  
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



print('The cost of the shortest path is',cost_array[x_goal,y_goal])

pic=cv.resize(image,None,fx=2,fy=2)


#Showing the graphical output
cv.circle(image,(int(goal[0]),int(goal[1])), (1), (0,0,255), -1);
cv.circle(image,(int(start[0]),int(start[1])), (1), (0,0,255), -1);

for i in visited_node:
    cv.circle(image,(int(i[0]),int(i[1])), (1), (255,0,0));
    pic=cv.resize(image,None,fx=2,fy=2)
    cv.imshow('map',pic)
    cv.waitKey(1)

for i in path:
    cv.circle(image,(int(i[0]),int(i[1])), (1), (150,50,204));
    pic=cv.resize(image,None,fx=2,fy=2)
    cv.imshow('map',pic)
    cv.waitKey(1)
    
print("Total time:")
print(time.time()-start_time)   
cv.waitKey(0) 
cv.destroyAllWindows()







