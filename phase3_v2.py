#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 03:20:05 2020

@author: arjun
"""

import math
import cv2
import numpy as np
import time
from math import pi,sin,atan,cos
from heapq import heappush, heappop
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt

##DRAW MAP#########
fig, ax = plt.subplots()
plt.figure(figsize=(10,10))
circle1=plt.Circle((5,5),1,fill=None)
circle2=plt.Circle((3,2),1,fill=None)
circle3=plt.Circle((7,2),1,fill=None)
circle4=plt.Circle((7,8),1,fill=None)

fig = plt.gcf()
fig.gca().add_artist(circle1)
fig.gca().add_artist(circle2)
fig.gca().add_artist(circle3)
fig.gca().add_artist(circle4)

# plt.show()  
currentAxis = plt.gca()
currentAxis.add_patch(Rectangle((0.25, 4.25), 1.5, 1.5, fill=None, alpha=1))
currentAxis.add_patch(Rectangle((8.25, 4.25), 1.5, 1.5, fill=None, alpha=1))
currentAxis.add_patch(Rectangle((2.25, 7.25), 1.5, 1.5, fill=None, alpha=1))

# # plt.grid()
# fig=plt.gcf()

ax.set_aspect('equal')

plt.xlim(0,10)
plt.ylim(0,10)

# plt.title('A-Star Search ',fontsize=10)

#Normalizing the positions with thresholding
def normalize(Node,thresholdDistance=0.01, thresholdAngle=10):
    x,y,t = Node[0],Node[1],Node[2]
    x = round(x/thresholdDistance)* thresholdDistance
    y = round(y/thresholdDistance)* thresholdDistance
    t = round(t/thresholdAngle) * thresholdAngle
    x=round(x,4)
    y=round(y,4)
    n=t//360
    t=t-(360*n)
    t=(t/thresholdAngle)
    return [x,y,int(t)]
thresholdDistance=0.01  #0.01
nd=int(1/thresholdDistance)  #100
sizex=10*nd  #1000
thresholdAngle=10   #10
na=int(360/thresholdAngle)  #36

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
    if (i<tot or j>=10 or j<tot or i>=10):
        return 0
    else:
        return 1
#Obstacle Map with Rigid body clearence
def obs_map(x,y):
    circle1 = ((np.square(x-5))+ (np.square(y-5)) <=np.square(1+tot))
    circle2 = ((np.square(x-3))+ (np.square(y-2)) <=np.square(1+tot))
    circle3 = ((np.square(x-7))+ (np.square(y-2)) <=np.square(1+tot))
    circle4 = ((np.square(x-7))+ (np.square(y-8)) <=np.square(1+tot))
    square1=(x>=0) and (x<=1.75+tot) and (y>=4.25-tot) and (y <= 5.75+tot)
    square2=(x>=8.25-tot) and (x<=9.75+tot) and (y>=4.25-tot) and (y <= 5.75+tot)
    square3=(x>=2.25-tot) and (x<=3.75+tot) and (y>=7.25-tot) and (y <= 8.25+tot)
    if circle1 or circle2 or circle3 or circle4 or square1 or square2 or square3 :
        obj_val =0
    else:
        obj_val = 1
   
    return obj_val
def plot_curve(Xi,Yi,Thetai,UL,UR,p):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.2
    Xn=Xi
    Yn=Yi
    Thetan = math.radians(Thetai)
    while t<2:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += r * (UL + UR) * math.cos(Thetan) * dt
        Yn += r * (UL + UR) * math.sin(Thetan) * dt
        status=boundary_check(Xn,Yn)
        flag=obs_map(Xn,Yn)
        if (status and flag != 1):
            return None
        Thetan += (r / L) * (UR - UL) * dt
        if(p==True):
          plt.plot([Xs, Xn], [Ys, Yn], color="blue")
        if(p==False):
          plt.plot([Xs, Xn], [Ys, Yn], color="red")
    Thetan = math.degrees(Thetan)
    return [Xn, Yn, Thetan]


def move_bot(Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.2
    Xn=Xi
    Yn=Yi
    Thetan = math.radians(Thetai*thresholdAngle)
# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes

    while t<2:
        t = t + dt
        Xn += r * (UL + UR) * math.cos(Thetan) * dt
        Yn += r * (UL + UR) * math.sin(Thetan) * dt
        status=boundary_check(Xn,Yn)
        flag=obs_map(Xn,Yn)
        if (status and flag != 1):
            print(flag)
            return None
        Thetan += (r / L) * (UR - UL) * dt
    Thetan = math.degrees(Thetan)
    return [Xn, Yn, Thetan]

# Priority Queue Function
def  convergence(a,goal_node):
            if((np.square(a[0]-goal_node[0]))+ (np.square(a[1]-goal_node[1])) <=np.square(0.25)) :
                            return 0
            else:
                return 1
           
###############  TAKING INPUTS ################################
# clearance of the robot 
tot=0.3
########  pre init #####################3
rpm1=5
rpm2=6
x_start = float(1.1)
y_start = float(1.1)
theta_start = int(0)
x_goal=float(5.0)
y_goal=float(1.5)
theta_goal = int(15)
actions=[[rpm1,0],[0,rpm1],[rpm1,rpm1],[0,rpm2],[rpm2,0],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]
start=normalize([x_start,y_start,theta_start])
norm_current_node=start
goal=normalize([x_goal,y_goal,theta_goal])
goal_node=[goal[0],goal[1],goal[2]]

#################### user init ################################
# #Wheel RPM
# rpm1=int(input("Please enter the rpm1:"))
# rpm2=int(input("Please enter the rpm2:"))
# actions=[[rpm1,0],[0,rpm1],[rpm1,rpm1],[0,rpm2],[rpm2,0],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]

# #Getting the start nodes from the user
# x_start = float(input("Please enter start point x coordinate:"))
# y_start = float(input("Please enter start point y coordinate:"))
# theta_start = int(input("Please enter start orientation in degrees:"))
# start_obs = obs_map(x_start,y_start)
# start_boundary = boundary_check(x_start,y_start)

# while(start_obs!=1 or start_boundary!=1):
#     print("Incorrect start point! Please enter a valid start point:")
#     x_start = float(input("Please enter start point x coordinate:"))
#     y_start = float(input("Please enter start point y coordinate:"))
#     theta_start = int(input("Please enter start orientation in degrees:"))
#     start_obs = obs_map(x_start,y_start)
#     start_boundary = boundary_check(x_start,y_start)
 
# start=normalize([x_start,y_start,theta_start])
# # start_node=[int(start[0]),int(start[1]),start[2]]
# norm_current_node=start
# #Geting the goal nodes from the user
# x_goal=float(input("Please enter goal point x coordinate:"))
# y_goal=float(input("Please enter goal point y coordinate:"))
# theta_goal = int(input("Please enter goal orientation in degrees:"))
# goal_obs=obs_map(x_goal,y_goal)
# goal_boundary=boundary_check(x_goal,y_goal)


# while(goal_obs!=1 or goal_boundary!=1):
#     print("Incorrect goal point! Please enter a valid goal point:")
#     x_goal=float(input("Please enter another goal point x coordinate:"))
#     y_goal=float(input("Please enter another goal point y coordinate:"))
#     theta_goal = int(input("Please enter goal orientation in degrees:"))
#     goal_obs=obs_map(x_goal,y_goal)
#     goal_boundary=boundary_check(x_goal,y_goal)
# goal=normalize([x_goal,y_goal,theta_goal])
# goal_node=[goal[0],goal[1],goal[2]]

###################   INTIALISING COST,VISITED,PARENT AND QUEUE  ###############
#Initializing cost as infinity
cost_array=np.array(np.ones((sizex,sizex,na)) * np.inf)
#Initializing visited nodes as empty array
visited=np.array(np.zeros((sizex,sizex,na)))
#Heuristic distance-Eucledian
# euclidean_array=np.array(np.ones((sizex,sizex))* np.inf)
euclidean_array=np.array(np.ones((sizex,sizex)) * np.inf)

#Initializing total cost with cost and distance
# totalcost=np.array(np.ones((sizex,sizex,na) * np.inf))
totalcost=np.array(np.ones((sizex,sizex,na)) * np.inf)

parent={}
Q=[]
# append start point and initialize it's cost to zero
heappush(Q,(0,start))
cost_array[int(nd*start[0])][int(nd*start[1])][start[2]]=0
totalcost[int(nd*start[0])][int(nd*start[1])][start[2]]=0
explored=[]
###################### A-STAR##################################
start_time=time.time()    
breakflag=0

while convergence(norm_current_node,goal_node):
    if breakflag==1:
        break
    norm_current_node=heappop(Q)[1]
    if convergence(norm_current_node,goal_node)==0:
        goalfound=[norm_current_node,action]
        print("found")
        print(norm_current_node)
        break
    for action in actions:
        curr_node=move_bot(norm_current_node[0], norm_current_node[1], norm_current_node[2], action[0], action[1])
        if(curr_node==None):
            continue
        norm_curr_node=normalize(curr_node)
        if convergence(norm_curr_node,goal_node)==0:
            print("found")
            parent[listToString(norm_curr_node)]=[norm_current_node,action]
            goalfound=[norm_curr_node,action]
            print(norm_curr_node)
            breakflag=1
            break
        status=boundary_check(norm_curr_node[0],norm_curr_node[1])
        flag=obs_map(norm_curr_node[0],norm_curr_node[1])
        if (status and flag == 1):
          if visited[int(nd*norm_curr_node[0]),int(nd*norm_curr_node[1]),norm_curr_node[2]]==0:
            visited[int(nd*norm_curr_node[0]),int(nd*norm_curr_node[1]),norm_curr_node[2]]=1
            explored.append([norm_current_node,action,norm_curr_node])
            parent[listToString(norm_curr_node)]=[norm_current_node,action]
            cost_array[int(nd*norm_curr_node[0]),int(nd*norm_curr_node[1]),norm_curr_node[2]]=sum(action)+cost_array[int(nd*norm_current_node[0]),int(nd*norm_current_node[1]),norm_current_node[2]]
            euclidean_array[int(nd*norm_curr_node[0]),int(nd*norm_curr_node[1])]=distance(norm_curr_node, goal_node)
            totalcost[int(nd*norm_curr_node[0]),int(nd*norm_curr_node[1]),norm_curr_node[2]]=cost_array[int(nd*norm_curr_node[0]),int(nd*norm_curr_node[1]),norm_curr_node[2]] + euclidean_array[int(nd*norm_curr_node[0]),int(nd*norm_curr_node[1])]
            heappush(Q,( totalcost[int(nd*norm_curr_node[0]),int(nd*norm_curr_node[1]),norm_curr_node[2]] ,norm_curr_node ))
          else:
             if totalcost[int(nd*norm_curr_node[0]),int(nd*norm_curr_node[1]),norm_curr_node[2]]>(totalcost[int(nd*norm_current_node[0]),int(nd*norm_current_node[1]),norm_current_node[2]]+sum(action)):
                totalcost[int(nd*norm_curr_node[0]),int(nd*norm_curr_node[1]),norm_curr_node[2]]=(totalcost[int(nd*norm_current_node[0]),int(nd*norm_current_node[1]),norm_current_node[2]]+sum(action))
                explored.append([norm_current_node,action,norm_curr_node])
                parent[listToString(norm_curr_node)]=[norm_current_node,action]


################## BACKTRACK ##########################
path=[]
def path_find(goal,start):
    path.append(goal)
    GN=parent[listToString(goal[0])]
    path.append(GN)
    while (GN[0]!=start):
        GN=parent[listToString(GN[0])]
        path.append(GN)
    return path


path=path_find(goalfound,start)
path.reverse()
print("Time taken")
print(time.time()-start_time)  

sx=x_start
sy=y_start
sz=theta_start

# for action in explored:
#     x1= plot_curve(action[0][0],action[0][1],action[0][2], action[1][0],action[1][1],p=True)


for action in path:
    x1= plot_curve(sx,sy,sz, action[1][0],action[1][1],p=False)
    sx=x1[0]
    sy=x1[1]
    sz=x1[2]

# for action in path:
#     x1= move_bot(sx,sy,sz, action[1][0],action[1][1]) 
#     print(x1)
#     sx=x1[0]
#     sy=x1[1]
#     sz=x1[2]
