# -*- coding: utf-8 -*-
"""
@author: saxen
"""

# import required libraries

import numpy as np
import matplotlib.pyplot as plt
import cv2
import time

# create node class to initialise each node with location, parent and cost-to-come

class createNode:
    def __init__(self, loc, parent, ctc):
        self.loc = loc
        self.parent = parent
        self.ctc = ctc

# create action set for the robot, creates a node in the direction for possibility
## input: pointer to current node
## output: cost of action, created child

# action for moving down
def moveDown(node):
    cost = 1
    x,y = node.loc
    actionChild = createNode((x,y-1),node,node.ctc+cost)
    return cost,actionChild

# action for moving up
def moveUp(node):
    cost = 1
    x,y = node.loc
    actionChild = createNode((x,y+1),node,node.ctc+cost)
    return cost,actionChild

# action for moving left
def moveLeft(node):
    cost = 1
    x,y = node.loc
    actionChild = createNode((x-1,y),node,node.ctc+cost)
    return cost,actionChild

# action for moving right
def moveRight(node):
    cost = 1
    x,y = node.loc
    actionChild = createNode((x+1,y),node,node.ctc+cost)
    return cost,actionChild

# action for moving Up-left
def moveUpLeft(node):
    cost = 1.4
    x,y = node.loc
    actionChild = createNode((x-1,y+1),node,node.ctc+cost)
    return cost,actionChild

# action for moving Up-right
def moveUpRight(node):
    cost = 1.4
    x,y = node.loc
    actionChild = createNode((x+1,y+1),node,node.ctc+cost)
    return cost,actionChild

# action for moving Down-left
def moveDownLeft(node):
    cost = 1.4
    x,y = node.loc
    actionChild = createNode((x-1,y-1),node,node.ctc+cost)
    return cost,actionChild

# action for moving Down-right
def moveDownRight(node):
    cost = 1.4
    x,y = node.loc
    actionChild = createNode((x+1,y-1),node,node.ctc+cost)
    return cost,actionChild

# function to draw obstacle space as given
## input: size of the arena, default = (600,250)
## output: obstacle space without bolating

def drawObstacles(size = (600,250)):
    w,h = size
    th = np.deg2rad(30)
    
    # initialize empty space
    space = np.zeros((h+1, w+1, 3), dtype = np.uint8)
    space.fill(255)
    
    # plot rectangles
    rec1 = np.array([[100,150], [150,150], [150, 250], [100,250]])
    rec2 = np.array([[100,100], [150,100], [150, 0], [100,0]])
    
    # plot triangle
    tri = np.array([[460,25], [510,125], [460, 225]])
    
    # plot hexagon
    hexgn = np.array([[300,200], [300+75*np.cos(th), 200-75*np.sin(th)], [300+75*np.cos(th), 50+75*np.sin(th)], [300,50],
                      [300-75*np.cos(th), 50+75*np.sin(th)] , [300-75*np.cos(th), 200-75*np.sin(th)]]).astype(int)
    
    # fill the shapes 
    space = cv2.fillPoly(space, pts=[rec1,rec2,tri,hexgn], color=(0, 0, 0))
    space = cv2.flip(space,0)
    return space

# function to check if robot is in obstacle space, also bloats the walls and the obstacles
## input: location of the robot
## output: boolean for presence in obstacle space

def inObstacleSpace(location):
    xMax, yMax = [600+1,250+1]
    xMin, yMin = [0,0]
    x,y = location
    bl = 5
    
    # bloated walls equation
    if (x < xMin+bl) or (y < yMin+bl) or (x >= xMax-bl) or (y >= yMax-bl):
        return True
    
    # bloated rectangles equation
    elif ((x <= 150+bl) and (x >= 100-bl) and (y <= 100+bl)) or ((x <= 150+bl) and (x >= 100-bl) and (y >= 150-bl)):
        return True
    
    # bloated hexagon equations
    elif ((x >= (235 - bl)) and (x <= (365 + bl)) and 
        ((x + 2*y) >= 395) and ((x - 2*y) <= 205) and 
        ((x - 2*y) >= -105) and ((x + 2*y) <= 705)):
        return True
    
    # bloated triangle equations
    elif ((y >= 1.75*x - 776.25) and (y <= -1.75*x + 1026.25) and (x >= 460-bl)):
        return True
    
    # if robot is not in any above area
    else:
        return False

