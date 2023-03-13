# -*- coding: utf-8 -*-
"""
@author: saxen
"""

# import required libraries

import numpy as np
import matplotlib.pyplot as plt
import cv2

# create node class to initialise each node with location, parent and cost-to-come

class createNode:
    def __init__(self, loc, parent, ctc):
        self.loc = loc
        self.parent = parent
        self.ctc = ctc

# create action set for the robot, creates a node in the direction for possibility
def moveDown(node):
    cost = 1
    x,y = node.loc
    actionChild = createNode((x,y-1),node,node.ctc+cost)
    return cost,actionChild

def moveUp(node):
    cost = 1
    x,y = node.loc
    actionChild = createNode((x,y+1),node,node.ctc+cost)
    return cost,actionChild

def moveLeft(node):
    cost = 1
    x,y = node.loc
    actionChild = createNode((x-1,y),node,node.ctc+cost)
    return cost,actionChild

def moveRight(node):
    cost = 1
    x,y = node.loc
    actionChild = createNode((x+1,y),node,node.ctc+cost)
    return cost,actionChild

def moveUpLeft(node):
    cost = 1.4
    x,y = node.loc
    actionChild = createNode((x-1,y+1),node,node.ctc+cost)
    return cost,actionChild

def moveUpRight(node):
    cost = 1.4
    x,y = node.loc
    actionChild = createNode((x+1,y+1),node,node.ctc+cost)
    return cost,actionChild

def moveDownLeft(node):
    cost = 1.4
    x,y = node.loc
    actionChild = createNode((x-1,y-1),node,node.ctc+cost)
    return cost,actionChild

def moveDownRight(node):
    cost = 1.4
    x,y = node.loc
    actionChild = createNode((x+1,y-1),node,node.ctc+cost)
    return cost,actionChild