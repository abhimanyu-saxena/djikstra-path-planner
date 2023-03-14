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
    elif ((x >= (230)) and (x <= (369)) and 
        (y <= (-40/69)*x + 8764/69) and (y <= (4/7)*x - 732/7) and 
        (y >= (40/69)*x - 13812/69) and (y >= (-4/7)*x + 8492/7)):
        return True
    
    # bloated triangle equations
    elif ((y <= (-114/59)*x + 9955/59) and (y >= (114/59)*x + 6515/59) and (x>=457)):
        return True
    
    # if robot is not in any above area
    else:
        return False

# function to generate all physically valid children
## input: pointer to current node
## output: Valid children list of a node

def getChildList(node):
    xMax, yMax = [600+1,250+1]
    xMin, yMin = [0,0]
    
    xCurr, yCurr = node.loc
    children = [] 
    
    # check if moving down is possible 
    if yCurr > yMin:
        (actionCost, child) = moveDown(node)
        if not inObstacleSpace(child.loc):
            # if node is not in obstacle space, append in child list
            children.append((actionCost, child))
        else:
            del child
            
    # check if moving up is possible 
    if yCurr < yMax:
        (actionCost, child) = moveUp(node)
        if not inObstacleSpace(child.loc):
            # if node is not in obstacle space, append in child list
            children.append((actionCost, child))
        else:
            del child
            
    # check if moving left is possible 
    if xCurr > xMin:
        (actionCost, child) = moveLeft(node)
        if not inObstacleSpace(child.loc):
            # if node is not in obstacle space, append in child list
            children.append((actionCost, child))
        else:
            del child
    
    # check if moving right is possible 
    if xCurr < xMax:
        (actionCost, child) = moveRight(node)
        if not inObstacleSpace(child.loc):
            # if node is not in obstacle space, append in child list
            children.append((actionCost, child))
        else:
            del child  
    
    # check if moving up-right is possible 
    if yCurr < yMax and xCurr < xMax:
        (actionCost, child) = moveUpRight(node)
        if not inObstacleSpace(child.loc):
            # if node is not in obstacle space, append in child list
            children.append((actionCost, child))
        else:
            del child
    
    
    # check if moving up-left is possible 
    if yCurr < yMax and xCurr > xMin:
        (actionCost, child) = moveUpLeft(node)
        if not inObstacleSpace(child.loc):
            # if node is not in obstacle space, append in child list
            children.append((actionCost, child))
        else:
            del child
    
    
    # check if moving down-right is possible 
    if yCurr > yMin and xCurr < xMax:
        (actionCost, child) = moveDownRight(node)
        if not inObstacleSpace(child.loc):
            # if node is not in obstacle space, append in child list
            children.append((actionCost, child))
        else:
            del child
    
    
    # check if moving down-left is possible 
    if yCurr > yMin and xCurr > xMin:
        (actionCost, child) = moveDownLeft(node)
        if not inObstacleSpace(child.loc):
            # if node is not in obstacle space, append in child list
            children.append((actionCost, child))
        else:
            del child
    
    return children

# function to generate itme for open list
## input: pointer to current node
## output: tuple of (node.ctc, node)

def olItem(node):
    item = (node.ctc, node)
    return item

# function to backtrack to generate path
## input: pointer to goal node
## output: list of path taken

def backtrack(current):
    path = []
    parent = current
    while parent!= None:
        path.append(parent.loc)
        parent = parent.parent
    return path[::-1]


def djikstra(start, goal):
    start_time = time.time()
    # create object to save vizualization video
    saveViz = cv2.VideoWriter('djikstra_viz.avi',cv2.VideoWriter_fourcc('M','J','P','G'),60,(600+1,250+1))
    space = drawObstacles((600,250))
    space_viz = drawObstacles((600,250))
    pathTaken = []
    
    # check if start position is in obstacle space
    if inObstacleSpace(start.loc):
        print("Start position is in Obstacle space")
        return False, pathTaken
    
    # check if goal position is in obstacle space
    if inObstacleSpace(goal.loc):
        print("Start position is in Obstacle space")
        return False, pathTaken
    
    # mark start and goal locations
    cv2.circle(space, (start[0],space.shape[0]-start[1]-1), 1, (0, 0, 255), 2)
    cv2.circle(space, (goal[0],space.shape[0]-goal[1]-1), 1, (255, 0, 0), 2)
    
    # initialize open and closed list
    openList = []
    openListData = {}
    closedList = []
    closedListData = {}
    
    # push start node in open-list
    init = createNode(start,None,0)
    openList.append(olItem(init))
    openListData[init.loc] = init
    
    # run loop till path is found or open-list is empty
    while len(openList) > 0:
        
        # pop node with least cost from open-list
        openList.sort(key = lambda x: x[0])
        currentCost, current = openList.pop(0)
        openListData.pop(current.loc)
        
        # add node to closed list
        closedList.append(current)
        closedListData[current.loc] = current
        
        # check if current node is goal or not
        if current.loc == goal:
            
            # backtrack to start if current node is goal
            pathTaken = backtrack(current)
            for i in pathTaken:
                space[space.shape[0]-i[1]-1,i[0]] = [0,0,0]
                space_viz[space.shape[0]-i[1]-1,i[0]] = [0,0,0]
                saveViz.write(space)
            cv2.imwrite("Path_Taken.jpg", space_viz)
            saveViz.release()
            end_time = time.time()
            print("Time taken = ", (end_time-start_time)/1000, " sec")
            return True, pathTaken
        else:
            
            # else get children of current node
            childList = getChildList(current)
            for actionCost, actionChild in childList:
                
                # ignore child if it is in closed list
                if actionChild.loc in list(closedListData.keys()):
                    del actionChild
                    continue
                
                # check for child in open list
                if actionChild.loc in list(openListData.keys()):
                    
                    # if better path is present, update cost and parent of child
                    if openListData[actionChild.loc].ctc > current.ctc + actionCost:
                        openListData[actionChild.loc].parent = current
                        openListData[actionChild.loc].ctc = current.ctc + actionCost
                        
                # if not found in open list, add child to open list
                else:
                    actionChild.parent = current
                    actionChild.ctc = current.ctc + actionCost
                    openList.append(olItem(actionChild))
                    openListData[actionChild.loc] =  actionChild
                    x,y = actionChild.loc
                    space[space.shape[0]-y-1,x] = [144, 238, 144]
                    saveViz.write(space)
        
        return False, pathTaken

def main():
    print("----Djikstra's Path Planner----")
    start_x = int(input("Enter x co-ordnate of start position: "))
    start_y = int(input("Enter y co-ordnate of start position: "))

    goal_x = int(input("Enter x co-ordnate of goal position: "))
    goal_y = int(input("Enter y co-ordnate of goal position: "))    
    

if __name__ == "__main__":
    main()