# -*- coding: utf-8 -*-
from cell_based_forward_search import CellBasedForwardSearch
import Queue
import math
import numpy as np
from math import sqrt

# This class implements the A* Planning
# algorithm. It works by using a priority queue based on the cost-to-go for each cell.
# the element with the highest priority is always popped first.

class AStarPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.astarQueue = Queue.PriorityQueue()

        #Determine which heuristic to use!
        self.heuristic = 4

        #Determine the weighting of the scaled A* algorithm
        self.w = 1
        
        #Heuristics available:
        #0:Euclidean Distance 
        #1:Octile Distance
        #2:Manhattan Distance (non-admissible)
        #3:Minkowski Distance
        #4:Cosine Distance
        #5:Euclidean SQUARE Distance (non-admissible)

    # Find the cell's "priority value" and add onto the priority queue.
    # We can simply add the cell onto the back of the queue because the get() function returns us the highest priority cell
    def pushCellOntoQueue(self, cell):
        cell.pathCost = self.computePathCost(cell)
        self.astarQueue.put((cell.pathCost,cell))
        #Checks if the new cell length is more than the existing max cell length.
        #If it is then update the max queue length value.
        if self.astarQueue.qsize() > self.max_queue_length:
            self.max_queue_length = self.astarQueue.qsize()
    #  Calculates the Euclidean distance to the goal
    def cal_heuristic(self,cell):

        if self.heuristic == 0:
            #Euclidean distance to the goal. Admissible heuristic.
            return math.sqrt((self.goal.coords[0]-cell.coords[0])**2 + (self.goal.coords[1]-cell.coords[1])**2)
        elif self.heuristic == 1:
            #Octile distance to the goal. Admissible if the robot moves in 8 directions.
            return max(abs(cell.coords[0]-self.goal.coords[0]),abs(cell.coords[1]-self.goal.coords[1]))+(sqrt(2)-1)*min(abs(cell.coords[0]-self.goal.coords[0]),abs(cell.coords[1]-self.goal.coords[1]))
        elif self.heuristic == 2:
            #Manhattan distance to the goal. Only admissible if the robot moves in four directions only.
            return abs(cell.coords[0]-self.goal.coords[0])+abs(cell.coords[1]-self.goal.coords[1])
        elif self.heuristic == 3:
            #Minkowski sum distance to the goal.
            h=10
            return ((self.goal.coords[0]-cell.coords[0])**h + (self.goal.coords[1]-cell.coords[1])**h)**(1/h)
        elif self.heuristic == 4:
            #Cosine heuristic to the goal.
            return np.dot(self.goal.coords,cell.coords)/(np.sqrt(np.dot(self.goal.coords,cell.coords)*np.dot(self.goal.coords,cell.coords)))
        elif self.heuristic == 5:
            #Euclidean Square - demonstration of a NON-admissible heuristic
            return ((self.goal.coords[0]-cell.coords[0])**2 + (self.goal.coords[1]-cell.coords[1])**2)
        else:
            # Returns a constant as the heuristic
            return 100

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.astarQueue.empty()

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        priority, cell = self.astarQueue.get()
        return cell

    def computePathCost(self,cell):
        itercell = cell.parent
        pathCost = 0
        if itercell:
            pathCost = self.computeLStageAdditiveCost(itercell,cell)  + (self.w)*(self.cal_heuristic(cell))
        while (itercell is not None):
            pathCost = pathCost + self.computeLStageAdditiveCost(itercell.parent, itercell)
            itercell = itercell.parent
        return pathCost

    def resolveDuplicate(self, cell, parentCell):
        newPathpathCost = parentCell.pathCost-(self.w)*(self.cal_heuristic(parentCell)) + self.computeLStageAdditiveCost(parentCell,cell)
        #print("newpathpathCost:"+str(newPathpathCost))
        #print("parentCell.pathCost:"+str(parentCell.pathCost))
        #print("cell.pathCost:" + str(cell.pathCost))
        #print("parentCell.parents:"+str(parentCell.parent.coords))
        #while(parentCell.parent):
        #    print(parentCell.parent.coords)
        #    parentCell = parentCell.parent
        #print("cell.parent:"+str(cell.parent))
        #print("cell:"+str(cell.coords))
        #print("start:"+str(self.start.coords))
        if newPathpathCost < (self.computePathCost(cell)-(self.w)*(self.cal_heuristic(cell))) and cell != parentCell.parent:
            cell.parent = parentCell
            cell.pathCost = newPathpathCost
            self.pushCellOntoQueue(cell)
            
