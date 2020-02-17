# -*- coding: utf-8 -*-
from cell_based_forward_search import CellBasedForwardSearch
import Queue
import math

# This class implements the A* Planning
# algorithm. It works by using a priority queue based on the cost-to-go for each cell.
# the element with the highest priority is always popped first.

class AStarPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.astarQueue = Queue.PriorityQueue()

    # Find the cell's "priority value" and add onto the priority queue.
    # We can simply add the cell onto the back of the queue because the get() function returns us the highest priority cell
    def pushCellOntoQueue(self, cell):
        itercell = cell.parent
        travelCost = self.computeLStageAdditiveCost(itercell,cell)  + self.EuclideanDistance(cell)
        while (itercell is not None):
            travelCost = travelCost + self.computeLStageAdditiveCost(itercell.parent, itercell)
            itercell = itercell.parent
        cell.travelCost = travelCost
        self.astarQueue.put((travelCost,cell))
        #Checks if the new cell length is more than the existing max cell length.
        #If it is then update the max queue length value.
        if self.astarQueue.qsize() > self.max_queue_length:
            self.max_queue_length = self.astarQueue.qsize()
    #  Calculates the Euclidean distance to the goal
    def EuclideanDistance(self,cell):
        return math.sqrt((self.goal.coords[0]-cell.coords[0])**2 + (self.goal.coords[1]-cell.coords[1])**2)

    # Calculates the octile distance to the goal. Admissible if the robot moves in 8 directions.
    def OctileDistance(self,cell):
        return max(abs(cell.coords[0]-self.goal.coords[0]),abs(cell.coords[1]-self.goal.coords[1]))+(sqrt(2)-1)*min(abs(cell.coords[0]-self.goal.coords[0]),abs(cell.coords[1],self.goal.coords[1]))

    # Returns a constant as the heuristic
    def constantDistance(self,cell):
        return 5

    #Calculates the manhattan distance to the goal. Only admissible if the robot moves in four directions only.
    def ManhattanDistance(self,cell):
        return abs(cell.coords[0]-self.goal.coords[0])+abs(cell.coords[1]-self.goal.coords[1])

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.astarQueue.empty()

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        priority, cell = self.astarQueue.get()
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass
