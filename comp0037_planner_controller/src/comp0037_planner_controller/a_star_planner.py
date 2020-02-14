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

    #  Calculates the Euclidean distance to the goal
    def EuclideanDistance(self,cell):
        return math.sqrt((self.goal.coords[0]-cell.coords[0])**2 + (self.goal.coords[1]-cell.coords[1])**2)

    def OctileDistance(self,cell):
        return None

    def constantDistance(self,cell):
        return 5

    def ManhattanDistance(self,cell):
        return None

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
