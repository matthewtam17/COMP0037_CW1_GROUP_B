# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
import Queue
import math

# This class implements the greedy planning
# algorithm. It works by using a priority queue. Each element in the queue has a priority, and 
# the element with the highest priority is always popped first.

class GreedyPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.greedyQueue = Queue.PriorityQueue()

    # Find the cell's "priority value" and add onto the priority queue.
    # We can simply add the cell onto the back of the queue because the get() function returns us the highest priority cell
    def pushCellOntoQueue(self, cell):
        #Checks if the new cell length is more than the existing max cell length.
        #If it is then update the max queue length value.
        if self.greedyQueue.qsize() > self.max_queue_length:
            self.max_queue_length = self.greedyQueue.qsize()
        self.greedyQueue.put((self.EuclideanDistance(cell),cell))

    #  Calculates the Euclidean distance to the goal
    def EuclideanDistance(self,cell):
        return math.sqrt((self.goal.coords[0]-cell.coords[0])**2 + (self.goal.coords[1]-cell.coords[1])**2)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.greedyQueue.empty()

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        priority, cell = self.greedyQueue.get()
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass
