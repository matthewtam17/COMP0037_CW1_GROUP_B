# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import PriorityQueue

# This class implements the greedy planning
# algorithm. It works by using a priority queue. Each element in the queue has a priority, and 
# the element with the highest priority is always popped first.

class GreedyPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.greedyQueue = PriorityQueue()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        #Checks if the new cell length is more than the existing max cell length.
        #If it is then update the max queue length value.
        if len(self.greedyQueue) > self.max_queue_length:
            self.max_queue_length = len(self.greedyQueue)
        self.greedyQueue.append(cell)

    #  Calculates the Euclidean distance to the goal
    def EuclideanDistance():
        return 

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.greedyQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.greedyQueue.popleft()
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass
