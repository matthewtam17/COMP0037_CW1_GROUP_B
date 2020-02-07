# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch

class LIFOPlanner(CellBasedForwardSearch):

    # This implements a simple LIFO (last in first out or depth first) search algorithm
    
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.lifoQueue = list()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        #Checks if the new cell length is more than the existing max cell length.
        #If it is then update the max queue length value.
        if len(self.lifoQueue) > self.max_queue_length:
            self.max_queue_length = self.lifoQueue
        self.lifoQueue.append(cell)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.lifoQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.lifoQueue.pop()
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass
