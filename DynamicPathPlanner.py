import heapq
import matplotlib.pyplot as plt
import numpy as np

# Define a class to represent nodes in the search graph
class Node:
    def __init__(self, indices:np.array, g, h, parent, epsilon):
        self.indices = indices.flatten()
        self.g = g
        self.h = h
        self.parent = parent
        self.epsilon = epsilon

    def f(self):
        # TODO: YOUR IMPLEMENTATION HERE
        return self.g + self.epsilon*self.h

    # Define __lt__ for comparison with other nodes in the priority queue
    def __lt__(self, other):
        return self.f() < other.f()

class DynamicPathPlanner:
    def __init__(self, env):
        pass

    def Plan(self, start, goal):
        path = []
        # TODO: YOUR IMPLEMENTATION HERE
        return path

    def plot(self, ):
        # Write your own plot function to visualize
        pass