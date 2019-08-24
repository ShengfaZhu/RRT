#!usr/local/bin/python3

import scipy.io as scio
import numpy as np
import matplotlib.pyplot as plt
import random as random 
import copy
from math import sqrt

## Node
class Node(object):
    def __init__(self):
        self.id_ = -1
        self.position_ = np.array([-1.0, -1.0], dtype = int)
        self.parent_ = -1
        self.chidren_ = []

    def __eq__(self, rhs):
        distance_threshold = 1.0
        if np.linalg.norm(self.position_ - rhs.position_) <= distance_threshold:
            return True
        else:
            return False

    def printInfo(self):
        print("ID: ", self.id_)
        print("position : ", self.position_)
        print("parent id : ", self.parent_)
        print("chidren id : ", self.chidren_)

class RRT_Core(object):
    # initilization
    def __init__(self, map):
        self.map_ = map
        self.check_interval_ = 5.0
        self.nodes_ = []
        # self.max_single_len_ = 30.0

    # calculate Euclid distance between two points
    def calcDistance(self, node1, node2):
        dist = np.linalg.norm(node1.position_- node2.position_)
        return dist 

    # check if node in freespace 
    def isInFreespace(self, node):
        if self.map_[(node.position_)] is 0:
            return True
        else:
            return False

    # check if edge in freespace
    def checkEdgeInFreespace(self, position1, position2):
        if self.map_[tuple(position1)] == 1 or self.map_[tuple(position2)] == 1:
            return False
        direction = position1 - position2
        length = np.linalg.norm(direction)
        if length < 2.0:
            return True
        direction = direction / length
        prob_times = np.linalg.norm(position1 - position2) / self.check_interval_ - 1
        # TODO shuffle to detect 
        # unnecessary to check prob_round less than 1, due to range()
        for i in range(1, int(prob_times)):
            prob_position = position2 + direction * self.check_interval_ * i
            if self.map_[tuple(prob_position.astype(int))] == 1:
                return False 
        return True 

    # find nestest node according to distance
    def findNearestNode(self, new_node):
        min_dist = float('inf')
        min_id = -1
        for i in range(len(self.nodes_)):
            dist = self.calcDistance(self.nodes_[i], new_node)
            if dist < min_dist:
                min_dist = dist
                min_id = i 
        node_nearest = Node()
        if (min_id < 0 or min_id >= len(self.nodes_)):
            print("Tree is empty, can`t find nearest node!")
            return node_nearest 
        node_nearest = self.nodes_[min_id]
        return node_nearest

    # sample to get rand node
    def sample(self):
        # selecting goal with prob is unnecessary 
        node_rand = Node()
        n_row, n_col = self.map_.shape
        postion_rand = np.array([random.randint(0, n_row - 1), \
                random.randint(0, n_col - 1)])
        node_rand.position_ = postion_rand
        return node_rand

    # add new node into rrt
    def addNewNode(self, node_new):
        # verify if node is valid
        if (node_new.parent_ < 0 or node_new.parent_ >= len(self.nodes_)):
            print("node is invalid!, reject to add into tree")
            return False  
        # add into rrt
        node_new.id_ = len(self.nodes_)
        self.nodes_[node_new.parent_].chidren_.append(node_new.id_)
        print("node is added into tree sucessfully")
        return True 

if __name__ == '__main__':
    pass
