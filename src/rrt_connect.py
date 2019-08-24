#!usr/local/bin/python3

import scipy.io as scio
import numpy as np
import matplotlib.pyplot as plt
import random as random 
from math import sqrt
from rrt_core import Node, RRT_Core

class RRT_Connect(object):
    # initilization
    def __init__(self, map, start_position, goal_position):
        # initilization start and goal tree
        self.start_tree_ = RRT_Core(map)
        self.goal_tree_ =RRT_Core(map)
        # put start and goal node into trees
        start = Node()
        start.id_ = 0
        start.position_ = start_position
        self.start_tree_.nodes_.append(start)
        goal = Node()
        goal.id_ = 0
        goal.position_ = goal_position 
        self.goal_tree_.nodes_.append(goal)
        self.prob_select_goal_ = 0.2
        self.max_iter_ = 2000
        self.check_interval_ = 5.0
        self.prob_round_ = 4
        self.path_ =[]
        self.find_feasible_path_ = False
        self.max_single_len_ = 30.0

    # build Rapidly random exploring tree 
    def buildTree(self):
        for i in range(self.max_iter_):
            node_rand = self.start_tree_.sample()
            node_nearest = self.start_tree_.findNearestNode(node_rand)
            node_new = self.start_tree_.steer(node_rand, node_nearest)
            if self.start_tree_.checkEdgeInFreespace(node_nearest.position_, node_new.position_) \
                    == True:
                if (self.start_tree_.addNewNode(node_new) ==  False):
                    continue 
                node_nearest_1 = self.goal_tree_.findNearestNode(node_new)
                node_new_1 = self.goal_tree_.steer(node_new, node_nearest_1)
                if (self.goal_tree_.checkEdgeInFreespace(node_new_1.position_, node_nearest_1.position_)) \
                        == True:
                    if (self.goal_tree_.addNewNode(node_new_1) == False):
                        continue
                    while (node_new != node_new_1):









    # find path from start point to goal point
    def findPath(self):
        if self.find_feasible_path_ == False: 
            print("no feasible path found, maybe need more iterations")
            return 
        # put goal index into feasible path 
        self.path_.append(len(self.nodes_) - 1)
        parent_id = self.nodes_[-1].parent_ 
        while parent_id != 0:
            self.path_.append(parent_id)
            parent_id = self.nodes_[parent_id].parent_ 
        # put root index into feasible path
        self.path_.append(0)
        self.path_.reverse()

    # draw Rapidly random exploring tree
    def drawRRT(self):
        # draw map 
        plt.ion()
        plt.matshow(self.map_, cmap = "gray_r")
        # draw start and goal point
        plt.scatter(self.start_position_[1], self.start_position_[0], marker = 'o')
        plt.scatter(self.goal_position_[1], self.goal_position_[0], marker = 'o')
        # draw RRT 
        for node in self.nodes_:
            # print("ID: " , node.id_)
            for child in node.chidren_:
                # print("parent id is ", node.id_, ", child id is ", child)
                if child < 0 or child > len(self.nodes_):
                    # TODO raise error to show bug
                    print("child < 0 or child > len(self.nodes_)")
                    return 
                x = [node.position_[0], self.nodes_[child].position_[0]]
                y = [node.position_[1], self.nodes_[child].position_[1]]
                plt.pause(0.002)
                plt.plot(y, x, color = 'r')
                plt.draw()

        # draw feasible path
        self.findPath()
        for i in range(len(self.path_) - 1):
            x = [self.nodes_[self.path_[i]].position_[0], \
                    self.nodes_[self.path_[i + 1]].position_[0]]
            y = [self.nodes_[self.path_[i]].position_[1], \
                    self.nodes_[self.path_[i + 1]].position_[1]]
            plt.pause(0.05)
            plt.plot(y, x, color = 'k')
        plt.draw()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    ## load map data
    data_file = "../assets/map.mat"
    data = scio.loadmat(data_file)
    data = data['map']
    start_position = (70, 80)
    goal_position = (615, 707)
    rrt = RRT_Connect(data, start_position, goal_position)
    rrt.buildTree()
    # rrt.drawRRT()
