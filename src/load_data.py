#!usr/local/bin/python3

import scipy.io as scio
import numpy as np
import matplotlib.pyplot as plt
import random as random 
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

class RRT_Core(object):
    # initilization
    def __init__(self, map, start_position, goal_position):
        self.map_ = map
        self.start_position_ = np.array(start_position) 
        self.goal_position_ = np.array(goal_position) 
        self.prob_select_goal_ = 0.2
        self.max_iter_ = 2000
        self.check_interval_ = 5.0
        self.prob_round_ = 4
        self.nodes_ = []
        self.path_ =[]
        self.find_feasible_path_ = False
        self.max_single_len_ = 30.0

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
    def findNearestNodeID(self, new_node):
        min_dist = float('inf')
        min_id = -1
        for i in range(len(self.nodes_)):
            dist = self.calcDistance(self.nodes_[i], new_node)
            if dist < min_dist:
                min_dist = dist
                min_id = i 
        return min_id 

    # find new node according to rand node and nearest node
    def findNewNode(self, node_rand):
        node_new = Node()
        # find neasted node
        nearest_id = self.findNearestNodeID(node_rand)
        # print("newest id is ", nearest_id)
        # return false when invalid nearest node id 
        if nearest_id == -1 or nearest_id >= len(self.nodes_):
            print("can`t find nearest node!")
            return node_new 
        direction = (node_rand.position_ - self.nodes_[nearest_id].position_)
        length = np.linalg.norm(direction)
        length_threshold = 1.0
        if length < length_threshold:
            print("node rand is too close to existed nodes, abandon it.")
            return node_new 
        direction = direction / length 
        curr_position = np.array([-1, -1])
        if length > self.max_single_len_:
            curr_position = self.nodes_[nearest_id].position_ + direction * self.max_single_len_
        else:
            curr_position = node_rand.position_ 
        curr_position = curr_position.astype(int)
        if self.checkEdgeInFreespace(self.nodes_[nearest_id].position_, curr_position) == True:
            node_new.parent_ = nearest_id
            node_new.position_ = curr_position 

        # for i in range(self.prob_round_, -1, -1):
            # curr_position = self.nodes_[nearest_id].position_ + direction * prob_interval * i
            # curr_position = curr_position.astype(int) 
            # if self.checkEdgeInFreespace(self.nodes_[nearest_id].position_, curr_position) is True:
                # node_new.parent_ = nearest_id 
                # node_new.position_ = curr_position
                # return node_new 
        return node_new 

    # sample to get rand node
    def sample(self):
        # select goal with prob
        node_rand = Node()
        if random.random() < self.prob_select_goal_:
            node_rand.position_ = self.goal_position_ 
        else: # select random point otherwise
            n_row, n_col = self.map_.shape
            postion_rand = np.array([random.randint(0, n_row - 1), \
                    random.randint(0, n_col - 1)])
            node_rand.position_ = postion_rand
        return node_rand

    # build Rapidly random exploring tree 
    def buildTree(self):
        random.seed()
        # put start point into nodes 
        root = Node()
        root.id_ = len(self.nodes_)
        root.position_ = self.start_position_  
        # id of root node should be 0
        root.parent_ = 0
        self.nodes_.append(root)
        goal = Node()
        goal.position_ = self.goal_position_

        for i in range(self.max_iter_):
            node_rand = self.sample()
            node_new = self.findNewNode(node_rand)
            if node_new.parent_ < 0 or node_new.parent_ >= len(self.nodes_):
                # print("new node is invalid!")
                continue
            else:
                node_new.id_ = len(self.nodes_)
                self.nodes_.append(node_new)
                self.nodes_[node_new.parent_].chidren_.append(node_new.id_)
                # flag of break iteration
                if node_new == goal:
                    print("find feasible path form start to goal, break iteration")
                    self.find_feasible_path_ = True 
                    break

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
    print(data.shape)
    start_position = (70, 80)
    goal_position = (615, 707)
    rrt = RRT_Core(data, start_position, goal_position)
    # p1 = np.array([487, 357])
    # p2 = np.array([615, 707])
    # print(rrt.checkEdgeInFreespace(p1, p2))
    rrt.buildTree()
    rrt.drawRRT()
