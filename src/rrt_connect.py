#!usr/local/bin/python3

import scipy.io as scio
import numpy as np
import matplotlib.pyplot as plt
import random as random 
from math import sqrt
from rrt_node import Node

class RRT_Connect(object):
    # initilization
    def __init__(self, map, start_position, goal_position):
        self.map_ = map
        # initilization start and goal tree
        self.start_tree_ = []
        self.goal_tree_ = []
        self.start_position_ = start_position
        self.goal_position_ = goal_position 
        # expand parameters
        self.prob_select_goal_ = 0.2
        self.max_iter_ = 500
        self.check_interval_ = 5.0
        self.prob_round_ = 4
        self.path_ =[]
        self.find_feasible_path_ = False
        self.max_single_len_ = 30.0
        # put start and goal node into trees
        start = Node()
        start.id_ = 0
        start.position_ = np.array(start_position)
        start.parent_ = -1
        self.start_tree_.append(start)
        goal = Node()
        goal.id_ = 0
        goal.parent_ = -1
        goal.position_ = np.array(goal_position) 
        self.goal_tree_.append(goal)
    
    # sample randomly to get rand node
    def sample(self):
        # selecting goal with prob is unnecessary 
        node_rand = Node()
        n_row, n_col = self.map_.shape
        postion_rand = np.array([random.randint(0, n_row - 1), \
                random.randint(0, n_col - 1)])
        node_rand.position_ = postion_rand.astype(int)
        return node_rand

    # calculate distance between two nodes
    def calcDistance(self, node1, node2):
        # Eucilid distance between two nodes
        return np.linalg.norm(node1.position_ - node2.position_)

    # find nestest node according to distance
    def findNearestNode(self, node, node_list):
        min_dist = float('inf')
        node_nearest = Node()
        for tmp_node in node_list:
            dist = self.calcDistance(tmp_node, node)
            if dist < min_dist:
                min_dist = dist
                node_nearest = tmp_node
        if (node_nearest.id_ == -1):
            print("bug! cannot find nearest node!")
        return node_nearest

    # steer from node_nearest to node_rand
    def steer(self, node_nearest, node_rand):
        node_new = Node()
        node_new.parent_ = node_nearest.id_ 
        direction = node_rand.position_ - node_nearest.position_ 
        length = np.linalg.norm(direction)
        # if length is less than max_single_len, just return rand node 
        if (length <= self.max_single_len_):
            # print("length less than max length")
            node_new.position_ = node_rand.position_
            return node_new
        # trim node to max_single_len
        direction = direction / length
        position = node_nearest.position_ + self.max_single_len_ * direction
        node_new.position_ = position.astype(int)
        return node_new 

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

    # add new node into rrt
    def addNewNode(self, node_new, node_list):
        # verify if node is valid
        if (node_new.parent_ < 0 or node_new.parent_ >= len(node_list)):
            print("node parent is ", node_new.parent_, ", node is invalid!, reject to add into tree")
            return False  
        # add into rrt
        node_new.id_ = len(node_list)
        node_list.append(node_new)
        node_list[node_new.parent_].chidren_.append(node_new.id_)
        return True 

    # build Rapidly random exploring tree 
    def buildTree(self):
        print("start build rapidly random exploring tree...")
        for i in range(self.max_iter_):
            print(i, "th iteration")
            node_rand = self.sample()
            node_nearest = self.findNearestNode(node_rand, self.start_tree_)
            node_new = self.steer(node_nearest, node_rand)
            if self.checkEdgeInFreespace(node_nearest.position_, node_new.position_) \
                    == True:
                self.addNewNode(node_new, self.start_tree_)
                node_nearest_1 = self.findNearestNode(node_new, self.goal_tree_)
                node_new_1 = self.steer(node_nearest_1, node_new) 
                if (self.checkEdgeInFreespace(node_new_1.position_, node_nearest_1.position_)) == True:
                    self.addNewNode(node_new_1, self.goal_tree_)
                    while (node_new != node_new_1):
                        node_new_2 = self.steer(node_new_1, node_new)
                        if (self.checkEdgeInFreespace(node_new.position_, node_new_2.position_)) == True:
                            self.addNewNode(node_new_2, self.goal_tree_)
                            node_new_1 = node_new_2 
                        else:
                            break
                    if node_new == node_new_1:
                        self.find_feasible_path_ = True 
                        print("RRT find path from start to goal")
                        return
            if (len(self.start_tree_) > len(self.goal_tree_)):
                self.start_tree_, self.goal_tree_ = self.goal_tree_, self.start_tree_
        print("rapidly random exploring tree built!!!")

    # find path from start point to goal point
    def findPath(self):
        if self.find_feasible_path_ == False: 
            print("no feasible path found, maybe need more iterations")
            return 
        # put index of start tree into path 
        self.path_.append(len(self.start_tree_) - 1)
        parent_id = self.start_tree_[-1].parent_ 
        while parent_id >= 0 and parent_id < len(self.start_tree_):
            self.path_.append(parent_id)
            parent_id = self.start_tree_[parent_id].parent_ 
        # put root index into feasible path
        self.path_.reverse()
        # put index of goal tree into path
        self.path_.append(len(self.goal_tree_) - 1)
        parent_id = self.goal_tree_[-1].parent_
        while parent_id >= 0 and parent_id < len(self.goal_tree_):
            self.path_.append(parent_id)
            parent_id = self.goal_tree_[parent_id].parent_ 
        

    # draw Rapidly random exploring tree
    def drawRRT(self):
        # draw map 
        plt.ion()
        plt.matshow(self.map_, cmap = "gray_r")
        # draw start and goal point
        plt.scatter(self.start_position_[1], self.start_position_[0], marker = 'o')
        plt.scatter(self.goal_position_[1], self.goal_position_[0], marker = 'o')
        # draw RRT 
        # draw start tree 
        for node in self.start_tree_:
            for child in node.chidren_:
                if child < 0 or child >= len(self.start_tree_):
                    # TODO raise error to show bug
                    print("child < 0 or child > len(nodes_)")
                    return 
                x = [node.position_[0], self.start_tree_[child].position_[0]]
                y = [node.position_[1], self.start_tree_[child].position_[1]]
                plt.pause(0.002)
                plt.plot(y, x, color = 'r')
                plt.draw()
        # draw goal tree
        for node in self.goal_tree_:
            for child in node.chidren_:
                if child < 0 or child >= len(self.goal_tree_):
                    # TODO raise error to show bug
                    print("child < 0 or child > len(nodes_)")
                    return 
                x = [node.position_[0], self.goal_tree_[child].position_[0]]
                y = [node.position_[1], self.goal_tree_[child].position_[1]]
                plt.pause(0.002)
                plt.plot(y, x, color = 'r')
                plt.draw()

        # draw feasible path
        # self.findPath()
        if self.find_feasible_path_ == True:
            child_id = len(self.start_tree_) - 1
            parent_id = self.start_tree_[child_id].parent_
            while parent_id != -1:
                x = [self.start_tree_[parent_id].position_[0], \
                        self.start_tree_[child_id].position_[0]]
                y = [self.start_tree_[parent_id].position_[1], \
                        self.start_tree_[child_id].position_[1]]
                child_id = parent_id
                parent_id = self.start_tree_[child_id].parent_
                plt.pause(0.02)
                plt.plot(y, x, color = 'k')
            child_id = len(self.goal_tree_) - 1
            parent_id = self.goal_tree_[child_id].parent_
            while parent_id != -1:
                x = [self.goal_tree_[parent_id].position_[0], \
                        self.goal_tree_[child_id].position_[0]]
                y = [self.goal_tree_[parent_id].position_[1], \
                        self.goal_tree_[child_id].position_[1]]
                child_id = parent_id
                parent_id = self.goal_tree_[child_id].parent_
                plt.pause(0.02)
                plt.plot(y, x, color = 'k')
        else:
            print("No feasible path found, maybe need more iteration.")
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
    # start_node = Node()
    # start_node.position_ = np.array(start_position)
    # start_position_2 = (74, 80)
    # start_node_2 = Node()
    # start_node_2.position_ = np.array(start_position_2)
    # if (start_node != start_node_2):
        # print("start node is not equal start node 2")
    # else:
        # print("start node is equal start node 2")

    rrt_connect = RRT_Connect(data, start_position, goal_position)
    rrt_connect.buildTree()
    rrt_connect.drawRRT()
