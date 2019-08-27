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
        self.distance_threshold_ = 3.0

    def __eq__(self, rhs):
        if np.linalg.norm(self.position_ - rhs.position_) <=self.distance_threshold_:
            return True
        else:
            return False

    def __ne__(self, rhs):
        if self.__eq__(rhs) == True:
            return False
        else:
            return True

    def printInfo(self):
        print("ID: ", self.id_)
        print("position : ", self.position_)
        print("parent id : ", self.parent_)
        print("chidren id : ", self.chidren_)


if __name__ == '__main__':
    pass
