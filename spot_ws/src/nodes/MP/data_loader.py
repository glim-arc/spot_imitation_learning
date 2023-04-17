import numpy as np
import os
import os.path
import csv
import pandas as pd
import pickle

class Bag:
    def __init__(self,name, odom_time, pos, ori, twist_lin, twist_angular,pcl_time,obs_list):
        self.name = name
        self.odom_time = odom_time
        self.pos = pos
        self.ori = ori
        self.twist_lin = twist_lin
        self.twist_angular = twist_angular
        self.pcl_time = pcl_time
        self.obs_list = obs_list #list
        self.goal = pos[-1]


def load_dataset(load_pickle):
    current_path = os.getcwd()
    data_path = os.path.join(current_path, "MP")
    # bag_path = os.path.join(data_path, "bag")
    # bagfilelist = os.listdir(bag_path)
    temp = []

    if load_pickle:
        pickle_path = os.path.join(data_path, "bag_list_7.pickle")
        with open(pickle_path, 'rb') as f:
            bag_list = pickle.load(f)
            print("parsed bag loaded")
            return bag_list


#load_obs_from_lidar("./data/bag/A_Spot_Jester_Jester_Wed_Nov_10_64/velodyne_points")
#load_dataset(False)