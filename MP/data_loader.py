import numpy as np
import os
import os.path
import csv
import pandas as pd
from data.lidar_parser import parse_lidar

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
def load_obs_from_lidar(path):
    pcl_list = os.listdir(path)
    time = [float(curpcl[:-4]) for curpcl in pcl_list]
    time.sort()
    time = np.array(time)

    obs_list = []

    print("Start pcl parsing")
    for curpcl in time:
        print(curpcl)
        curpclpath = os.path.join(path, str(curpcl)+".pcd")
        cur_obs = parse_lidar(curpclpath)
        if len(cur_obs) == 1:
            if cur_obs[0][0] == 0 and cur_obs[0][1] == 0 and cur_obs[0][2] == 0:
                cur_obs = None
        obs_list.append(cur_obs)
    print("End pcl parsing")

    return time, obs_list

def load_dynamical_inputs(path):
    df = pd.read_csv(path)

    df.drop(df.columns[1:6], axis=1, inplace=True)
    df.drop(["pose.covariance", "twist.covariance"], axis=1, inplace=True)

    time = np.array(df['Time'])
    pos = np.array([df['pose.pose.position.x'],df['pose.pose.position.y'],df['pose.pose.position.z']])
    ori = np.array([df['pose.pose.orientation.x'],df['pose.pose.orientation.y'],df['pose.pose.orientation.z']])
    twist_lin = np.array([df['twist.twist.linear.x'],df['twist.twist.linear.y'],df['twist.twist.linear.z']])
    twist_angular = np.array([df['twist.twist.linear.x'],df['twist.twist.linear.y'],df['twist.twist.linear.z']])

    print("odom loaded")

    return time, pos, ori, twist_lin, twist_angular

def load_dataset():
    current_path = os.getcwd()
    current_path = os.path.join(current_path, "data/bag")
    bagfilelist = os.listdir(current_path)

    for i,bagfile in enumerate(bagfilelist):
        if os.path.isfile(os.path.join(current_path, bagfile)):
            bagfilelist.pop(i)

    bag_list = []

    for bagname in bagfilelist:
        print(bagname, "parsing start")
        cur_dir = os.path.join(current_path, bagname)
        odom_path = os.path.join(cur_dir, "odom.csv")
        pcl_path = os.path.join(cur_dir, "velodyne_points")
        odom_time, pos, ori, twist_lin, twist_angular =  load_dynamical_inputs(odom_path)
        pcl_time,obs_list = load_obs_from_lidar(pcl_path)
        bag_list.append(Bag(bagname,odom_time, pos, ori, twist_lin, twist_angular,pcl_time,obs_list))

    return bag_list

#load_obs_from_lidar("./data/bag/A_Spot_Jester_Jester_Wed_Nov_10_64/velodyne_points")
load_dataset()