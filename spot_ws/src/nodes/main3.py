#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.srv import GetModelState
import torch
from torch import nn
import numpy as np
from MP.data_loader import load_dataset, Bag
from torch.utils.data import TensorDataset
from torch.utils.data import DataLoader
import csv

class Simulation:
    def __init__(self, node_name="Move"):
        self.node_name = node_name
        print("Starting node Move")

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        # Publishes to robot twist
        self.cmd_vel_pub = rospy.Publisher('/spot1/cmd_vel', Twist, queue_size=1)  

    def pose_callback(self, data):
        self.currentPose = data
        rospy.loginfo('current_pose:{:.4f}'.format(self.currentPose.pose.position))

    def run(self):
        
        model = ImitationPlanner(34, 6).to("cpu").eval()
        model.load_state_dict(torch.load('/home/gabuntu/catkin_ws/src/spot_ws/src/data/planner.model'))

        curr_output = []
        with open("./data/odom1.csv", 'r') as file:
            csvreader = csv.reader(file)
            for row in csvreader:

                newTwist = Twist()
                newTwist.linear.x = float(row[7])
                newTwist.linear.y = float(row[8])
                newTwist.linear.z = float(row[9])
                newTwist.angular.x = float(row[10])
                newTwist.angular.y = float(row[11])
                newTwist.angular.z = float(row[12])
                curr_output.append(newTwist)


        index = 0
        while not rospy.is_shutdown():

            index += 1


            self.cmd_vel_pub.publish(curr_output[index])
            self.rate.sleep() 

        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.node_name))



class Models:

    def __init__(self):
        self.numHumans = 8
        self.nameHumans = []
        for i in range (1, self.numHumans+1):
            self.nameHumans.append("actor"+str(i))

        self.humanPoses = []
        self.spotPose = []
        self.spotOri = []

    def show_gazebo_models(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            self.humanPoses = []
            for i in range(self.numHumans):
                resp_coordinates = model_coordinates(self.nameHumans[i], '')
                if resp_coordinates.success:
                    self.humanPoses += [resp_coordinates.pose.position.x, resp_coordinates.pose.position.y, resp_coordinates.pose.position.z] 
                else:
                    self.humanPoses += [-1, -1, -1]

            spot_coord = model_coordinates('spot1', '')
            self.spotPose = [spot_coord.pose.position.x, spot_coord.pose.position.y, spot_coord.pose.position.z] 
            self.spotOri = [spot_coord.pose.orientation.x, spot_coord.pose.orientation.y, spot_coord.pose.orientation.z, spot_coord.pose.orientation.w] 

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


class ImitationPlanner(nn.Module):
    def __init__(self, input_size, output_size):
        super().__init__()
        p = 0.2
        self.fc = nn.Sequential(
            nn.Linear(input_size, 1280),nn.PReLU(),nn.Dropout(p=p),
            nn.BatchNorm1d(1280),
            nn.Linear(1280, 1024),nn.PReLU(),nn.Dropout(p=p),
            nn.BatchNorm1d(1024),
            nn.Linear(1024, 896),nn.PReLU(),nn.Dropout(p=p),
            nn.BatchNorm1d(896),
            nn.Linear(896, 768),nn.PReLU(),nn.Dropout(p=p),
            nn.BatchNorm1d(768),
            nn.Linear(768, 512),nn.PReLU(),nn.Dropout(p=p),
            nn.BatchNorm1d(512),
            nn.Linear(512, 384),nn.PReLU(),nn.Dropout(p=p),
            nn.BatchNorm1d(384),
            nn.Linear(384, 256),nn.PReLU(), nn.Dropout(p=p),
            nn.BatchNorm1d(256),
            nn.Linear(256, 128),nn.PReLU(), nn.Dropout(p=p),
            nn.BatchNorm1d(128),
            nn.Linear(128, 64),nn.PReLU(), nn.Dropout(p=p),
            nn.BatchNorm1d(64),
            nn.Linear(64, 32),nn.PReLU(), nn.Dropout(p=p),
            nn.Linear(32, output_size))

    def forward(self, x):
        out = self.fc(x)
        return out
    

    
sim = Simulation()
sim.run()
