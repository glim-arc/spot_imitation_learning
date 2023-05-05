#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.srv import GetModelState
import torch
from torch import nn
import numpy as np


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
        M = Models()

        model = ImitationPlanner(34, 6).to("cpu").eval()

        model.load_state_dict(torch.load('/home/gabuntu/catkin_ws/src/spot_ws/src/data/planner.model'))

        #MANUALLY SET DESIRED GOAL
        goal = [8, 4, 0.34]


        while not rospy.is_shutdown():
            ##input
            M.show_gazebo_models()
            print(M.spotPose)
            # human = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
            cur_input = [M.spotPose + M.spotOri + goal + M.humanPoses]
            cur_input = np.array(cur_input).astype(np.float32)
            cur_input = torch.from_numpy(cur_input).float().to("cpu")

            ##test model
            curr_output = model(cur_input)


            ##output
            spot_twist = Twist()
            spot_twist.linear.x = curr_output[0][0].item()
            spot_twist.linear.y = curr_output[0][0].item()
            spot_twist.linear.z = 0
            spot_twist.angular.x = 0
            spot_twist.angular.y = 0
            spot_twist.angular.z = curr_output[0][5].item()

            # Close to goal
            e = ((M.spotPose[0] - goal[0])**2 + (M.spotPose[1] - goal[1])**2) / 2
            if e < 0.5:
                print("Destination arrived")
                spot_twist.linear.x = 0
                spot_twist.linear.y = 0
                spot_twist.linear.z = 0
                spot_twist.angular.x = 0
                spot_twist.angular.y = 0
                spot_twist.angular.z = 0

            self.cmd_vel_pub.publish(spot_twist)
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
                    x = resp_coordinates.pose.position.x
                    y = resp_coordinates.pose.position.y
                    z = resp_coordinates.pose.position.z
                    if (abs(x) < 5) and (abs(y) < 5):
                        self.humanPoses += [x, y, z] 
                    else:
                        self.humanPoses += [-1, -1, -1]
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
