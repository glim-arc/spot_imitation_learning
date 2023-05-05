#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
import torch
from torch import nn
import numpy as np
from RRT import *



class Simulation:
    def __init__(self, node_name="Move"):
        self.node_name = node_name
        print("Starting node Move")

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

    def run(self):
        M = Models()

        #Load NN model
        model = ImitationPlanner(31, 3).to("cpu").eval()
        model.load_state_dict(torch.load('/home/gabuntu/catkin_ws/src/spot_ws/src/data/planner.model'))

        #RRT sets local goal
        start_pose = [0,0]
        goal_pose = [8, -4]
        print("calculating path...")
        #path_conf = RRT(start_pose, goal_pose)
        path_conf = [[0,0,0], [-1,0,0], [-2,0,0]]

        if path_conf is None:
            input("no collision-free path is found within the time budget, finish?")

        index = 0
        pose_x = 0
        pose_y = 0
        delta_time = 0.06
        while not rospy.is_shutdown():
            ##input
            if index == len(path_conf):
                index -= 1

            M.show_gazebo_models()
            dist = [M.ballPose[0]-path_conf[index][0],M.ballPose[1]-path_conf[index][1], 0]

            cur_input = [dist + M.ballOri + M.humanPoses]
            cur_input = np.array(cur_input).astype(np.float32)
            cur_input = torch.from_numpy(cur_input).float().to("cpu")

            ##test model
            curr_output = model(cur_input)

            ##output
            robotpose_msg = ModelState()
            robotpose_msg.model_name = 'cricket_ball'
            pose_x += curr_output[0][0].item()*delta_time
            pose_y += curr_output[0][1].item()*delta_time
            robotpose_msg.pose.position.x = pose_x
            robotpose_msg.pose.position.y = pose_y
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state(robotpose_msg)


            # Close to goal
            e = ((M.ballPose[0] - goal_pose[0])**2 + (M.ballPose[1] - goal_pose[1])**2) / 2
            if e < 0.5:
                print("Destination arrived")

            self.rate.sleep() 
            index += 1

        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.node_name))


class Models:

    def __init__(self):
        self.numHumans = 8
        self.nameHumans = []
        for i in range (1, self.numHumans+1):
            self.nameHumans.append("actor"+str(i))

        self.humanPoses = []
        self.ballPose = []
        self.ballOri = []

    def show_gazebo_models(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

            ball_coord = model_coordinates('cricket_ball', '')
            self.ballPose = [ball_coord.pose.position.x, ball_coord.pose.position.y] 
            self.ballOri = [ball_coord.pose.orientation.x, ball_coord.pose.orientation.y, ball_coord.pose.orientation.z, ball_coord.pose.orientation.w]
            
            self.humanPoses = []
            for i in range(self.numHumans):
                resp_coordinates = model_coordinates(self.nameHumans[i], '')
                if resp_coordinates.success:
                    x = resp_coordinates.pose.position.x
                    y = resp_coordinates.pose.position.y
                    z = resp_coordinates.pose.position.z

                    dif_x = x - self.ballPose[0]
                    dif_y = y - self.ballPose[1] 
                    if (abs(dif_x) < 5) and (abs(dif_y) < 5):
                        self.humanPoses += [dif_x, dif_y, z] 
                    else:
                        self.humanPoses += [-1, -1, -1]
                else:
                    self.humanPoses += [-1, -1, -1]


        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


class ImitationPlanner(nn.Module):
    def __init__(self, input_size, output_size):
        super().__init__()
        p = 0.4
        self.fc = nn.Sequential(
            nn.Linear(input_size, 48),nn.PReLU(), nn.Dropout(p=p),nn.BatchNorm1d(48),
            nn.Linear(48, 32),nn.PReLU(), nn.Dropout(p=p),nn.BatchNorm1d(32),
            nn.Linear(32, output_size))

        
    def forward(self, x):
        out = self.fc(x)
        return out
    
    
sim = Simulation()
sim.run()
