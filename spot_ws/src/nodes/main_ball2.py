#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
import pickle
import os
import argparse
import numpy as np
import torch
from torch import nn



class Simulation:
    def __init__(self, node_name="Move"):
        self.node_name = node_name
        print("Starting node Move")

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        #DIRECTORIES INFO
        pickle_file = args.pickle_file

        current_path = os.getcwd()
        data_path = os.path.join(current_path, "data")

        #GET OBSTACLES
        obs_dir = data_path + "/validation_data/obs/" + str(pickle_file) +".pickle"
        with open(obs_dir, 'rb') as f:
            self.obs_list = pickle.load(f)
        #GET GOALS
        obs_dir = data_path +  "/validation_data/" + str(pickle_file) +"_goal.pickle"
        with open(obs_dir, 'rb') as f:
            self.localGoal_list = pickle.load(f)
        #GET real path
        real_dir = data_path +  "/validation_data/" + str(pickle_file) +".pickle"
        with open(real_dir, 'rb') as f:
            self.real_path_list = pickle.load(f)



    def run(self):
        M = Models()
         
        obs_list_index = 0
        obs_list = self.obs_list

        model = ImitationPlanner(31, 3).to("cpu").eval()
        model.load_state_dict(torch.load('/home/gabuntu/catkin_ws/src/spot_ws/src/data/planner.model'))

        pose_x = 0
        pose_y = 0
        delta_time = 0.06

        #initialize ball position
        robotpose_msg = ModelState()
        robotpose_msg.model_name = 'cricket_ball'
        robotpose_msg.pose.position.x = pose_x
        robotpose_msg.pose.position.y = pose_y
        M.move_box(robotpose_msg)

        #get real path
        pos_list = self.real_path_list[1]
        start_pose = [pos_list[0][0], pos_list[0][1]]
        real_path = []
        for pos in pos_list:
            newPose = [pos[0] - start_pose[0], pos[1] - start_pose[1]]
            real_path.append(newPose)



        while not rospy.is_shutdown():
            M.show_gazebo_models()

            obs_list_mod = []
            

            if obs_list[obs_list_index] != None:
                lenList = len(obs_list[obs_list_index])
            else:
                lenList = 0

            for i in range (lenList):
                box_msg = ModelState()
                box_msg.model_name = 'unit_box_' + str(i)
                box_msg.pose.position.x = obs_list[obs_list_index][i][0] + real_path[obs_list_index][0]
                box_msg.pose.position.y = obs_list[obs_list_index][i][1] + real_path[obs_list_index][1]
                box_msg.pose.position.z = 0
                M.move_box(box_msg)
                obs_list_mod += [obs_list[obs_list_index][i][0] + real_path[obs_list_index][0], obs_list[obs_list_index][i][1] + real_path[obs_list_index][1], 0]

            for i in range (lenList, 8):
                box_msg = ModelState()
                box_msg.model_name = 'unit_box_' + str(i)
                box_msg.pose.position.x = 100
                box_msg.pose.position.y = 100
                box_msg.pose.position.z = 100
                M.move_box(box_msg)
                obs_list_mod += [-1, -1, -1]


            M.show_gazebo_models()
            dist = [M.ballPose[0]-self.localGoal_list[obs_list_index][0],M.ballPose[1]-self.localGoal_list[obs_list_index][1], 0]
            # if obs_list[obs_list_index] == [] or obs_list[obs_list_index] == None:
            #     obs_list[obs_list_index] = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
            # else:
            #     mod_obs = []
            #     for obs in obs_list[obs_list_index]:
            #         mod_obs += obs
            #     for i in range (len(obs_list[obs_list_index]), 8):
            #         mod_obs += [-1, -1, -1]
            #     obs_list[obs_list_index] = mod_obs


            cur_input = [dist + M.ballOri + obs_list_mod]
            

            cur_input = np.array(cur_input).astype(np.float32)
            cur_input = torch.from_numpy(cur_input).float().to("cpu")

            ##test model
            curr_output = model(cur_input)

            ##output
            pose_x += curr_output[0][0].item()*delta_time
            pose_y += curr_output[0][1].item()*delta_time
            robotpose_msg = ModelState()
            robotpose_msg.model_name = 'cricket_ball'
            robotpose_msg.pose.position.x = pose_x
            robotpose_msg.pose.position.y = pose_y
            M.move_box(robotpose_msg)

            self.rate.sleep() 

            if obs_list_index == len(obs_list) - 1:
                print("Bag finished")
               
            else:
                obs_list_index += 1


            self.rate.sleep() 

        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.node_name))


class Models:

    def __init__(self):
        self.ballPose = []
        self.ballOri = []

    def move_box(self, msg):
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state(msg)

    def show_gazebo_models(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            robot_coord = model_coordinates('cricket_ball', '')
            self.ballPose = [robot_coord.pose.position.x, robot_coord.pose.position.y, robot_coord.pose.position.z] 
            self.ballOri = [robot_coord.pose.orientation.x, robot_coord.pose.orientation.y, robot_coord.pose.orientation.z, robot_coord.pose.orientation.w]

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


    
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()

	# mpnet training data generation
    parser.add_argument('--pickle_file', type=int, default=5)
    args = parser.parse_args()
    
    sim = Simulation()
    sim.run()

