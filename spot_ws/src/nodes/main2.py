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

        bag_list = load_dataset(load_pickle=True)

        # SELECT BAG FILE e.g.: bag_list[0], bag_list[1]...    
        torch_bags = dataloader(8, bag_list[1], 34, 6, "cpu", 1)
        
        model = ImitationPlanner(34, 6).to("cpu").eval()
        model.load_state_dict(torch.load('/home/gabuntu/catkin_ws/src/spot_ws/src/data/planner.model'))

        curr_output = []
        for i, curbag in enumerate(torch_bags):
            for batch_idx, batch in enumerate(curbag):

                cur_batch = batch[0].to("cpu")
                out = model(cur_batch)[0]
                curr_output.append(out)

        index = 0
        while not rospy.is_shutdown():

            ##output
            spot_twist = Twist()
            spot_twist.linear.x = curr_output[index][0].item()
            spot_twist.linear.y = curr_output[index][1].item()
            spot_twist.linear.z = curr_output[index][2].item()
            spot_twist.angular.x = curr_output[index][3].item()
            spot_twist.angular.y = curr_output[index][4].item()
            spot_twist.angular.z = curr_output[index][5].item()

            index += 1

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
    

def match_arrays(odom_time_list, pcl_time_list, obs_list):

    obs_list_new = []

    pcl_idx = 0

    for i,odom_time in enumerate(odom_time_list):
        if obs_list[pcl_idx] == None:
            obs_list[pcl_idx] = []
        if odom_time < pcl_time_list[pcl_idx]:
            obs_list_new.append(obs_list[pcl_idx])
        else:
            pcl_idx += 1
            
            if pcl_idx > len(obs_list)-1:
                pcl_idx = -1
                
            obs_list_new.append(obs_list[pcl_idx])


    return obs_list_new

def dataloader(max_obs, bag, input_size, output_size, device, batchsize):
    dataset = []

    name = bag.name
    odom_time_list = bag.odom_time
    pos_list = bag.pos.T
    ori_list = bag.ori.T
    twist_lin_list = bag.twist_lin.T
    twist_angular_list = bag.twist_angular.T
    pcl_time_list = bag.pcl_time
    obs_list = bag.obs_list
    goal = bag.goal

    new_obs_list = match_arrays(odom_time_list, pcl_time_list, obs_list)
    
    curbag_input = np.zeros((len(new_obs_list), input_size))
    curbag_output = np.zeros((len(new_obs_list), output_size))
    
    for i, obs in enumerate(new_obs_list):
        curpos = pos_list[i].tolist()
        ori = ori_list[i].tolist()
        twist_lin = twist_lin_list[i].tolist()
        twist_angular = twist_angular_list[i].tolist()
        goal = pos_list[-1].tolist()

        if obs == None:
            obs = []

        num_empty = max_obs -len(obs)

        for j in range(num_empty):
            obs.append([-1.0,-1.0,-1.0])

        new_obs = np.array(obs).flatten().tolist()
        cur_input = curpos + ori + goal + new_obs
        cur_input = np.array(cur_input).astype(np.float32)
        
        cur_output = np.array(twist_lin + twist_angular)
        
        curbag_input[i] = cur_input.astype(np.float32)
        curbag_output[i] = cur_output.astype(np.float32)
        
    curbag_input = torch.from_numpy(curbag_input).float().to(device)
    curbag_output = torch.from_numpy(curbag_output).float().to(device)
    curbag = TensorDataset(curbag_input, curbag_output)
    curbag = DataLoader(curbag, batch_size=batchsize, shuffle=True)
    
    dataset.append(curbag)

    return dataset




    
sim = Simulation()
sim.run()
