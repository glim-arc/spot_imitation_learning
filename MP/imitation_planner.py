# load libraries

import torch
from torch import nn
import os
import sys
sys.path.append(os.getcwd())
from MP.data_loader import load_dataset, Bag
import numpy as np
from torch.utils.data import TensorDataset
from torch.utils.data import DataLoader

class ImitationPlanner(nn.Module):
    def __init__(self, input_size, output_size):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_size, 1280),nn.PReLU(),nn.Dropout(),
            nn.Linear(1280, 1024),nn.PReLU(),nn.Dropout(),
            nn.Linear(1024, 896),nn.PReLU(),nn.Dropout(),
            nn.Linear(896, 768),nn.PReLU(),nn.Dropout(),
            nn.Linear(768, 512),nn.PReLU(),nn.Dropout(),
            nn.Linear(512, 384),nn.PReLU(),nn.Dropout(),
            nn.Linear(384, 256),nn.PReLU(), nn.Dropout(),
            nn.Linear(256, 256),nn.PReLU(), nn.Dropout(),
            nn.Linear(256, 128),nn.PReLU(), nn.Dropout(),
            nn.Linear(128, 64),nn.PReLU(), nn.Dropout(),
            nn.Linear(64, 32),nn.PReLU(),
            nn.Linear(32, output_size))

    def forward(self, x):
        out = self.fc(x)
        return out
    
def match_arrays(odom_time_list, pcl_time_list, obs_list):

    obs_list_new = []

    pcl_idx = 0

    for i,odom_time in enumerate(odom_time_list):
        if obs_list[pcl_idx] == None:
            obs_list[pcl_idx] = [-1,-1,-1]
        if odom_time < pcl_time_list[pcl_idx]:
            obs_list_new.append(obs_list[pcl_idx])
        else:
            pcl_idx += 1
            
            if pcl_idx > len(obs_list)-1:
                pcl_idx = -1
                
            obs_list_new.append(obs_list[pcl_idx])


    return obs_list_new

def dataloader(bag_list, input_size, output_size, obstacle_size, device, batchsize):
    input_data = []
    real_output = []
    for bag in bag_list:
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
            twist_lin = twist_lin_list[i]
            twist_angular = twist_angular_list[i]
            goal = pos_list[-1].tolist()
            
            num_empty = 8-len(obs)
            
            for j in range(num_empty):
                obs.append([-1,-1,-1])
                
            cur_input = curpos + ori + goal + obs
            cur_input = np.array(cur_input)
            
            cur_output = np.concatenate((twist_lin,twist_angular),axis=0)
            
            curbag_input[i] = cur_input
            curbag_output[i] = cur_output
            
        curbag_input = torch.from_numpy(curbag_input).to(device)
        curbag_output = torch.from_numpy(curbag_output).to(device)
        curbag_input = TensorDataset(curbag_input)
        curbag_input = DataLoader(curbag_input, batch_size=batchsize, shuffle=True)
        
        input_data.append(curbag_input)
        real_output.append(real_output)
            
    return input_data, real_output


def train():
    obstacle_size = 8 * 3 
    input_size = obstacle_size + 6 + 4 #obstacles in environment and start, goal, ori
    output_size = 7 #twist (linear and angular velocity)
    batchsize = 30
    num_epochs =1000

    #initialize policy neural network
    device = "cpu"
    planner = ImitationPlanner(input_size, output_size).to(device)

    #load data
    bag_list = load_dataset(load_pickle=True)
    
    optimizer = torch.optim.Adam(planner.parameters())
    
    torch_bags, real_outputs = dataloader(bag_list, input_size, output_size, obstacle_size, device, batchsize)
    
    mse_loss = nn.MSELoss()
    avg_loss_list = []

    print("env loaded")
    
    for epoch in range(num_epochs):
            print ("epoch" + str(epoch))
            avg_loss=0
            print("training starts")
            
            for i, curbag in enumerate(torch_bags):
                for batch_idx, batch in enumerate(curbag):
                    optimizer.zero_grad()
                    planner.zero_grad()
                    cur_batch = batch[0].to(device)
                    cur_real_output = real_outputs[batch_idx]

                    # ===================forward=====================
                    cur_planner_output = planner(batch)
                    loss = mse_loss(cur_planner_output, cur_batch)
                    avg_loss = avg_loss + loss.data
                    # ===================backward====================
                    loss.backward()
                    optimizer.step()

            print ("--average loss:")
            avg_loss = avg_loss/batchsize/len(torch_bags)
            print (avg_loss)
            avg_loss_list.append(avg_loss.cpu().numpy())
        
        
def planner():
    pos,ori,twist_lin, goal = None
    
    return pos,ori,twist_lin, goal
            
train()