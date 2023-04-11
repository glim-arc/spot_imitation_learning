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
            obs_list[pcl_idx] = []
        if odom_time < pcl_time_list[pcl_idx]:
            obs_list_new.append(obs_list[pcl_idx])
        else:
            pcl_idx += 1
            
            if pcl_idx > len(obs_list)-1:
                pcl_idx = -1
                
            obs_list_new.append(obs_list[pcl_idx])


    return obs_list_new

def dataloader(bag_list, input_size, output_size, obstacle_size, device, batchsize):
    dataset = []
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
            twist_lin = twist_lin_list[i].tolist()
            twist_angular = twist_angular_list[i].tolist()
            goal = pos_list[-1].tolist()

            if obs == None:
                obs = []

            num_empty = 8-len(obs)

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


def train():
    obstacle_size = 8 * 3 
    input_size = obstacle_size + 6 + 4 #obstacles in environment and start, goal, ori
    output_size = 6 #twist (linear and angular velocity)
    batchsize = 30
    num_epochs =1000

    #initialize policy neural network
    device = "cpu"
    planner = ImitationPlanner(input_size, output_size).to(device)

    #load data
    bag_list = load_dataset(load_pickle=True)
    
    optimizer = torch.optim.Adam(planner.parameters())
    
    torch_bags = dataloader(bag_list, input_size, output_size, obstacle_size, device, batchsize)
    
    mse_loss = nn.MSELoss()
    avg_loss_list = []

    print("env loaded")
    
    for epoch in range(num_epochs):
        print ("epoch" + str(epoch))
        avg_loss=0
        print("training starts")

        counter = 0

        for i, curbag in enumerate(torch_bags):
            for batch_idx, batch in enumerate(curbag):
                optimizer.zero_grad()
                planner.zero_grad()
                cur_batch = batch[0].to(device)
                cur_real_output = batch[1].to(device)

                # ===================forward=====================
                cur_planner_output = planner(cur_batch)
                loss = mse_loss(cur_planner_output, cur_real_output)
                avg_loss = avg_loss + loss.data
                # ===================backward====================
                loss.backward()
                optimizer.step()

                counter += 1

        print ("--average loss:")
        avg_loss = avg_loss/counter
        print (avg_loss)
        avg_loss_list.append(avg_loss.cpu().numpy())
        
        
def planner():
    pos,ori,twist_lin, goal = None
    
    return pos,ori,twist_lin, goal
            
train()