# load libraries

import torch
from torch import nn
import os
import sys
sys.path.append(os.getcwd())
print(os.getcwd())
from MP.data_loader import load_dataset, Bag
import numpy as np
from torch.utils.data import TensorDataset
from torch.utils.data import DataLoader
import argparse

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

def dataloader(max_obs, bag_list, input_size, output_size, device, batchsize):
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
            
            time = i + 20
            
            if time >= len(pos_list):
                time = len(pos_list) -1
            
            goal = pos_list[time].tolist()
            
            #goal = pos_list[-1].tolist()

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


def train(args):
    obstacle_size = args.obstacle_size
    input_size = args.input_size
    output_size = args.output_size
    batchsize = args.batchsize
    num_epochs = args.num_epochs
    current_path = args.current_path
    data_path = args.data_path
    torch.manual_seed(args.seed)

    #initialize planner neural network
    device = args.device
    planner = ImitationPlanner(input_size, output_size).to(device)

    #load data
    bag_list = load_dataset(load_pickle=True)
    
    optimizer = torch.optim.Adam(planner.parameters(), lr=args.lr)
    
    torch_bags = dataloader(args.max_obs,bag_list, input_size, output_size, device, batchsize)
    
    mse_loss = nn.MSELoss()
    avg_loss_list = []

    print("env loaded")
    print("training starts")
    print("num bags", len(torch_bags))
    
    for epoch in range(num_epochs):
        print ("epoch" + str(epoch))
        avg_loss=0
        counter = 0

        for i, curbag in enumerate(torch_bags):
            for batch_idx, batch in enumerate(curbag):
                optimizer.zero_grad()
                planner.zero_grad()
                planner.train()
                cur_batch = batch[0].to(device)
                #print(cur_batch[0])
                cur_real_output = batch[1].to(device)
                # ===================forward=====================
                cur_planner_output = planner(cur_batch)
                loss = mse_loss(cur_planner_output, cur_real_output)
                avg_loss = avg_loss + loss.data
                # ===================backward====================
                loss.backward()
                optimizer.step()

                counter += 1

        avg_loss = avg_loss.cpu().numpy()/counter
        print ("--average loss:", avg_loss)
        avg_loss_list.append(avg_loss)
    
    torch.save(planner.state_dict(),os.path.join(data_path,'planner.model'))
    np.save(os.path.join(data_path,'avg_loss_list.npy'), avg_loss_list)
        
        
def planner(args, curpos, ori, goal, obs):
    device = args.device
    current_path = args.current_path
    data_path = args.data_path
    
    model = torch.load(os.path.join(data_path,'planner.model'))
    planner = ImitationPlanner(input_size, output_size).to(device).eval()

    curpos = curpos.tolist()
    ori = ori.tolist()
    goal = goal.tolist()
    obs = obs.tolist()
    
    cur_input = curpos + ori + goal + obs
    cur_input = np.array(cur_input).astype(np.float32)
    cur_input =  np.expand_dims(cur_input, axis=0)
    cur_input = torch.from_numpy(cur_input).float().to(device)
    print(cur_input.shape)
    
    cur_planner_output = planner(cur_input)[0]
    
    twist_lin = cur_planner_output[:3].cpu().detach().numpy()
    twist_angular = cur_planner_output[3:].cpu().detach().numpy()
    
    print(twist_lin, twist_angular)
    
    return twist_lin, twist_angular

if __name__ == '__main__':
    current_path = os.getcwd()
    data_path = os.path.join(current_path, "data")
    obstacle_size = 8 * 3 
    input_size = obstacle_size + 6 + 4 #obstacles in environment and start, goal, ori
    output_size = 6 #twist (linear and angular velocity)
    batchsize = 4000
    num_epochs = 4000
    lr=0.0001
    seed = 0
    
    parser = argparse.ArgumentParser()

	# mpnet training data generation
    parser.add_argument('--device', type=str, default="cuda")
    parser.add_argument('--current_path', type=str, default=current_path)
    parser.add_argument('--data_path', type=str, default=data_path)
    parser.add_argument('--obstacle_size', type=int, default=obstacle_size)
    parser.add_argument('--max_obs', type=int, default=8)
    parser.add_argument('--input_size', type=int, default=input_size)
    parser.add_argument('--output_size', type=int, default=output_size)
    parser.add_argument('--batchsize', type=int, default=batchsize)
    parser.add_argument('--num_epochs', type=int, default=num_epochs)
    parser.add_argument('--lr', type=int, default=lr)
    parser.add_argument('--seed', type=int, default=seed)
    
    args = parser.parse_args()
    
    #planner(args, np.zeros(3), np.zeros(4), np.zeros(3), np.zeros(24))
    train(args)