# load libraries

import torch
from torch import nn
import os
import sys
sys.path.append(os.getcwd())
sys.path.append(os.path.join(os.getcwd(), "data"))
print(os.getcwd())
from MP.data_loader import load_dataset, Bag
from plot import plot
import numpy as np
from torch.utils.data import TensorDataset
from torch.utils.data import DataLoader
import argparse
import pickle

class ImitationPlanner(nn.Module):
    def __init__(self, input_size, output_size, p):
        super().__init__()
        self.fc = nn.Sequential(
            # nn.Linear(input_size, 1280),nn.PReLU(),nn.Dropout(p=p),
            # nn.BatchNorm1d(1280),
            # nn.Linear(1280, 1024),nn.PReLU(),nn.Dropout(p=p),
            # nn.BatchNorm1d(1024),
            # nn.Linear(1024, 896),nn.PReLU(),nn.Dropout(p=p),
            # nn.BatchNorm1d(896),
            # nn.Linear(896, 768),nn.PReLU(),nn.Dropout(p=p),
            # nn.BatchNorm1d(768),
            # nn.Linear(768, 512),nn.PReLU(),nn.Dropout(p=p),
            # nn.BatchNorm1d(512),
            # nn.Linear(512, 384),nn.PReLU(),nn.Dropout(p=p),
            # nn.BatchNorm1d(384),
            # nn.Linear(384, 256),nn.PReLU(), nn.Dropout(p=p),
            # nn.BatchNorm1d(256),
            # nn.Linear(256, 128),nn.PReLU(), nn.Dropout(p=p),
            # nn.BatchNorm1d(128),
            # nn.Linear(128, 64),nn.PReLU(), nn.Dropout(p=p),
            # nn.BatchNorm1d(64),
            nn.Linear(input_size, 48),nn.PReLU(), nn.Dropout(p=p),
            nn.BatchNorm1d(48),
            nn.Linear(48, 32),nn.PReLU(), nn.Dropout(p=p),
            nn.BatchNorm1d(32),
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

def dataloader(args, traj_path, max_obs, bag_list, input_size, output_size, device, batchsize):
    dataset = []
    names = []
    for k, bag in enumerate(bag_list):
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
        
        obs_dir = os.path.join(traj_path, "obs")
        
        if not os.path.isdir(obs_dir):
            os.makedirs(obs_dir, exist_ok=True)
        
        obs_dir = os.path.join(obs_dir, str(k)+".pickle")
        
        with open(obs_dir, 'wb') as f:
            pickle.dump(new_obs_list, f, pickle.HIGHEST_PROTOCOL)
        
        for i, obs in enumerate(new_obs_list):
            curpos = pos_list[i].tolist()
            ori = ori_list[i].tolist()
            twist_lin = twist_lin_list[i].tolist()
            twist_angular = twist_angular_list[i].tolist()
            
            time = i + 250
            
            if time >= len(pos_list):
                time = len(pos_list) -1
            
            goal = pos_list[time].tolist()
            
            #goal = pos_list[-1].tolist()

            if obs == None:
                obs = []

            num_empty = max_obs -len(obs)

            for j in range(num_empty):
                obs.append([-1.0,-1.0,-1.0])

            # new_obs = np.array(obs).flatten().tolist()
            # cur_input = curpos + ori + goal + new_obs
            # cur_input = np.array(cur_input).astype(np.float32)
            new_obs = np.array(obs).flatten().tolist()
            new_goal = np.array(goal) - np.array(curpos)
            new_goal = new_goal.tolist()
            cur_input = new_goal + ori + new_obs
            cur_input = np.array(cur_input).astype(np.float32)
            
            cur_output = np.array(twist_lin)
            
            
            curbag_input[i] = cur_input.astype(np.float32)
            curbag_output[i] = cur_output.astype(np.float32)
            
        curbag_input = torch.from_numpy(curbag_input).float().to(device)
        curbag_output = torch.from_numpy(curbag_output).float().to(device)
        curbag = TensorDataset(curbag_input, curbag_output)
        #curbag = DataLoader(curbag, batch_size=1, shuffle=False)
        
        save_bag = [name, pos_list, ori_list, twist_lin_list, twist_angular_list]
        
        new_path_dir = os.path.join(traj_path, str(k)+".pickle")
        
        with open(new_path_dir, 'wb') as f:
            pickle.dump(save_bag, f, pickle.HIGHEST_PROTOCOL)
        
        dataset.append(curbag)
        names.append(name)

    return dataset, names

def validation(args):
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
    device = "cpu"
    planner = ImitationPlanner(input_size, output_size, p=1).to(device)
    model_path = os.path.join(data_path,"planner.model")
    print(model_path)
    trained = torch.load(model_path, map_location=torch.device('cpu'))
    planner.load_state_dict(trained)

    #load data
    print("Trained Bags")
    bag_list = load_dataset(load_pickle=True, bag_path = args.bag_path)
    traj_path = os.path.join(args.data_path, "path_comparison/"+"training")
    if not os.path.isdir(traj_path):
        os.makedirs(traj_path, exist_ok=True)
    torch_bags, names = dataloader(args, traj_path, args.max_obs,bag_list, input_size, output_size, device, batchsize)
    mse_loss = nn.MSELoss()
    print("bag num", len(bag_list))
    
    avg_loss=[]

    for i, curbag in enumerate(torch_bags):
        curbag_loss = []
        curbag_output = []
        for batch_idx, batch in enumerate(curbag):
            planner.zero_grad()
            planner.eval()
            cur_batch = batch[0].to(device).unsqueeze(0)
            #print(cur_batch[0])
            cur_real_output = batch[1].to(device)
            # ===================forward=====================
            cur_planner_output = planner(cur_batch)[0]
            loss = mse_loss(cur_planner_output, cur_real_output)
            
            npout = cur_planner_output.detach().numpy()
            # print(npout)
            # print(batch[1])
            curbag_output.append(cur_planner_output.detach().numpy())
            curbag_loss.append(loss.cpu().detach().numpy())
        
        new_path_dir = os.path.join(traj_path, str(i)+"_gen.pickle")
        
        with open(new_path_dir, 'wb') as f:
            pickle.dump(curbag_output, f, pickle.HIGHEST_PROTOCOL)
        
        avg_loss.append(sum(curbag_loss)/len(curbag_loss))
        print(avg_loss[-1])
        print(curbag_output[-3:])

    avg_loss = sum(avg_loss)/len(avg_loss)

    val_loss = avg_loss
    print ("--validation loss:", val_loss)
    
    print("Untrained Bags")
    bag_list = load_dataset(load_pickle=True, bag_path = args.val_path)
    traj_path = os.path.join(args.data_path, "path_comparison/"+"validation")
    if not os.path.isdir(traj_path):
        os.makedirs(traj_path, exist_ok=True)
    torch_bags, names = dataloader(args, traj_path, args.max_obs,bag_list, input_size, output_size, device, batchsize)
    print("bag num", len(bag_list))
    
    avg_loss=[]

    for i, curbag in enumerate(torch_bags):
        curbag_loss = []
        curbag_output = []
        for batch_idx, batch in enumerate(curbag):
            planner.zero_grad()
            cur_batch = batch[0].to(device).unsqueeze(0)
            #print(cur_batch[0])
            cur_real_output = batch[1].to(device)
            # ===================forward=====================
            cur_planner_output = planner(cur_batch)[0]
            loss = mse_loss(cur_planner_output, cur_real_output)
            
            npout = cur_planner_output.detach().numpy()
            curbag_output.append(cur_planner_output.detach().numpy())
            
            curbag_loss.append(loss.cpu().detach().numpy())
        
        new_path_dir = os.path.join(traj_path, str(i)+"_gen.pickle")
        
        with open(new_path_dir, 'wb') as f:
            pickle.dump(curbag_output, f, pickle.HIGHEST_PROTOCOL)
        
        avg_loss.append(sum(curbag_loss)/len(curbag_loss))
        print(avg_loss[-1])
        print(curbag_output[-3:])
        print(3)

    avg_loss = sum(avg_loss)/len(avg_loss)
    
    val_loss = avg_loss
    print ("--validation loss:", val_loss)
    
if __name__ == '__main__':
    current_path = os.getcwd()
    data_path = os.path.join(current_path, "data")
    bag_path = os.path.join(data_path, "bag")
    val_path = os.path.join(data_path, "validation_bags")
    obstacle_size = 8 * 3 
    input_size = obstacle_size + 3 + 4 #obstacles in environment and start, goal, ori
    output_size = 3 #twist (linear and angular velocity)
    batchsize = 500
    num_epochs = 1000
    lr=0.0001
    seed = 0
    
    parser = argparse.ArgumentParser()

	# mpnet training data generation
    parser.add_argument('--device', type=str, default="cuda")
    parser.add_argument('--current_path', type=str, default=current_path)
    parser.add_argument('--data_path', type=str, default=data_path)
    parser.add_argument('--bag_path', type=str, default=bag_path)
    parser.add_argument('--val_path', type=str, default=val_path)
    parser.add_argument('--obstacle_size', type=int, default=obstacle_size)
    parser.add_argument('--max_obs', type=int, default=8)
    parser.add_argument('--input_size', type=int, default=input_size)
    parser.add_argument('--output_size', type=int, default=output_size)
    parser.add_argument('--batchsize', type=int, default=batchsize)
    parser.add_argument('--num_epochs', type=int, default=num_epochs)
    parser.add_argument('--lr', type=int, default=lr)
    parser.add_argument('--seed', type=int, default=seed)
    
    args = parser.parse_args()

    validation(args)