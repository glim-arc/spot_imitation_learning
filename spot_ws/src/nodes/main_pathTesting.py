#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
import pickle
import os
import argparse



class Simulation:
    def __init__(self, node_name="Move"):
        self.node_name = node_name
        print("Starting node Move")

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        #SETUP Publisher
        self.real_path_pub = rospy.Publisher('real_path', Path, queue_size=1)
        self.comp0_pub = rospy.Publisher('comp0_path', Path, queue_size=1)
        self.comp1_pub = rospy.Publisher('comp1_path', Path, queue_size=1)
        self.comp2_pub = rospy.Publisher('comp2_path', Path, queue_size=1)
        self.comp3_pub = rospy.Publisher('comp3_path', Path, queue_size=1)
        self.comp4_pub = rospy.Publisher('comp4_path', Path, queue_size=1)

        #DIRECTORIES INFO
        pickle_file = args.pickle_file
        data_type = args.data_type
        comp0 = args.comp0
        comp1 = args.comp1
        comp2 = args.comp2
        comp3 = args.comp3
        comp4 = args.comp4

        current_path = os.getcwd()
        data_path = os.path.join(current_path, "data")

        #GET OBSTACLES
        obs_dir = data_path +  '/' + comp1 + "/path_comparison/" + data_type + "/obs/" + str(pickle_file) +".pickle"
        with open(obs_dir, 'rb') as f:
            self.obs_list = pickle.load(f)
        #GET real path
        real_dir = data_path +  '/' + comp1 + "/path_comparison/" + data_type + '/' + str(pickle_file) +".pickle"
        with open(real_dir, 'rb') as f:
            self.real_path_list = pickle.load(f)
        #GET zero-th PATH to compare
        comp0_dir = data_path +  '/' + comp0 + "/path_comparison/" + data_type + '/' + str(pickle_file) +"_gen.pickle"
        with open(comp0_dir, 'rb') as f:
            self.comp0_path_list = pickle.load(f)
        #GET first PATH to compare
        comp1_dir = data_path +  '/' + comp1 + "/path_comparison/" + data_type + '/' + str(pickle_file) +"_gen.pickle"
        with open(comp1_dir, 'rb') as f:
            self.comp1_path_list = pickle.load(f)
        #GET second PATH to compare
        comp2_dir = data_path +  '/' + comp2 + "/path_comparison/" + data_type + '/' + str(pickle_file) +"_gen.pickle"
        with open(comp2_dir, 'rb') as f:
            self.comp2_path_list = pickle.load(f)
        #GET third PATH to compare
        comp3_dir = data_path +  '/' + comp3 + "/path_comparison/" + data_type + '/' + str(pickle_file) +"_gen.pickle"
        with open(comp3_dir, 'rb') as f:
            self.comp3_path_list = pickle.load(f)
        #GET second PATH to compare
        comp4_dir = data_path +  '/' + comp4 + "/path_comparison/" + data_type + '/' + str(pickle_file) +"_gen.pickle"
        with open(comp4_dir, 'rb') as f:
            self.comp4_path_list = pickle.load(f)

        #SETUP path for RVIZ
        self.path_real = Path()
        self.path_real.header.frame_id = 'map'
        self.path_comp0 = Path()
        self.path_comp0.header.frame_id = 'map'
        self.path_comp1 = Path()
        self.path_comp1.header.frame_id = 'map'
        self.path_comp2 = Path()
        self.path_comp2.header.frame_id = 'map'
        self.path_comp3 = Path()
        self.path_comp3.header.frame_id = 'map'
        self.path_comp4 = Path()
        self.path_comp4.header.frame_id = 'map'

        #Real Path
        pos_list = self.real_path_list[1]
        start_pose = [pos_list[0][0], pos_list[0][1]]
        for pos in pos_list:
            newPose = PoseStamped()
            newPose.pose.position.x = pos[0] - start_pose[0]
            newPose.pose.position.y = pos[1] - start_pose[1]
            newPose.pose.position.z = 0
            self.path_real.poses.append(newPose)

        #comp0 path
        pose_x = 0
        pose_y = 0
        for bag in self.comp0_path_list:
            newPose = PoseStamped()
            delta_time = 0.06
            pose_x += bag[0].item()*delta_time
            pose_y += bag[1].item()*delta_time
            newPose.pose.position.x = pose_x
            newPose.pose.position.y = pose_y
            self.path_comp0.poses.append(newPose)

        #comp1 path
        pose_x = 0
        pose_y = 0
        for bag in self.comp1_path_list:
            newPose = PoseStamped()
            delta_time = 0.06
            pose_x += bag[0].item()*delta_time
            pose_y += bag[1].item()*delta_time
            newPose.pose.position.x = pose_x
            newPose.pose.position.y = pose_y
            self.path_comp1.poses.append(newPose)

        #comp2 path
        pose_x = 0
        pose_y = 0
        for bag in self.comp2_path_list:
            newPose = PoseStamped()
            delta_time = 0.06
            pose_x += bag[0].item()*delta_time
            pose_y += bag[1].item()*delta_time
            newPose.pose.position.x = pose_x
            newPose.pose.position.y = pose_y
            self.path_comp2.poses.append(newPose)

        #comp3 path
        pose_x = 0
        pose_y = 0
        for bag in self.comp3_path_list:
            newPose = PoseStamped()
            delta_time = 0.06
            pose_x += bag[0].item()*delta_time
            pose_y += bag[1].item()*delta_time
            newPose.pose.position.x = pose_x
            newPose.pose.position.y = pose_y
            self.path_comp3.poses.append(newPose)

        #comp4 path
        pose_x = 0
        pose_y = 0
        for bag in self.comp4_path_list:
            newPose = PoseStamped()
            delta_time = 0.06
            pose_x += bag[0].item()*delta_time
            pose_y += bag[1].item()*delta_time
            newPose.pose.position.x = pose_x
            newPose.pose.position.y = pose_y
            self.path_comp4.poses.append(newPose)



    def run(self):
        M = Models()
         
        obs_list_index = 0
        obs_list = self.obs_list

        while not rospy.is_shutdown():
            M.show_gazebo_models()
            
            self.real_path_pub.publish(self.path_real)
            self.comp0_pub.publish(self.path_comp0)
            self.comp1_pub.publish(self.path_comp1)
            # self.comp2_pub.publish(self.path_comp2)
            # self.comp3_pub.publish(self.path_comp3)
            # self.comp4_pub.publish(self.path_comp4)

            if obs_list[obs_list_index] != None:
                lenList = len(obs_list[obs_list_index])
            else:
                lenList = 0

            for i in range (lenList):
                box_msg = ModelState()
                box_msg.model_name = 'unit_box_' + str(i)
                box_msg.pose.position.x = obs_list[obs_list_index][i][0] + M.robotPose[0]
                box_msg.pose.position.y = obs_list[obs_list_index][i][1] + M.robotPose[1]
                box_msg.pose.position.z = 0
                M.move_box(box_msg)

            for i in range (lenList, 8):
                box_msg = ModelState()
                box_msg.model_name = 'unit_box_' + str(i)
                box_msg.pose.position.x = 100
                box_msg.pose.position.y = 100
                box_msg.pose.position.z = 100
                M.move_box(box_msg)

            robotpose_msg = ModelState()
            robotpose_msg.model_name = 'cricket_ball'
            robotpose_msg.pose = self.path_real.poses[obs_list_index].pose
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state(robotpose_msg)

            robotpose_msg = ModelState()
            robotpose_msg.model_name = 'cricket_ball_green'
            robotpose_msg.pose = self.path_comp1.poses[obs_list_index].pose
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state(robotpose_msg)

            if obs_list_index == len(obs_list) - 1:
                print("Bag finished")
               
            else:
                obs_list_index += 1

            
            

            self.rate.sleep() 

        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.node_name))


class Models:

    def __init__(self):
        self.robotPose = []

    def move_box(self, msg):
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state(msg)

    def show_gazebo_models(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            robot_coord = model_coordinates('cricket_ball', '')
            self.robotPose = [robot_coord.pose.position.x, robot_coord.pose.position.y, robot_coord.pose.position.z] 

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


    
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()

	# mpnet training data generation
    parser.add_argument('--pickle_file', type=int, default=3)
    parser.add_argument('--data_type', type=str, default="validation")
    parser.add_argument('--comp0', type=str, default="old_data/t_200")
    parser.add_argument('--comp1', type=str, default="new_data/t_200")
    parser.add_argument('--comp2', type=str, default="new_data/t_150")
    parser.add_argument('--comp3', type=str, default="new_data/t_200")
    parser.add_argument('--comp4', type=str, default="new_data/t_250")
    args = parser.parse_args()
    
    sim = Simulation()
    sim.run()

