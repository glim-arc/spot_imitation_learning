#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
import csv
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import pickle
import os



def position_node():

    # Create a publisher object with Twist
    pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
    path_pub = rospy.Publisher('global_plan', Path, queue_size=1)
    path_pub2 = rospy.Publisher('global_plan2', Path, queue_size=1)
    path_pub3 = rospy.Publisher('global_plan3', Path, queue_size=1)


    # Declare the node, and register it with a unique name
    rospy.init_node('model_service_node', anonymous=True)

    # Define the execution rate object (10Hz)
    rate = rospy.Rate(10)


    # Create message object with a specific type
    path_real = Path()
    path_real.header.frame_id = 'map'

    path_calc = Path()
    path_calc.header.frame_id = 'map'

    path_gen = Path()
    path_gen.header.frame_id = 'map'

    pickle_path = os.path.join("data/path_comparison/validation/6.pickle")
    with open(pickle_path, 'rb') as f:
        bag_list = pickle.load(f)

    pickle_path = os.path.join("data/path_comparison/validation/6_gen.pickle")
    with open(pickle_path, 'rb') as f:
        bag_list_gen = pickle.load(f)


    
    pos_list = bag_list[1]
    start_pose = [pos_list[0][0], pos_list[0][1]]
    for pos in pos_list:
        newPose = PoseStamped()
        newPose.pose.position.x = pos[0] - start_pose[0]
        newPose.pose.position.y = pos[1] - start_pose[1]
        newPose.pose.position.z = 0
        path_real.poses.append(newPose)

    
    pose_x = 0
    pose_y = 0
    twist_lin_list = bag_list[3]
    for twist in twist_lin_list:
        newPose = PoseStamped()
        delta_time = 0.06
        pose_x += twist[0]*delta_time
        pose_y += twist[1]*delta_time
        newPose.pose.position.x = pose_x
        newPose.pose.position.y = pose_y
        path_calc.poses.append(newPose)


    
    pose_x = 0
    pose_y = 0
    for bag in bag_list_gen:
        newPose = PoseStamped()
        delta_time = 0.06
        pose_x += bag[0].item()*delta_time
        pose_y += bag[1].item()*delta_time
        newPose.pose.position.x = pose_x
        newPose.pose.position.y = pose_y
        path_gen.poses.append(newPose)

    while not rospy.is_shutdown():

        path_pub.publish(path_real)
        path_pub2.publish(path_calc)
        path_pub3.publish(path_gen)



        # Sleep the necessary amount of time to keep a 10Hz execution rate
        rate.sleep()


if __name__ == '__main__':

    try:
        position_node()

    except rospy.ROSInterruptException:
        pass