#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState

import numpy as np

import csv
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Simulation:
    def __init__(self, node_name="Move"):
        self.node_name = node_name
        print("Starting node Move")

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        # Publishes to robot twist
        self.cmd_vel_pub = rospy.Publisher('/spot1/cmd_vel', Twist, queue_size=1)  

        #PATH publisher
        self.robot_path_pub = rospy.Publisher('robo_plan', Path, queue_size=1)
        self.ball_path_pub = rospy.Publisher('ball_plan', Path, queue_size=1)

    def pose_callback(self, data):
        self.currentPose = data
        rospy.loginfo('current_pose:{:.4f}'.format(self.currentPose.pose.position))

    def run(self):
        M = Models()

        robot_curr_output = []
        ball_curr_output = []
        robot_path = Path()
        robot_path.header.frame_id = 'map'

        ball_path = Path()
        ball_path.header.frame_id = 'map'

        pose_x = 0
        pose_y = 0

        
        for i in range(200):

            newTwist = Twist()
            newTwist.linear.x = 0
            newTwist.linear.y = 0.4
            newTwist.linear.z = 0
            newTwist.angular.x = 0
            newTwist.angular.y = 0
            newTwist.angular.z = 0
            robot_curr_output.append(newTwist)

            #calculated path
            delta_time = 0.06
            pose_x += 0*delta_time
            pose_y += 0.4*delta_time
            calcpose_msg = ModelState()
            calcpose_msg.model_name = 'cricket_ball'
            calcpose_msg.pose.position.x = pose_x
            calcpose_msg.pose.position.y = pose_y
            calcpose_msg.pose.position.z = 0
            calcpose_msg.pose.orientation.x = 0
            calcpose_msg.pose.orientation.y = 0
            calcpose_msg.pose.orientation.z = 0
            ball_curr_output.append(calcpose_msg)

            newPose = PoseStamped()
            newPose.pose = calcpose_msg.pose
            ball_path.poses.append(newPose)

        index = 0
        while not rospy.is_shutdown():
            M.show_gazebo_models()

            index += 1

            self.cmd_vel_pub.publish(robot_curr_output[index])

            #add robot pose to path and publish all the time

            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state(ball_curr_output[index])

            newPose = PoseStamped()
            newPose.pose.position.x = M.spotPose[0]
            newPose.pose.position.y = M.spotPose[1]
            newPose.pose.position.z = M.spotPose[2]

            robot_path.poses.append(newPose)

            self.robot_path_pub.publish(robot_path)
            self.ball_path_pub.publish(ball_path)
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

            spot_coord = model_coordinates('spot1', '')
            self.spotPose = [spot_coord.pose.position.x, spot_coord.pose.position.y, spot_coord.pose.position.z] 
            self.spotOri = [spot_coord.pose.orientation.x, spot_coord.pose.orientation.y, spot_coord.pose.orientation.z, spot_coord.pose.orientation.w] 

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))
    
sim = Simulation()
sim.run()
