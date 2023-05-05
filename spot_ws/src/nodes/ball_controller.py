#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
import csv
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped



def position_node():

    # Create a publisher object with Twist
    pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
    path_pub = rospy.Publisher('global_plan', Path, queue_size=1)
    path_pub2 = rospy.Publisher('global_plan2', Path, queue_size=1)

    # Declare the node, and register it with a unique name
    rospy.init_node('model_service_node', anonymous=True)

    # Define the execution rate object (10Hz)
    rate = rospy.Rate(10)


    # Create message object with a specific type

    curr_output = []
    path_real = Path()
    path_real.header.frame_id = 'map'

    path_calc = Path()
    path_calc.header.frame_id = 'map'

    with open("./data/odom3.csv", 'r') as file:
        csvreader = csv.reader(file)
        pose_x = 0
        pose_y = 0
        for row in csvreader:
            #real path
            realpose_msg = ModelState()
            realpose_msg.model_name = 'cricket_ball'
            realpose_msg.pose.position.x = float(row[0])
            realpose_msg.pose.position.y = float(row[1])
            realpose_msg.pose.position.z = 1.02
            realpose_msg.pose.orientation.x = float(row[3])
            realpose_msg.pose.orientation.y = float(row[4])
            realpose_msg.pose.orientation.z = float(row[5])
            realpose_msg.pose.orientation.w = float(row[6])
            curr_output.append(realpose_msg)

            newPose = PoseStamped()
            newPose.pose = realpose_msg.pose
            path_real.poses.append(newPose)

            #calculated path
            delta_time = 0.1
            pose_x += float(row[7])*delta_time
            pose_y += float(row[8])*delta_time
            calcpose_msg = ModelState()
            calcpose_msg.pose.position.x = pose_x
            calcpose_msg.pose.position.y = pose_y
            calcpose_msg.pose.position.z = 1.02
            calcpose_msg.pose.orientation.x = 0
            calcpose_msg.pose.orientation.y = 0
            calcpose_msg.pose.orientation.z = 0

            newPose = PoseStamped()
            newPose.pose = calcpose_msg.pose
            path_calc.poses.append(newPose)


    index = 0
    while not rospy.is_shutdown():

        try:

            index +=1 
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            resp = set_state(curr_output[index])

            path_pub.publish(path_real)
            path_pub2.publish(path_calc)


        except rospy.ServiceException:
            print("Service call failed: ")


        # Sleep the necessary amount of time to keep a 10Hz execution rate
        rate.sleep()


if __name__ == '__main__':

    try:
        position_node()

    except rospy.ROSInterruptException:
        pass