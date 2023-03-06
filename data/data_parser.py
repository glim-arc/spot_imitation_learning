#!/usr/bin/env python

"""Extract topics from a rosbag.
"""

import os
import cv2
import numpy as np
import rospy 

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from bagpy import bagreader
from depth import Depth

depth_calc = True

def main():
    """Extract a folder of images from a rosbag.
    """

    current_path = os.getcwd()

    output_path = current_path + "/bag"

    # if not os.path.isdir(output_path):
    #         os.makedirs(output_path, exist_ok=True)
    # else:
    #     idx = 1
    #     while os.path.isdir("./output"+str(idx)):
    #         idx += 1
        
    #     output_path = "./output"+ str(idx)
    #     os.makedirs(output_path, exist_ok=True)
    
    bagfilelist = os.listdir(output_path)
    temp = []

    for i,file in enumerate(bagfilelist):
        if file[-3:] == "bag":
            temp.append(file)

    bagfilelist = temp

    print(bagfilelist)

    imgtopics = ["/spot/camera/back/image/compressed",
             "/spot/camera/frontleft/image/compressed",
             "/spot/camera/frontright/image/compressed",
             "/spot/camera/left/image/compressed",
             "/spot/camera/right/image/compressed"]
    
    other_topics = ["/joystick", "/localization", "/joint_states", "/odom", "/navigation/cmd_vel"]

    for file in bagfilelist:
        bag = rosbag.Bag("./bag/" + file, "r")
        bridge = CvBridge()

        bagidx  = file.find(".bag")

        curdir = output_path + "/" + file[:bagidx]

        if not os.path.isdir(file[:bagidx]):
            os.makedirs(curdir, exist_ok=True)

        b = bagreader("./bag/" + file)

        for savedtopic in other_topics:
            bmesg = b.message_by_topic(savedtopic)

        right = []
        left = []
        frontleft = []
        frontright = []

        for savedtopic in imgtopics:
            # save in csv
            #bmesg = b.message_by_topic(savedtopic)

            print(savedtopic)
            count = 0

            camloc = savedtopic.find("/spot/camera/")+12
            lastidx = camloc

            while savedtopic[lastidx+1] != '/':
                lastidx += 1

            campos = savedtopic[camloc:lastidx+1]

            curcamdir = curdir + savedtopic[camloc:lastidx+1]
            print("dir", curcamdir)

            if not os.path.isdir(curcamdir):
                os.makedirs(curcamdir)

            print(campos)
            for topic, msg, t in bag.read_messages(topics=[savedtopic]):
                cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
                
                if campos == "/frontleft":
                    cv_img = cv2.rotate(cv_img, cv2.ROTATE_90_CLOCKWISE)
                    frontleft.append(cv_img)
                elif campos == "/frontright":
                    cv_img = cv2.rotate(cv_img, cv2.ROTATE_90_CLOCKWISE)
                    frontright.append(cv_img)
                elif campos == "/right":
                    cv_img = cv2.rotate(cv_img, cv2.ROTATE_180)
                    right.append(cv_img)
                elif campos == "/left":
                    left.append(cv_img)

                time = float(t.secs) + float(t.nsecs)*(1e-9)
                time = round(time, 6)

                cv2.imwrite(os.path.join(curcamdir, "%f.png" % time), cv_img)
                count += 1
            

        bag.close()

        if depth_calc == True:
            #left
            Depth(curdir + "/depth_left", left, frontleft)

            #right
            Depth(curdir+ "/depth_right", right, frontright)

    return

if __name__ == '__main__':
    main()