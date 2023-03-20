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
    
    bagfilelist = os.listdir(output_path)
    temp = []

    for i,file in enumerate(bagfilelist):
        if file[-3:] == "bag":
            temp.append(file)

    bagfilelist = temp

    print(bagfilelist)

    imgtopics = ["/spot/camera/frontleft/image/compressed",
             "/spot/camera/frontright/image/compressed",]
    
    for file in bagfilelist:
        bagidx  = file.find(".bag")

        curdir = output_path + "/" + file[:bagidx]
        print(curdir)

        frontleft  = []
        frontright = []

        for savedtopic in imgtopics:

            camloc = savedtopic.find("/spot/camera/")+12
            lastidx = camloc

            while savedtopic[lastidx+1] != '/':
                lastidx += 1

            campos = savedtopic[camloc:lastidx+1]

            curcamdir = curdir + savedtopic[camloc:lastidx+1]

            if campos == "/frontleft":
                temp  = os.listdir(curcamdir)
                for i in range(len(temp)):
                    img = cv2.imread(curcamdir + "/" + temp[i])
                    grayLeft = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    frontleft.append(grayLeft)

            elif campos == "/frontright":
                temp = os.listdir(curcamdir)
                for i in range(len(temp)):
                    img = cv2.imread(curcamdir + "/" + temp[i])
                    grayright = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    frontright.append(grayright)

        if depth_calc == True:
            #left
            print(len(frontleft), len(frontright))
            Depth(curdir + "/depth", frontleft, frontright)

    return

if __name__ == '__main__':
    main()