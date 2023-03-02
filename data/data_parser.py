#!/usr/bin/env python

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file",type=str, default='./library2pond_spot.bag',  help="Input ROS bag.")
    parser.add_argument("output_dir", type=str, default='./output', help="Output directory.")

    imgtopics = ["/spot/camera/back/image/compressed",
             "/spot/camera/frontleft/image/compressed",
             "/spot/camera/frontright/image/compressed",
             "/spot/camera/left/image/compressed",
             "/spot/camera/right/image/compressed"]
    
    joystick_input = ["/joystick"]
    localization = ["/localization"]

    args = parser.parse_args()

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    
    for savedtopic in imgtopics:
        print(savedtopic)

        camloc = savedtopic.find("/spot/camera/")
        lastidx = camloc
        while camloc[lastidx] != '/':
            lastidx += 1

        curdir = "./" + savedtopic[camloc+1:lastidx]
        os.makedirs(curdir)

        for topic, msg, t in bag.read_messages(topics=[savedtopic]):
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            cv2.imwrite(os.path.join(curdir, "frame%06i.png" % count), cv_img)
            print("Wrote image %i" % count)

            count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()