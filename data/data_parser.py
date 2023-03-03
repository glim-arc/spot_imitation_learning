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

    current_path = os.getcwd()

    output_path = "./output"

    if not os.path.isdir(output_path):
            os.makedirs(output_path, exist_ok=True)
    else:
        idx = 1
        while os.path.isdir("./output"+str(idx)):
            idx += 1
        
        output_path = "./output"+ str(idx)
        os.makedirs(output_path, exist_ok=True)

    bagfilelist= None

    for (root, directories, files) in os.walk("./bag"):
        bagfilelist = files

    print(bagfilelist)

    imgtopics = ["/spot/camera/back/image/compressed",
             "/spot/camera/frontleft/image/compressed",
             "/spot/camera/frontright/image/compressed",
             "/spot/camera/left/image/compressed",
             "/spot/camera/right/image/compressed"]
    
    joystick_input = ["/joystick"]
    localization = ["/localization"]

    for file in bagfilelist:
        bag = rosbag.Bag("./bag/" + file, "r")
        bridge = CvBridge()

        bagidx  = file.find(".bag")

        curdir = output_path + "/" + file[:bagidx]

        if not os.path.isdir(file[:bagidx]):
            os.makedirs(curdir, exist_ok=True)

        for savedtopic in imgtopics:
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
                elif campos == "/frontright":
                    cv_img = cv2.rotate(cv_img, cv2.ROTATE_90_CLOCKWISE)
                elif campos == "/right":
                    cv_img = cv2.rotate(cv_img, cv2.ROTATE_180)

                cv2.imwrite(os.path.join(curcamdir, "%i.png" % count), cv_img)
                count += 1
            
            print("Wrote image " + str(count) +  " at " + curcamdir)

        bag.close()

    return

if __name__ == '__main__':
    main()