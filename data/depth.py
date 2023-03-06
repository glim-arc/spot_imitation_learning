import os
import cv2

class Depth:
    def __init__(self, path, side, front):
        if not os.path.isdir(path):
                os.makedirs(path)

        for i in range(len(side)):
            stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
            disparity = stereo.compute(path, side)

            cv2.imwrite(os.path.join(path, "%f.png" % i), disparity)