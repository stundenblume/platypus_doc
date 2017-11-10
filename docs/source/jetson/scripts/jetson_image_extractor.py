#/usr/bin/env python

import os
import cv2
import sys
import rosbag
import numpy as np
from cv_bridge import CvBridge
from os.path import join, dirname, exists

def extractor(bagname):
    root_dir = dirname(bagname)
    left_dir = join(root_dir, 'left')
    right_dir = join(root_dir, 'right')
    depth_dir = join(root_dir, 'depth')
    
    if not exists(root_dir):
        os.makedirs("%s" % left_dir)
        os.makedirs("%s" % right_dir)
        os.makedirs("%s" % depth_dir)

    bridge = CvBridge()
    bag = rosbag.Bag(bagname)
    print "Saving left images to %s" % left_dir
    k = 0
    for topic, msg, t in bag.read_messages(topics=['/zed/left/image_raw_color']): 
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        img = np.array(cv_img)
        cv2.imwrite("%s/%d.jpg" % (left_dir, k), img)
        k += 1

    print "Done!\nSaved %d left color images in total" % k

    k = 0
    for topic, msg, t in bag.read_messages(topics=['/zed/right/image_raw_color']): 
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        img = np.array(cv_img)
        cv2.imwrite("%s/%d.jpg" % (right_dir, k), img)
        k += 1

    print "Done!\nSaved %d right color images in total" % k

    print "Saving depth images to %s" % depth_dir
    k = 0
    for topic, msg, t in bag.read_messages(topics=['/zed/depth/depth_registered']):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        img = np.array(cv_img)
        cv2.imwrite("%s/%d.jpg" % (depth_dir, k), img)
        k = k+1

    print "Done\nSaved %d depth images in total" % k



if __name__ == "__main__":
    if (len(sys.argv) < 2):
        print "Usage: python image_extractor.py <bag-filename>\n"
        exit()

    extractor(sys.argv[1])
