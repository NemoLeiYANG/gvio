#!/usr/bin/env python
import sys

import cv2
import rosbag
from cv_bridge import CvBridge


def print_usage():
    print("Usage: bag2imgs.py <ros bag> <ros topic>")


if __name__ == "__main__":
    # Check CLI args
    if len(sys.argv) != 3:
        print_usage()
        exit(-1)

    # Parse CLI args
    bag = rosbag.Bag(sys.argv[1], 'r')
    topics = [sys.argv[2]]

    # COnvert bag to images
    br = CvBridge()
    index = 0
    for topic, msg, t in bag.read_messages(topics=topics):
        image = br.compressed_imgmsg_to_cv2(msg)
        image_fname = "image_%d.jpg" % index
        cv2.imwrite(image_fname, image)
        print(image_fname)
        index += 1
