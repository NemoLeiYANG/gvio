#!/usr/bin/env python
import os
import sys
import shutil
from os import listdir
from os.path import isfile, join

import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge


def print_usage():
    print("Usage: ds2vid.py <ros bag> <output path>")
    print("Example: ds2vid.py record.bag record.avi")


def bag2imgs(topic, camera_index, output_dir):
    # Create output directory
    output_path = join(output_dir, "cam%d" % camera_index)
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    # Convert bag to images
    br = CvBridge()
    frame_index = 0
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # Convert image message to np.array
        if msg_type == "sensor_msgs/CompressedImage":
            image = br.compressed_imgmsg_to_cv2(msg)
        else:
            image = br.imgmsg_to_cv2(msg)

        # Write image to file
        image_fname = "%d.jpg" % (frame_index)
        image_output_path = os.path.join(output_path, image_fname)
        cv2.imwrite(image_output_path, image)
        frame_index += 1


if __name__ == "__main__":
    # Check CLI args
    if len(sys.argv) != 3:
        print_usage()
        exit(-1)

    # Parse CLI args
    bag = rosbag.Bag(sys.argv[1], 'r')
    output_path = sys.argv[2]
    bag_name = os.path.basename(sys.argv[1]).replace(".bag", "")
    topics = ["/gvio/cam0/image", "/gvio/cam1/image", "/gvio/cam2/image"]

    # Check if topic is in bag
    info = bag.get_type_and_topic_info()
    for topic in topics:
        if topic not in info.topics:
            raise RuntimeError("Opps! topic not in bag!")

    # Check image message type
    print("Processing [%s] bag .. " % bag_name)
    fps = info.topics[topic].frequency
    msg_type = info.topics[topic].msg_type
    supported_msgs = ["sensor_msgs/CompressedImage", "sensor_msgs/Image"]
    if msg_type not in supported_msgs:
        err_msg = "ds2vid only supports %s!" % " or ".join(supported_msgs)
        raise RuntimeError(err_msg)

    # Convert bag to images
    print("-- Converting bag to images")
    bag2imgs(topics[0], 0, bag_name)
    bag2imgs(topics[1], 1, bag_name)
    bag2imgs(topics[2], 2, bag_name)

    # Get image paths for all three cameras
    cam0_path = join(bag_name, "cam0")
    cam1_path = join(bag_name, "cam1")
    cam2_path = join(bag_name, "cam2")
    cam0_files = [f for f in listdir(cam0_path) if isfile(join(cam0_path, f))]
    cam1_files = [f for f in listdir(cam1_path) if isfile(join(cam1_path, f))]
    cam2_files = [f for f in listdir(cam2_path) if isfile(join(cam2_path, f))]
    cam0_files.sort()
    cam1_files.sort()
    cam2_files.sort()

    length = len(cam0_files)
    if all(len(lst) != length for lst in [cam0_files, cam1_files, cam2_files]):
        err_msg = "Camera images are not the same length!\n"
        err_msg += "Recommend you use rqt_bag to inspect bag to make sure\n"
        err_msg += "cam0, cam1 and cam2 topics have the same number of msgs!"
        raise RuntimeError(err_msg)

    # Combine images to video
    print("-- Converting images to video")
    frame = cv2.imread(join(cam0_path, "0.jpg"))
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    width = 752 * 2
    height = 480 * 2
    video = cv2.VideoWriter(join(output_path, bag_name) + ".avi",
                            fourcc,
                            fps,
                            (width, height))

    for i in range(len(cam0_files)):
        frame0 = cv2.imread(join(cam0_path, "%d.jpg" % i))
        frame1 = cv2.imread(join(cam1_path, "%d.jpg" % i))
        frame2 = cv2.imread(join(cam2_path, "%d.jpg" % i))

        # Top - cam0 and cam2
        top = np.hstack((frame0, frame2))

        # Bottom - cam1
        bottom = np.zeros((height / 2, width, 3), dtype='uint8')
        padding_left = width / 4
        padding_right = width - width / 4
        bottom[:, padding_left:padding_right, :] = frame1[:, :, :]

        # Stack top and bottom then write to video
        frame = np.vstack((top, bottom))
        video.write(frame)

    video.release()

    # Delete images directory
    shutil.rmtree(bag_name)
    print("Done!")
