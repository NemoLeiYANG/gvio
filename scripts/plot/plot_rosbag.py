#!/usr/bin/env python
import sys
from math import floor
from math import ceil
from math import asin
from math import atan2

import yaml

import rosbag

import numpy as np
import matplotlib.pylab as plt


def plot_position(est_data, win_data, gnd_data):
    plt.figure()

    # Plot position - XY
    plt.subplot(211)
    plt.plot(gnd_data["x"], gnd_data["y"], label="Ground truth")
    if est_data:
        plt.plot(est_data["x"], est_data["y"], label="Estimated")
    if win_data:
        plt.plot(win_data["x"], win_data["y"], label="Camera states")

    plt.title("Position")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.axis('equal')
    plt.legend(loc=0)

    # Plot altitude
    plt.subplot(212)
    plt.plot(gnd_data["t"], gnd_data["z"], label="Ground Truth")
    if est_data:
        plt.plot(est_data["t"], est_data["z"], label="Estimated")

    plt.title("Altitude")
    plt.xlabel("Time (s)")
    plt.ylabel("Height (m)")
    plt.legend(loc=0)

    if est_data and gnd_data:
        z_min = floor(min(np.min(gnd_data["z"]), np.min(est_data["z"])))
        z_max = ceil(max(np.max(gnd_data["z"]), np.max(est_data["z"])))
        z_min = min(-1, z_min)
        z_max = max(1, z_max)
        plt.ylim([z_min, z_max])


def plot_attitude(est_data, gnd_data):
    plt.figure()
    plt.suptitle("Attitude")

    plt.subplot(311)
    plt.plot(gnd_data["t"], gnd_data["roll"], label="Ground truth")
    plt.plot(est_data["t"], est_data["roll"], label="Estimate")
    plt.title("Roll")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")
    plt.legend(loc=0)

    plt.subplot(312)
    plt.plot(gnd_data["t"], gnd_data["pitch"], label="Ground truth")
    plt.plot(est_data["t"], est_data["pitch"], label="Estimate")
    plt.title("Pitch")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")
    plt.legend(loc=0)

    plt.subplot(313)
    plt.plot(gnd_data["t"], gnd_data["yaw"], label="Ground truth")
    plt.plot(est_data["t"], est_data["yaw"], label="Estimate")
    plt.title("Yaw")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")
    plt.legend(loc=0)


def plot_measurements(imu_data):
    plt.figure()
    plt.title("IMU Mesaurements")

    plt.subplot(211)
    plt.title("Accelerometer")
    plt.plot(imu_data["t"], imu_data["ax_B"])
    plt.plot(imu_data["t"], imu_data["ay_B"])
    plt.plot(imu_data["t"], imu_data["az_B"])

    plt.legend(loc=0)

    plt.subplot(212)
    plt.title("Gyroscope")
    plt.plot(imu_data["t"], imu_data["wx_B"])
    plt.plot(imu_data["t"], imu_data["wy_B"])
    plt.plot(imu_data["t"], imu_data["wz_B"])

    plt.legend(loc=0)


def parse_imu_messages(bag, topic):
    # Get time start
    bag_info = yaml.load(bag._get_yaml_info())
    time_start = bag_info["start"]

    # Parse imu messages
    time = []
    a_B = []
    w_B = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        t_k = t.to_sec() - time_start
        a_B_k = np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
        w_B_k = np.array([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z])

        time.append(t_k)
        a_B.append(a_B_k)
        w_B.append(w_B_k)

    # Convert list to numpy array
    time = np.array(time)
    a_B = np.array(a_B)
    w_B = np.array(w_B)

    # Form results
    imu_data = {
        "t": time,
        "ax_B": a_B[:, 0],
        "ay_B": a_B[:, 1],
        "az_B": a_B[:, 2],
        "wx_B": w_B[:, 0],
        "wy_B": w_B[:, 1],
        "wz_B": w_B[:, 2]
    }

    return imu_data


def quat2euler(quat, euler_seq):
    qw = quat[0]
    qx = quat[1]
    qy = quat[2]
    qz = quat[3]

    qw2 = pow(qw, 2)
    qx2 = pow(qx, 2)
    qy2 = pow(qy, 2)
    qz2 = pow(qz, 2)

    if euler_seq == 123:
        phi = atan2(2 * (qz * qw - qx * qy), (qw2 + qx2 - qy2 - qz2))
        theta = asin(2 * (qx * qz + qy * qw))
        psi = atan2(2 * (qx * qw - qy * qz), (qw2 - qx2 - qy2 + qz2))

    elif euler_seq == 321:
        phi = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2))
        theta = asin(2 * (qy * qw - qx * qz))
        psi = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2))

    else:
        raise RuntimeError("Not implemented!")

    return (phi, theta, psi)


def parse_transform_topic(bag, topic):
    # Get time start
    bag_info = yaml.load(bag._get_yaml_info())
    time_start = bag_info["start"]

    # Parse imu messages
    time = []
    t_G = []
    rpy_BG = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        t_k = msg.header.stamp.to_sec() - time_start
        t_G_k = np.array([msg.transform.translation.x,
                          msg.transform.translation.y,
                          msg.transform.translation.z])
        q_BG_k = np.array([msg.transform.rotation.w,
                           msg.transform.rotation.x,
                           msg.transform.rotation.y,
                           msg.transform.rotation.z])
        rpy_BG_k = quat2euler(q_BG_k, 321)

        time.append(t_k)
        t_G.append(t_G_k)
        rpy_BG.append(rpy_BG_k)

    # Convert list to numpy array
    time = np.array(time)
    t_G = np.array(t_G)
    rpy_BG = np.array(rpy_BG)

    # Form results
    transform_data = {
        "t": time,
        "x": t_G[:, 0],
        "y": t_G[:, 1],
        "z": t_G[:, 2],
        "roll": rpy_BG[:, 0],
        "pitch": rpy_BG[:, 1],
        "yaw": rpy_BG[:, 2]
    }

    return transform_data


def check_topics(bag, topics):
    info = bag.get_type_and_topic_info()
    for topic in topics:
        if topic not in info.topics:
            raise RuntimeError("Opps! topic not in bag!")


if __name__ == "__main__":
    bag_path = sys.argv[1]

    imu_topic = "/imu0"
    transform_topic = "/vicon/firefly_sbx/firefly_sbx"

    # Parse CLI args
    bag = rosbag.Bag(bag_path, 'r')
    bag_info = yaml.load(bag._get_yaml_info())

    # import pprint
    # pprint.pprint(bag_info)

    check_topics(bag, [imu_topic, transform_topic])
    # imu_data = parse_imu_messages(bag, imu_topic)
    gnd_data = parse_transform_topic(bag, transform_topic)

    plot_position(None, None, gnd_data)
    # plot_attitude(est_data, gnd_data)
    # plot_measurements(imu_data)
    plt.show()
