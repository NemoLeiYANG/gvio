import sys
from math import floor
from math import ceil

import numpy as np
import matplotlib.pylab as plt


def parse_data(data_path):
    # Check number of lines
    nb_lines = sum(1 for line in open(data_path))
    if nb_lines <= 2:
        print("No data in [" + data_path + "]?")
        return None

    # Load csv as numpy matrix
    data_file = open(data_path, "r")
    header = open(data_path, "r").readline()
    data = np.loadtxt(data_file, delimiter=",", skiprows=1)
    data_file.close()

    # Convert numpy matrix as dictionary where each column is represented by
    # its header name
    data_dict = {}
    index = 0
    for element in header.split(","):
        data_dict[element.strip()] = data[:, index]
        index += 1

    return data_dict


def plot_position(gnd_data):
    plt.figure()

    plt.subplot(211)
    plt.plot(gnd_data["x"], gnd_data["y"], label="Ground truth")
    plt.title("Position")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.axis('equal')
    plt.legend(loc=0)

    plt.subplot(212)
    plt.plot(gnd_data["t"], gnd_data["z"], label="Ground Truth")

    # z_min = floor(min(np.min(gnd_data["z"]), np.min(est_data["z"])))
    # z_max = ceil(max(np.max(gnd_data["z"]), np.max(est_data["z"])))
    # z_min = min(-1, z_min)
    # z_max = max(1, z_max)

    plt.title("Altitude")
    plt.xlabel("Time (s)")
    plt.ylabel("Height (m)")
    # plt.ylim([z_min, z_max])
    plt.legend(loc=0)


def plot_attitude(gnd_data):
    plt.figure()
    plt.suptitle("Attitude")

    plt.subplot(311)
    plt.plot(gnd_data["t"], gnd_data["roll"], label="Ground truth")
    plt.title("Roll")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")
    plt.legend(loc=0)

    plt.subplot(312)
    plt.plot(gnd_data["t"], gnd_data["pitch"], label="Ground truth")
    plt.title("Pitch")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")
    plt.legend(loc=0)

    plt.subplot(313)
    plt.plot(gnd_data["t"], gnd_data["yaw"], label="Ground truth")
    plt.title("Yaw")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")
    plt.legend(loc=0)

# def plot_measurements(mea_data):
#     plt.figure()
#     plt.title("Measurement")
#
#     plt.subplot(211)
#     plt.title("Accelerometer")
#     plt.plot(mea_data["ax_B"])
#     plt.plot(mea_data["ay_B"])
#     plt.plot(mea_data["az_B"])
#
#     plt.legend(loc=0)
#
#     plt.subplot(212)
#     plt.title("Gyroscope")
#     plt.plot(mea_data["wx_B"])
#     plt.plot(mea_data["wy_B"])
#     plt.plot(mea_data["wz_B"])
#
#     plt.legend(loc=0)


if __name__ == "__main__":
    data_path = sys.argv[1]
    gnd_data = parse_data(data_path)

    plot_position(gnd_data)
    plot_attitude(gnd_data)
    # plot_measurements(mea_data)
    plt.show()
