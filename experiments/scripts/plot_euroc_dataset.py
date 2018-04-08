import sys
import os
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


def plot_measurements(imu_data):
    plt.subplot(211)

    w_m = np.array([imu_data["w_RS_S_x [rad s^-1]"],
                    imu_data["w_RS_S_y [rad s^-1]"],
                    imu_data["w_RS_S_z [rad s^-1]"]])
    a_m = np.array([imu_data["a_RS_S_x [m s^-2]"],
                    imu_data["a_RS_S_y [m s^-2]"],
                    imu_data["a_RS_S_z [m s^-2]"]])

    R_body_imu = np.array([[0.0, 0.0, 1.0],
                           [0.0, -1.0, 0.0],
                           [1.0, 0.0, 0.0]])

    a_m = np.dot(R_body_imu, a_m)
    w_m = np.dot(R_body_imu, w_m)

    imu_ts = imu_data["#timestamp [ns]"]
    plt.plot(imu_ts, w_m[0, :], label="x")
    plt.plot(imu_ts, w_m[1, :], label="y")
    plt.plot(imu_ts, w_m[2, :], label="z")
    plt.title("Gyroscope")
    plt.legend(loc=0)

    plt.subplot(212)
    plt.plot(imu_ts, a_m[0, :], label="x")
    plt.plot(imu_ts, a_m[1, :], label="y")
    plt.plot(imu_ts, a_m[2, :], label="z")
    plt.title("Accelerometer")
    plt.legend(loc=0)

    plt.show()


def plot_ground_truth(gnd_data):
    plt.subplot(211)
    plt.plot(gnd_data["p_RS_R_x [m]"], gnd_data["p_RS_R_y [m]"])

    plt.subplot(212)
    plt.plot(gnd_data["#timestamp"], gnd_data["p_RS_R_z [m]"])

    plt.show()


if __name__ == "__main__":
    base_path = sys.argv[1]

    # IMU data
    imu_data_path = os.path.join(base_path, "imu0", "data.csv")
    imu_data = parse_data(imu_data_path)
    plot_measurements(imu_data)

    # # Ground truth data
    # gnd_data_path = os.path.join(base_path,
    #                              "state_groundtruth_estimate0",
    #                              "data.csv")
    # gnd_data = parse_data(gnd_data_path)
    # plot_ground_truth(gnd_data)
