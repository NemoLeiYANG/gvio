import sys

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


if __name__ == "__main__":
    data_path = sys.argv[1]
    data = parse_data(data_path)

    plt.subplot(411)
    plt.plot(data["x"], data["y"])
    plt.xlabel("Position - East (m)")
    plt.ylabel("Position - West (m)")

    plt.subplot(412)
    plt.plot(data["t"], data["vx"], marker="o")
    plt.plot(data["t"], data["vy"], marker="o")
    plt.plot(data["t"], data["vz"], marker="o")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")

    plt.subplot(413)
    plt.plot(data["t"], data["ax"])
    plt.plot(data["t"], data["ay"])
    plt.plot(data["t"], data["az"])
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")

    plt.subplot(414)
    plt.plot(data["t"], data["roll"])
    plt.plot(data["t"], data["pitch"])
    plt.plot(data["t"], data["yaw"])
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")

    plt.show()
