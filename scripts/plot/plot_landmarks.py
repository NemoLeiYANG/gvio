import sys

import numpy as np
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D  # NOQA


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
    X = np.loadtxt(open(sys.argv[1], "r"), delimiter=",")
    X = np.matrix(X)

    gnd_data = parse_data(sys.argv[2])

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter([X[:, 0]], [X[:, 1]], [X[:, 2]])

    plt.scatter([X[:, 0]], [X[:, 1]])
    plt.plot(gnd_data["x"], gnd_data["y"], color="red")

    x_min = np.min(X[:, 0])
    x_max = np.max(X[:, 0])
    y_min = np.min(X[:, 1])
    y_max = np.max(X[:, 1])

    plt.xlim([x_min, x_max])
    plt.ylim([y_min, y_max])
    plt.show()
