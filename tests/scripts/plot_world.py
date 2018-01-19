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
    # Setup plot
    fig = plt.figure()

    # Plot features
    features_file = sys.argv[1]
    features_data = np.loadtxt(open(features_file, "r"), delimiter=",")
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(features_data[:, 0], features_data[:, 1], features_data[:, 2])

    # Plot robot trajectory
    if len(sys.argv) > 3:
        trajectory_file = sys.argv[2]
        trajectory_data = parse_data(trajectory_file)
        ax.plot(trajectory_data["x"], trajectory_data["y"])

    # Show plot
    plt.show()
