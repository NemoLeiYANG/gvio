import sys
import numpy as np
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D  # NOQA


if __name__ == "__main__":
    # Load data
    data_file = sys.argv[1]
    data = np.loadtxt(open(data_file, "r"), delimiter=",")

    # Plot features
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(data[:, 0], data[:, 1], data[:, 2])
    plt.show()
