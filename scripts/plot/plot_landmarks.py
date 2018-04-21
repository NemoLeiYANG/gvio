import sys

import numpy as np
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D  # NOQA


if __name__ == "__main__":
    X = np.loadtxt(open(sys.argv[1], "r"), delimiter=",")
    X = np.matrix(X)
    print(X[:, 0].shape)

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter([X[:, 0]], [X[:, 1]], [X[:, 2]])

    plt.scatter([X[:, 0]], [X[:, 1]])

    x_min = np.min(X[:, 0])
    x_max = np.max(X[:, 0])
    y_min = np.min(X[:, 1])
    y_max = np.max(X[:, 1])

    plt.xlim([x_min, x_max])
    plt.ylim([y_min, y_max])
    plt.show()
