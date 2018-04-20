import sys
from os import listdir
from os.path import isfile, join

import numpy as np
import matplotlib.pylab as plt


if __name__ == "__main__":
    data_path = sys.argv[1]

    data_files = []
    for f in listdir(data_path):
        if isfile(join(data_path, f)):
            data_files.append(join(data_path, f))

    fig, ax = plt.subplots()
    for data_file in data_files:
        data = np.loadtxt(open(data_file, "r"), delimiter=",")
        track = np.array([data[:, 0], data[:, 1]]).T
        plt.plot(track[:, 0], track[:, 1])

    # ax.suptitle("Feature Track")
    ax.set_xlim([-1.0, 1.0])
    ax.set_ylim([-1.0, 1.0])
    ax.xaxis.tick_top()
    plt.gca().invert_yaxis()
    plt.show()
