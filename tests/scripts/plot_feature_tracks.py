import sys
from os import listdir
from os.path import isfile, join

import numpy as np
import matplotlib.pylab as plt


if __name__ == "__main__":
    data_path = sys.argv[1]
    image_width = int(sys.argv[2])
    image_height = int(sys.argv[3])

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
    ax.set_xlim([0, image_width])
    ax.set_ylim([image_height, 0])
    ax.xaxis.tick_top()
    plt.show()
