import sys
import numpy as np
import matplotlib.pylab as plt


if __name__ == "__main__":
    data_file = sys.argv[1]
    image_width = int(sys.argv[2])
    image_height = int(sys.argv[3])

    data = np.loadtxt(open(data_file, "r"), delimiter=",")
    track = np.array([data[:, 0], data[:, 1]]).T

    plt.title("Feature Track")
    plt.plot(track[:, 0], track[:, 1])
    plt.xlim([0, image_width])
    plt.ylim([image_height, 0])
    plt.show()
