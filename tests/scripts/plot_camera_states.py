import sys
import numpy as np
import matplotlib.pylab as plt


if __name__ == "__main__":
    data = np.loadtxt(open(sys.argv[1], "r"), delimiter=",")

    cam_p_G = np.array([data[:, 0], data[:, 1], data[:, 2]]).T
    cam_q_CG = np.array([data[:, 3], data[:, 4], data[:, 5]]).T

    plt.title("Camera states")
    plt.plot(cam_p_G[:, 0], cam_p_G[:, 1])
    plt.show()
