import sys
import numpy as np
import matplotlib.pylab as plt


if __name__ == "__main__":
    X = np.loadtxt(open(sys.argv[1], "r"), delimiter=",")
    plt.matshow(X)
    plt.show()