import os
import sys

import numpy as np
import matplotlib.pylab as plt
import matplotlib.image as mpimg
from mpl_toolkits.axes_grid1 import make_axes_locatable


if __name__ == "__main__":
    data_path = sys.argv[1]

    grid_fast_file = os.path.join(data_path, "grid_fast.csv")
    fast_file = os.path.join(data_path, "fast.csv")

    grid_fast_img = mpimg.imread(os.path.join(data_path, "grid_fast.png"))
    fast_img = mpimg.imread(os.path.join(data_path, "fast.png"))

    grid_fast_data = np.loadtxt(open(grid_fast_file, "r"), delimiter=",")
    fast_data = np.loadtxt(open(fast_file, "r"), delimiter=",")

    # Calculate histogram
    # -- FAST
    x = fast_data[:, 0]
    y = fast_data[:, 1]
    fast_hist = np.histogram2d(x, y, bins=10)
    # -- Grid-FAST
    x = grid_fast_data[:, 0]
    y = grid_fast_data[:, 1]
    grid_fast_hist = np.histogram2d(x, y, bins=10)
    # -- Max value
    vmax = max(np.max(fast_hist[0]), np.max(grid_fast_hist[0]))

    fig, axarr = plt.subplots(2, 2)
    cmap = "Blues"

    img1 = axarr[0, 0].imshow(fast_img)
    axarr[0, 0].xaxis.tick_top()
    img2 = axarr[0, 1].imshow(grid_fast_img)
    axarr[0, 1].xaxis.tick_top()

    mat1 = axarr[1, 0].matshow(fast_hist[0].T, vmin=0, vmax=vmax, cmap=cmap)
    axarr[1, 0].xaxis.tick_top()
    divider1 = make_axes_locatable(axarr[1, 0])
    cax1 = divider1.append_axes("right", size="5%", pad=0.05)
    fig.colorbar(mat1, cax=cax1)

    mat2 = axarr[1, 1].matshow(grid_fast_hist[0].T, vmin=0, vmax=vmax, cmap=cmap)
    axarr[1, 1].xaxis.tick_top()
    divider2 = make_axes_locatable(axarr[1, 1])
    cax2 = divider2.append_axes("right", size="5%", pad=0.05)
    fig.colorbar(mat2, cax=cax2)

    axarr[0, 0].title.set_text('FAST Detection')
    axarr[0, 0].title.set_position([.5, 1.15])
    axarr[0, 1].title.set_text('Grid-FAST Detection')
    axarr[0, 1].title.set_position([.5, 1.15])
    axarr[1, 0].title.set_text('2D-Histogram of Grid-FAST Detection')
    axarr[1, 0].title.set_position([.5, 1.1])
    axarr[1, 1].title.set_text('2D-Histogram of FAST Detection')
    axarr[1, 1].title.set_position([.5, 1.1])

    plt.tight_layout(h_pad=1)
    plt.show()
