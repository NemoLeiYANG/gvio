import sys
from os import listdir
from os.path import isfile, join

import numpy as np
import matplotlib.pylab as plt


def parse_track_files(data_path, cam0_K):
    files = [f for f in listdir(data_path) if isfile(join(data_path, f))]

    image_width = 752
    image_height = 480

    detection = np.zeros((image_height, image_width))

    for f in files:
        # Load track data
        data = np.loadtxt(open(join(data_path, f), "r"), delimiter=",")

        # Convert track data from ideal coordinates to image coordinates
        track_length = data.shape[0]
        track = np.array([data[:, 0], data[:, 1], np.ones(track_length)])
        track = np.dot(cam0_K, track)

        # Normalize data
        track[0, :] = track[0, :] / track[2, :]
        track[1, :] = track[1, :] / track[2, :]
        track = track.T

        for i in range(track.shape[0]):
            x, y, _ = track[i, :]

            if (x > 0 and x < image_width) and (y > 0 and y < image_height):
                detection[int(y), int(x)] += 1

    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # cax = ax.hist(detection)
    # fig.colorbar(cax)
    # plt.show()


def plot_tracker_stats(data_path):
    data = np.loadtxt(open(data_path, "r"), delimiter=",", skiprows=1)
    nb_frames = data.shape[0]

    plt.title("Tracker stats")
    plt.plot(range(1, nb_frames + 1), data[:, 0], label="Tracking")
    plt.plot(range(1, nb_frames + 1), data[:, 1], label="Lost")
    plt.show()


if __name__ == "__main__":
    data_path = sys.argv[1]
    cam0_K_file = sys.argv[2]
    stats_file = sys.argv[3]

    # cam0_K = np.loadtxt(open(join(data_path, cam0_K_file), "r"), delimiter=",")
    # parse_track_files(data_path, cam0_K)
    # plot_tracker_stats(stats_file)
