import sys
import argparse

import numpy as np
import matplotlib.pylab as plt


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--points_file', required=True)
    parser.add_argument('--pos_file', required=True)
    parser.add_argument('--vel_file')
    parser.add_argument('--acc_file')
    args = parser.parse_args()

    pos_data = np.loadtxt(open(args.pos_file, "r"), delimiter=",")
    points_data = np.loadtxt(open(args.points_file, "r"), delimiter=",")
    nb_plots = 1

    if args.vel_file:
        vel_data = np.loadtxt(open(args.vel_file, "r"), delimiter=",")
        nb_plots += 1

    if args.acc_file:
        acc_data = np.loadtxt(open(args.acc_file, "r"), delimiter=",")
        nb_plots += 1

    plt.subplot("%d11" % nb_plots)
    plt.plot(pos_data[:, 1], pos_data[:, 2])
    plt.scatter(points_data[:, 0], points_data[:, 1], marker="o", color="red")

    if nb_plots >= 2:
        plt.subplot("%d12" % nb_plots)
        plt.plot(vel_data[:, 0], vel_data[:, 1], label="x")
        plt.plot(vel_data[:, 0], vel_data[:, 2], label="y")
        plt.plot(vel_data[:, 0], vel_data[:, 3], label="z")
        plt.title("Velocity")
        plt.legend(loc=0)

    if nb_plots >= 3:
        plt.subplot("%d13" % nb_plots)
        plt.plot(acc_data[:, 0], acc_data[:, 1], label="x")
        plt.plot(acc_data[:, 0], acc_data[:, 2], label="y")
        plt.plot(acc_data[:, 0], acc_data[:, 3], label="z")
        plt.title("Acceleration")
        plt.legend(loc=0)

    plt.show()
