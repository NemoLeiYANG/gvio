import sys

import numpy as np
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D  # NOQA


def parse_data(data_path):
    # Check number of lines
    nb_lines = sum(1 for line in open(data_path))
    if nb_lines <= 2:
        print("No data in [" + data_path + "]?")
        return None

    # Load csv as numpy matrix
    data_file = open(data_path, "r")
    header = open(data_path, "r").readline()
    data = np.loadtxt(data_file, delimiter=",", skiprows=1)
    data_file.close()

    # Convert numpy matrix as dictionary where each column is represented by
    # its header name
    data_dict = {}
    index = 0
    for element in header.split(","):
        data_dict[element.strip()] = data[:, index]
        index += 1

    return data_dict


def plot_position(est_data, gnd_data):
    plt.figure()

    # Plot X-Y position
    plt.subplot(211)
    if gnd_data:
        plt.plot(gnd_data["x"], gnd_data["y"], label="Ground truth")
    if est_data:
        plt.plot(est_data["x"], est_data["y"], label="Estimated")
    plt.title("Position")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.axis('equal')
    plt.legend(loc=0)

    # Plot Z position
    plt.subplot(212)
    if gnd_data:
        plt.plot(gnd_data["t"], gnd_data["z"], label="Ground Truth")
    if est_data:
        plt.plot(est_data["t"], est_data["z"], label="Estimated")

    plt.title("Altitude")
    plt.xlabel("Time (s)")
    plt.ylabel("Height (m)")
    # plt.ylim([-1, np.max(gnd_data["z"], est_data["z"])])
    plt.legend(loc=0)


def plot_attitude(est_data, gnd_data):
    plt.figure()
    plt.suptitle("Attitude")

    plt.subplot(311)
    if gnd_data:
        plt.plot(gnd_data["t"], gnd_data["roll"], label="Ground truth")
    if est_data:
        plt.plot(est_data["t"], est_data["roll"], label="Estimate")
    plt.title("Roll")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")
    plt.legend(loc=0)

    plt.subplot(312)
    if gnd_data:
        plt.plot(gnd_data["t"], gnd_data["pitch"], label="Ground truth")
    if est_data:
        plt.plot(est_data["t"], est_data["pitch"], label="Estimate")
    plt.title("Pitch")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")
    plt.legend(loc=0)

    plt.subplot(313)
    if gnd_data:
        plt.plot(gnd_data["t"], gnd_data["yaw"], label="Ground truth")
    if est_data:
        plt.plot(est_data["t"], est_data["yaw"], label="Estimate")
    plt.title("Yaw")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")
    plt.legend(loc=0)


def plot_measurements(mea_data):
    plt.figure()
    plt.title("Measurement")

    plt.subplot(211)
    plt.title("Accelerometer")
    plt.plot(mea_data["ax_B"], label="ax")
    plt.plot(mea_data["ay_B"], label="ay")
    plt.plot(mea_data["az_B"], label="az")

    plt.legend(loc=0)

    plt.subplot(212)
    plt.title("Gyroscope")
    plt.plot(mea_data["wx_B"], label="wx")
    plt.plot(mea_data["wy_B"], label="wy")
    plt.plot(mea_data["wz_B"], label="wz")

    plt.legend(loc=0)


class PlotSimWorld:
    def __init__(self, data_path):
        # Load ground truth data and cam0 index file
        self.gnd_data = parse_data(data_path + "/ground_truth.csv")
        self.cam0_index_data = parse_data(data_path + "/cam0/index.csv")
        self.max_index = len(self.cam0_index_data["t"]) - 1

        # Plot elements
        self.landmarks = None
        self.camera = None
        self.camera_trajectory = None

        # Initialize 3d plot
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')

        self.plot_landmarks()
        self.plot_camera_trajectory(0)
        self.plot_observed_landmarks(0)
        self.plot_camera(0)
        self.ax.set_xlim3d(-50, 50)
        self.ax.set_ylim3d(-50, 50)
        self.ax.set_zlim3d(-20, 20)
        plt.show(block=False)

    def plot_landmarks(self):
        features_file = data_path + "/features.csv"
        features_data = np.loadtxt(open(features_file, "r"), delimiter=",")
        self.ax.scatter(features_data[:, 0],
                        features_data[:, 1],
                        features_data[:, 2],
                        color="skyblue",
                        marker=".")

    def plot_camera_trajectory(self, index):
        if self.camera_trajectory:
            self.camera_trajectory.pop(0).remove()

        self.camera_trajectory = self.ax.plot(self.gnd_data["x"][:index],
                                              self.gnd_data["y"][:index],
                                              self.gnd_data["z"][:index],
                                              color="red")

    def plot_observed_landmarks(self, index):
        # Load data
        observe_file = "%d.csv" % int(self.cam0_index_data["frame_id"][index])
        cam0_path = data_path + "/cam0"
        cam0_observed = parse_data(cam0_path + "/" + observe_file)

        # Remove observed landmarks if it already exist in plot
        if self.landmarks:
            self.ax.collections.remove(self.landmarks)

        # Plot observed landmarks
        self.landmarks = self.ax.scatter(cam0_observed["lm_x"],
                                         cam0_observed["lm_y"],
                                         cam0_observed["lm_z"],
                                         color="yellow",
                                         marker="o")

    def plot_camera(self, index):
        # Remove camera if it already exist in plot
        if self.camera:
            self.ax.collections.remove(self.camera)

        # Plot camera position
        self.camera = self.ax.scatter([self.gnd_data["x"][index]],
                                      [self.gnd_data["y"][index]],
                                      [self.gnd_data["z"][index]],
                                      color="red",
                                      marker="o")

    def update(self, index):
        assert index <= self.max_index

        self.plot_camera_trajectory(index)
        self.plot_observed_landmarks(index)
        self.plot_camera(index)
        plt.show(block=False)


if __name__ == "__main__":
    data_path = sys.argv[1]
    est_data = parse_data(data_path + "/estimate.csv")
    mea_data = parse_data(data_path + "/measurements.csv")
    gnd_data = parse_data(data_path + "/ground_truth.csv")
    cam0_index_data = parse_data(data_path + "/cam0/index.csv")

    # plot_position(est_data, gnd_data)
    # plot_attitude(est_data, gnd_data)
    # plot_measurements(mea_data)

    # Plot features
    sim_world = PlotSimWorld(data_path)

    for i in range(sim_world.max_index):
        user_input = input("Press any key to step (q to quit):")
        if user_input == "q":
            break

        sim_world.update(i)
