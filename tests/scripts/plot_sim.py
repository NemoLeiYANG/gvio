import sys
from math import cos
from math import sin

import numpy as np
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D  # NOQA


def euler2rot(euler, euler_seq):
    """Convert euler to rotation matrix R
    This function assumes we are performing a body fixed intrinsic rotation.

    Source:

        Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
        Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
        Princeton University Press, 1999. Print.

        Page 86.

    Parameters
    ----------
    euler : np.array
        Euler angle (roll, pitch, yaw)
    euler_seq : float
        Euler rotation sequence

    Returns
    -------

        Rotation matrix (np.array)

    """
    if euler_seq == 321:  # i.e. ZYX rotation sequence (world to body)
        phi, theta, psi = euler

        R11 = cos(psi) * cos(theta)
        R21 = sin(psi) * cos(theta)
        R31 = -sin(theta)

        R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi)
        R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi)
        R32 = cos(theta) * sin(phi)

        R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)
        R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)
        R33 = cos(theta) * cos(phi)

    elif euler_seq == 123:  # i.e. XYZ rotation sequence (body to world)
        phi, theta, psi = euler

        R11 = cos(psi) * cos(theta)
        R21 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi)
        R31 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)

        R12 = sin(psi) * cos(theta)
        R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi)
        R32 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)

        R13 = -sin(theta)
        R23 = cos(theta) * sin(phi)
        R33 = cos(theta) * cos(phi)

    else:
        err_msg = "Error! Unsupported euler sequence [%s]" % str(euler_seq)
        raise RuntimeError(err_msg)

    return np.array([[R11, R12, R13],
                     [R21, R22, R23],
                     [R31, R32, R33]])


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
        # self.ax.set_xlim3d(-2, 2)
        # self.ax.set_ylim3d(-2, 2)
        # self.ax.set_zlim3d(-2, 2)
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")
        self.axis_equal_3dplot(self.ax)
        plt.show(block=False)

    def axis_equal_3dplot(self, ax):
        extents = np.array([getattr(ax, 'get_{}lim'.format(dim))()
                            for dim in 'xyz'])
        sz = extents[:, 1] - extents[:, 0]
        centers = np.mean(extents, axis=1)
        maxsize = max(abs(sz))
        r = maxsize / 2
        for ctr, dim in zip(centers, 'xyz'):
            getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)

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

        # Camera frame
        frame_sz = 0.5
        corners = np.array([[frame_sz, frame_sz, frame_sz],  # top left
                            [frame_sz, -frame_sz, frame_sz],  # top right
                            [frame_sz, -frame_sz, -frame_sz],  # bottom right
                            [frame_sz, frame_sz, -frame_sz]]).T  # bottom left
        rpy_G = np.array([self.gnd_data["roll"][index],
                          self.gnd_data["pitch"][index],
                          self.gnd_data["yaw"][index]])
        R_BG = euler2rot(rpy_G, 321)
        corners = np.dot(R_BG, corners)

        # Plot camera position
        center = np.array([self.gnd_data["x"][index],
                           self.gnd_data["y"][index],
                           self.gnd_data["z"][index]])
        self.camera = self.ax.scatter(center[0],
                                      center[1],
                                      center[2],
                                      color="red",
                                      marker="o")

        # Plot camera frame only once every 10 frames
        if (index % 5) != 0:
            return

        # Plot camera center to frame corners
        for i in range(4):
            self.ax.plot([center[0], center[0] + corners[0, i]],
                         [center[1], center[1] + corners[1, i]],
                         [center[2], center[2] + corners[2, i]],
                         color="red")

        # Plot frame corners to frame corners
        # -- Top left to top right
        self.ax.plot([center[0] + corners[0, 0], center[0] + corners[0, 1]],
                     [center[1] + corners[1, 0], center[1] + corners[1, 1]],
                     [center[2] + corners[2, 0], center[2] + corners[2, 1]],
                     color="red")
        # -- Top right to bottom right
        self.ax.plot([center[0] + corners[0, 1], center[0] + corners[0, 2]],
                     [center[1] + corners[1, 1], center[1] + corners[1, 2]],
                     [center[2] + corners[2, 1], center[2] + corners[2, 2]],
                     color="red")
        # -- Bottom right to bottom left
        self.ax.plot([center[0] + corners[0, 2], center[0] + corners[0, 3]],
                     [center[1] + corners[1, 2], center[1] + corners[1, 3]],
                     [center[2] + corners[2, 2], center[2] + corners[2, 3]],
                     color="red")
        # -- Bottom left to top left
        self.ax.plot([center[0] + corners[0, 3], center[0] + corners[0, 0]],
                     [center[1] + corners[1, 3], center[1] + corners[1, 0]],
                     [center[2] + corners[2, 3], center[2] + corners[2, 0]],
                     color="red")

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
