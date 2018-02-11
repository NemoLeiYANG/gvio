import sys
from math import floor
from math import ceil
from math import asin
from math import atan2

import numpy as np
import matplotlib.pylab as plt


def parse_data(data_path):
    # Check number of lines
    nb_lines = sum(1 for line in open(data_path))
    if nb_lines <= 2:
        print("No data in [" + data_path + "]?")
        return None

    # Load csv as numpy matrix
    data_file = open(data_path, "r")
    header = open(data_path, "r").readline()
    header = header[1:] if header[0] == "#" else header
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


def quat2euler(q, euler_seq):
    """Quaternion to Euler
    Parameters
    ----------
    q : np.array
        Quaternion (w, x, y, z)
    euler_seq : int
        Euler sequence
    Returns
    -------
    euler : np.array
        Euler angles
    """
    qw, qx, qy, qz = q
    qw2 = pow(qw, 2)
    qx2 = pow(qx, 2)
    qy2 = pow(qy, 2)
    qz2 = pow(qz, 2)

    if euler_seq == 123:
        t1 = atan2(2 * (qz * qw - qx * qy), (qw2 + qx2 - qy2 - qz2))
        t2 = asin(2 * (qx * qz + qy * qw))
        t3 = atan2(2 * (qx * qw - qy * qz), (qw2 - qx2 - qy2 + qz2))
        return np.array([t3, t2, t1])

    elif euler_seq == 321:
        t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2))
        t2 = asin(2 * (qy * qw - qx * qz))
        t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2))

        return np.array([t1, t2, t3])

    else:
        error_msg = "Error! Unsupported euler sequence [%s]" % str(euler_seq)
        raise RuntimeError(error_msg)


def convert_timestamps(timestamps):
    ts0 = timestamps[0]

    time = [0.0]
    for ts in timestamps[1:]:
        time.append((ts - ts0) * 1e-9)

    return time


def convert_quaternions(qw, qx, qy, qz):
    quaternions = []
    for i in range(len(qw)):
        q = [qw[i], qx[i], qy[i], qz[i]]
        quaternions.append(q)

    rpy = []
    for q in quaternions:
        rpy.append(quat2euler(q, 123))
    return np.array(rpy)


def plot_position(gnd_data):
    plt.figure()

    plt.subplot(211)
    plt.plot(gnd_data["p_RS_R_x [m]"], gnd_data["p_RS_R_y [m]"])
    plt.title("Position")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.axis('equal')

    plt.subplot(212)
    time = convert_timestamps(gnd_data["timestamp"])
    plt.plot(time, gnd_data["p_RS_R_z [m]"])

    z_min = floor(np.min(gnd_data["p_RS_R_z [m]"]))
    z_max = ceil(np.max(gnd_data["p_RS_R_z [m]"]))
    z_min = min(-1, z_min)
    z_max = max(1, z_max)

    plt.title("Altitude")
    plt.xlabel("Time (s)")
    plt.ylabel("Height (m)")
    plt.ylim([z_min, z_max])


def plot_attitude(gnd_data):
    time = convert_timestamps(gnd_data["timestamp"])
    rpy = convert_quaternions(gnd_data["q_RS_w []"],
                              gnd_data["q_RS_x []"],
                              gnd_data["q_RS_y []"],
                              gnd_data["q_RS_z []"])

    plt.figure()
    plt.suptitle("Attitude")

    plt.subplot(311)
    plt.plot(time, rpy[:, 0])
    plt.title("Roll")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")

    plt.subplot(312)
    plt.plot(time, rpy[:, 1])
    plt.title("Pitch")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")

    plt.subplot(313)
    plt.plot(time, rpy[:, 2])
    plt.title("Yaw")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")


def plot_measurements(mea_data):
    plt.figure()
    plt.title("Measurement")

    plt.subplot(211)
    plt.title("Accelerometer")
    plt.plot(mea_data["a_RS_S_x [m s^-2]"])
    plt.plot(mea_data["a_RS_S_y [m s^-2]"])
    plt.plot(mea_data["a_RS_S_z [m s^-2]"])

    plt.legend(loc=0)

    plt.subplot(212)
    plt.title("Gyroscope")
    plt.plot(mea_data["w_RS_S_x [rad s^-1]"])
    plt.plot(mea_data["w_RS_S_y [rad s^-1]"])
    plt.plot(mea_data["w_RS_S_z [rad s^-1]"])

    plt.legend(loc=0)


if __name__ == "__main__":
    base_dir = sys.argv[1]
    gnd_data = parse_data(base_dir + "/state_groundtruth_estimate0/data.csv")
    mea_data = parse_data(base_dir + "/imu0/data.csv")

    plot_position(gnd_data)
    plot_attitude(gnd_data)
    plot_measurements(mea_data)
    plt.show()
