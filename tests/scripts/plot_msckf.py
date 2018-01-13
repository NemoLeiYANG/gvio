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


def plot_position(est_data, win_data, gnd_data):
    plt.figure()

    plt.plot(gnd_data["x"], gnd_data["y"], label="Ground truth")
    plt.plot(est_data["x"], est_data["y"], label="Estimated")
    if win_data:
        plt.plot(win_data["x"], win_data["y"], label="Camera states")

    plt.title("Position")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.axis('equal')
    plt.legend(loc=0)


def plot_attitude(est_data, gnd_data):
    plt.figure()
    plt.suptitle("Attitude")

    plt.subplot(311)
    plt.plot(gnd_data["t"], gnd_data["roll"], label="Ground truth")
    plt.plot(est_data["t"], est_data["roll"], label="Estimate")
    plt.title("Roll")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")
    plt.legend(loc=0)

    plt.subplot(312)
    plt.plot(gnd_data["t"], gnd_data["pitch"], label="Ground truth")
    plt.plot(est_data["t"], est_data["pitch"], label="Estimate")
    plt.title("Pitch")
    plt.xlabel("Time (s)")
    plt.ylabel("Attitude (radians)")
    plt.legend(loc=0)

    plt.subplot(313)
    plt.plot(gnd_data["t"], gnd_data["yaw"], label="Ground truth")
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
    plt.plot(mea_data["ax_B"])
    plt.plot(mea_data["ay_B"])
    plt.plot(mea_data["az_B"])

    plt.subplot(212)
    plt.title("Gyroscope")
    plt.plot(mea_data["wx_B"])
    plt.plot(mea_data["wy_B"])
    plt.plot(mea_data["wz_B"])


if __name__ == "__main__":
    est_data = parse_data("/tmp/test_msckf_predictionUpdate_est.dat")
    mea_data = parse_data("/tmp/test_msckf_predictionUpdate_mea.dat")
    gnd_data = parse_data("/tmp/test_msckf_predictionUpdate_gnd.dat")
    win_data = parse_data("/tmp/test_msckf_predictionUpdate_win.dat")

    plot_position(est_data, win_data, gnd_data)
    plot_attitude(est_data, gnd_data)
    plot_measurements(mea_data)
    plt.show()
